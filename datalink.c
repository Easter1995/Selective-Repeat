#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "datalink.h"
#include "protocol.h"

/* 窗口 */
#define MAX_SEQ 63
#define WINDOWSIZE ((MAX_SEQ + 1) / 2)

/* 超时 */
#define DATA_TIMER 3000
#define ACK_TIMER 300

/* buffer & seqno */
typedef unsigned char buffer_[PKT_LEN];
typedef unsigned char seqno_;

/* ACK机制：接收方发送的ACK实际上是期望接收的下一个帧序号 */

/* sender */
/* sendingWindow:[LastAckRcevd, NextSeq2Send) */
static seqno_ LastAckRcevd = 0;
static seqno_ NextSeq2Send = 0;
/* sending buffer */
static buffer_ sending_wnd[WINDOWSIZE];
static seqno_ sender_has_buffered = 0;

/* receiver */
/* receivingWindow:[ExpectSeqno, TooFar) */
static seqno_ ExpectSeqno = 0;
static seqno_ TooFar = WINDOWSIZE;
/* receiving buffer */
static buffer_ receiving_wnd[WINDOWSIZE];
static int has_arrived[WINDOWSIZE];

/* 帧结构 */
typedef struct {
	FRAME_KIND kind;
	seqno_ ack;
	seqno_ seq;
	buffer_ data;
	unsigned int padding;
}FRAME;

/* 标记 */
static int phl_ready = 0;
static int has_send_nak = 0;

/* 指向下一个序号 */
static seqno_ next(seqno_ x) {
	return x == MAX_SEQ ? 0 : x + 1;
}

/* 指向上一个序号 */
static seqno_ prev(seqno_ x) {
	return x == 0 ? MAX_SEQ : x - 1;
}

static int between(seqno_ begin, seqno_ t, seqno_ end) {
	return (t >= begin && t < end)
		|| (begin > end && t >= begin)
		|| (begin > end && t < end);
}

static void put_frame(unsigned char *frame, int len) {
	*(unsigned int *)(frame + len) = crc32(frame, len);
	send_frame(frame, len + 4);
	phl_ready = 0;
}

static void send_data_frame(seqno_ seq) {
	FRAME s;

	s.kind = FRAME_DATA;
	s.seq = seq;
	s.ack = ExpectSeqno;
	memcpy(s.data, sending_wnd[seq % WINDOWSIZE], PKT_LEN);

	dbg_frame("Send DATA %d %d, ID %d\n", s.seq, s.ack, *(short *)s.data);
	stop_ack_timer(); // ack被捎带
	put_frame((unsigned char *)&s, 3 + PKT_LEN);

	start_timer(seq, DATA_TIMER);
}

/* 发送ACK和发送NAK一起处理 */
static void send_acknak_frame(FRAME_KIND fk, seqno_ seq) {
	FRAME s;

	s.kind = FRAME_ACK;
	s.ack = ExpectSeqno;

	dbg_frame("Send ACK  %d\n", s.ack);

	put_frame((unsigned char *)&s, 2);
	if (fk == FRAME_NAK)
	{
		has_send_nak = 1;
		stop_ack_timer();
	}
}


int main(int argc, char **argv) {
	protocol_init(argc, argv);
	lprintf("Selective Repeat, Designed by ORANGEC, build: " __DATE__ "  "__TIME__"\n");
	disable_network_layer();
	
	while(1) {
		int arg, len;
		int event = wait_for_event(&arg);
		FRAME f;

		switch (event) {
			/* 作为发送方 */
		case NETWORK_LAYER_READY:
			get_packet(sending_wnd[NextSeq2Send % WINDOWSIZE]);
			send_data_frame(NextSeq2Send);
			// 更新发送窗口
			NextSeq2Send = next(NextSeq2Send);
			sender_has_buffered++;
			break;

		case PHYSICAL_LAYER_READY:
			phl_ready = 1;
			break;
			
			/* 作为接收方 */
		case FRAME_RECEIVED:
			len = recv_frame((unsigned char *)&f, sizeof f);
			// 当帧明确出错时才发送nak
			if (len < 5 || crc32((unsigned char *)&f, len) != 0) {
				if (len > 5) {
					dbg_event("**** Receiver Error, Bad CRC Checksum\n");
					if (!has_send_nak) {
						// 请求单独重传这一帧
						send_acknak_frame(FRAME_NAK, f.seq);
					}
				}
				break;
			}
			
			if (f.kind == FRAME_DATA) {
				// 在接收窗口内
				if (between(ExpectSeqno, f.seq, TooFar)) {
					//if (f.seq != ExpectSeqno && !has_send_nak) {
					//	send_acknak_frame(FRAME_NAK, ExpectSeqno); // 说明这一帧多半丢了
					//}
					// 保证f未接收
					if (!has_arrived[f.seq % WINDOWSIZE]) {
						memcpy(receiving_wnd[f.seq % WINDOWSIZE], f.data, len - 7);
						has_arrived[f.seq % WINDOWSIZE] = 1;
						
						// 成功存入缓存区
						// 叫发送方不用再发送这一帧了，这一帧已经在缓存区了
						if (f.seq != ExpectSeqno) {
							send_acknak_frame(FRAME_SEP_ACK, f.seq);
						}
						
						// 接收缓存区里的帧按序上交网络层
						while (has_arrived[ExpectSeqno % WINDOWSIZE]) {
							put_packet(receiving_wnd[ExpectSeqno % WINDOWSIZE], PKT_LEN);
							has_arrived[ExpectSeqno % WINDOWSIZE] = 0;
							// 更新接收窗口
							ExpectSeqno = next(ExpectSeqno);
							TooFar = next(TooFar);
							// 帧成功被接受
							start_ack_timer(ACK_TIMER);
							has_send_nak = 0;
						}
					}
				}
			}
			// ACK帧和piggybacking在这里一起处理，
			// 错误记录：注意prev(f.ack)才是目前最新被接收的帧，f.ack是下一次期望收到的帧
			// 单独ACK不能进入这个循环
			if (f.kind == FRAME_ACK || f.kind == FRAME_DATA)
			{
				while (between(LastAckRcevd, prev(f.ack), NextSeq2Send)) {
					stop_timer(LastAckRcevd); // 停止计时器
					LastAckRcevd = next(LastAckRcevd);
					sender_has_buffered--;
				}
				if (f.ack == NextSeq2Send)
				{
					LastAckRcevd = f.ack;
				}
			}
			
			if (f.kind == FRAME_NAK) {
				// 如果nak要求重传的帧在接收缓存区内
				if (between(LastAckRcevd, f.ack, NextSeq2Send)) {
					// SR只用重传一帧
					send_data_frame(f.ack);
				}
			}
			if (f.kind == FRAME_SEP_ACK) {
				dbg_frame("Recv Sep ACK  %d\n", f.ack);
				stop_timer(f.ack); // 单独停止这一帧的计时器，避免冗余信息过多
			}
			break;

			/* 超时重传定时器绑定的帧 */
		case DATA_TIMEOUT:
			dbg_event("---- DATA %d timeout\n", arg);
			send_data_frame(arg);
			break;

			/* 超时重传ack */
		case ACK_TIMEOUT:
			dbg_event("---- ACK timeout %d\n", ExpectSeqno);
			send_acknak_frame(FRAME_ACK, ExpectSeqno);
			break;
		}

		if (sender_has_buffered < WINDOWSIZE && phl_ready) {
			enable_network_layer();
		}
		else {
			disable_network_layer();
		}
	}
}