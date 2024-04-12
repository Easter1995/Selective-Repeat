#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "datalink.h"
#include "protocol.h"

/* ���� */
#define MAX_SEQ 63
#define WINDOWSIZE ((MAX_SEQ + 1) / 2)

/* ��ʱ */
#define DATA_TIMER 3000
#define ACK_TIMER 300

/* buffer & seqno */
typedef unsigned char buffer_[PKT_LEN];
typedef unsigned char seqno_;

/* ACK���ƣ����շ����͵�ACKʵ�������������յ���һ��֡��� */

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

/* ֡�ṹ */
typedef struct {
	FRAME_KIND kind;
	seqno_ ack;
	seqno_ seq;
	buffer_ data;
	unsigned int padding;
}FRAME;

/* ��� */
static int phl_ready = 0;
static int has_send_nak = 0;

/* ָ����һ����� */
static seqno_ next(seqno_ x) {
	return x == MAX_SEQ ? 0 : x + 1;
}

/* ָ����һ����� */
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
	stop_ack_timer(); // ack���Ӵ�
	put_frame((unsigned char *)&s, 3 + PKT_LEN);

	start_timer(seq, DATA_TIMER);
}

/* ����ACK�ͷ���NAKһ���� */
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
			/* ��Ϊ���ͷ� */
		case NETWORK_LAYER_READY:
			get_packet(sending_wnd[NextSeq2Send % WINDOWSIZE]);
			send_data_frame(NextSeq2Send);
			// ���·��ʹ���
			NextSeq2Send = next(NextSeq2Send);
			sender_has_buffered++;
			break;

		case PHYSICAL_LAYER_READY:
			phl_ready = 1;
			break;
			
			/* ��Ϊ���շ� */
		case FRAME_RECEIVED:
			len = recv_frame((unsigned char *)&f, sizeof f);
			// ��֡��ȷ����ʱ�ŷ���nak
			if (len < 5 || crc32((unsigned char *)&f, len) != 0) {
				if (len > 5) {
					dbg_event("**** Receiver Error, Bad CRC Checksum\n");
					if (!has_send_nak) {
						// ���󵥶��ش���һ֡
						send_acknak_frame(FRAME_NAK, f.seq);
					}
				}
				break;
			}
			
			if (f.kind == FRAME_DATA) {
				// �ڽ��մ�����
				if (between(ExpectSeqno, f.seq, TooFar)) {
					//if (f.seq != ExpectSeqno && !has_send_nak) {
					//	send_acknak_frame(FRAME_NAK, ExpectSeqno); // ˵����һ֡��붪��
					//}
					// ��֤fδ����
					if (!has_arrived[f.seq % WINDOWSIZE]) {
						memcpy(receiving_wnd[f.seq % WINDOWSIZE], f.data, len - 7);
						has_arrived[f.seq % WINDOWSIZE] = 1;
						
						// �ɹ����뻺����
						// �з��ͷ������ٷ�����һ֡�ˣ���һ֡�Ѿ��ڻ�������
						if (f.seq != ExpectSeqno) {
							send_acknak_frame(FRAME_SEP_ACK, f.seq);
						}
						
						// ���ջ��������֡�����Ͻ������
						while (has_arrived[ExpectSeqno % WINDOWSIZE]) {
							put_packet(receiving_wnd[ExpectSeqno % WINDOWSIZE], PKT_LEN);
							has_arrived[ExpectSeqno % WINDOWSIZE] = 0;
							// ���½��մ���
							ExpectSeqno = next(ExpectSeqno);
							TooFar = next(TooFar);
							// ֡�ɹ�������
							start_ack_timer(ACK_TIMER);
							has_send_nak = 0;
						}
					}
				}
			}
			// ACK֡��piggybacking������һ����
			// �����¼��ע��prev(f.ack)����Ŀǰ���±����յ�֡��f.ack����һ�������յ���֡
			// ����ACK���ܽ������ѭ��
			if (f.kind == FRAME_ACK || f.kind == FRAME_DATA)
			{
				while (between(LastAckRcevd, prev(f.ack), NextSeq2Send)) {
					stop_timer(LastAckRcevd); // ֹͣ��ʱ��
					LastAckRcevd = next(LastAckRcevd);
					sender_has_buffered--;
				}
				if (f.ack == NextSeq2Send)
				{
					LastAckRcevd = f.ack;
				}
			}
			
			if (f.kind == FRAME_NAK) {
				// ���nakҪ���ش���֡�ڽ��ջ�������
				if (between(LastAckRcevd, f.ack, NextSeq2Send)) {
					// SRֻ���ش�һ֡
					send_data_frame(f.ack);
				}
			}
			if (f.kind == FRAME_SEP_ACK) {
				dbg_frame("Recv Sep ACK  %d\n", f.ack);
				stop_timer(f.ack); // ����ֹͣ��һ֡�ļ�ʱ��������������Ϣ����
			}
			break;

			/* ��ʱ�ش���ʱ���󶨵�֡ */
		case DATA_TIMEOUT:
			dbg_event("---- DATA %d timeout\n", arg);
			send_data_frame(arg);
			break;

			/* ��ʱ�ش�ack */
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