#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "gbn.h"

#define RTT  16.0
#define TIMEOUT RTT
#define WINDOWSIZE 6
#define SEQSPACE 7
#define NOTINUSE (-1)

extern float get_sim_time();  // 提供模拟时间函数声明

struct SendPacket {
    struct pkt packet;
    bool acked;
    float send_time;
};

static struct SendPacket send_window[SEQSPACE];
static int base;
static int nextseqnum;

static int expectedseqnum;

int ComputeChecksum(struct pkt packet) {
    int checksum = 0;
    int i;
    checksum = packet.seqnum + packet.acknum;
    for (i = 0; i < 20; i++)
        checksum += (int)(packet.payload[i]);
    return checksum;
}

bool IsCorrupted(struct pkt packet) {
    return packet.checksum != ComputeChecksum(packet);
}

void A_output(struct msg message) {
    if ((nextseqnum + SEQSPACE - base) % SEQSPACE >= WINDOWSIZE) {
        if (TRACE > 0)
            printf("----A: window is full\n");
        window_full++;
        return;
    }

    struct pkt sendpkt;
    int i;
    sendpkt.seqnum = nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for (i = 0; i < 20; i++)
        sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt);

    send_window[nextseqnum].packet = sendpkt;
    send_window[nextseqnum].acked = false;
    send_window[nextseqnum].send_time = get_sim_time();

    tolayer3(A, sendpkt);
    if (TRACE > 0)
        printf("Sending packet %d to layer 3\n", sendpkt.seqnum);

    starttimer(A, TIMEOUT);
    nextseqnum = (nextseqnum + 1) % SEQSPACE;
}

void A_input(struct pkt packet) {
    int acknum = packet.acknum;
    if (!IsCorrupted(packet)) {
        if (TRACE > 0)
            printf("----A: uncorrupted ACK %d is received\n", acknum);
        total_ACKs_received++;
        if (!send_window[acknum].acked) {
            new_ACKs++;
            send_window[acknum].acked = true;
            while (send_window[base].acked && base != nextseqnum) {
                base = (base + 1) % SEQSPACE;
            }
        }
    } else {
        if (TRACE > 0)
            printf("----A: corrupted ACK is received, do nothing!\n");
    }
}

void A_timerinterrupt(void) {
    float current_time = get_sim_time();
    int i;
    if (TRACE > 0)
        printf("----A: time out, resend packets!\n");

    for (i = 0; i < SEQSPACE; i++) {
        if (!send_window[i].acked &&
            ((current_time - send_window[i].send_time) >= TIMEOUT)) {
            tolayer3(A, send_window[i].packet);
            if (TRACE > 0)
                printf("---A: resending packet %d\n", send_window[i].packet.seqnum);
            send_window[i].send_time = current_time;
            packets_resent++;
        }
    }
    starttimer(A, TIMEOUT);
}

void A_init(void) {
    int i;
    for (i = 0; i < SEQSPACE; i++) {
        send_window[i].acked = false;
    }
    base = 0;
    nextseqnum = 0;
}


/********* Receiver (B)  ************/

static struct pkt recv_buffer[SEQSPACE];
static bool received[SEQSPACE];

void B_input(struct pkt packet) {
    int i;
    struct pkt ackpkt;

    if (!IsCorrupted(packet)) {
        int seq = packet.seqnum;
        if (!received[seq]) {
            received[seq] = true;
            recv_buffer[seq] = packet;
            if (TRACE > 0)
                printf("----B: packet %d is correctly received, send ACK!\n", seq);
            packets_received++;
        } else {
            if (TRACE > 0)
                printf("----B: duplicate packet %d received, resend ACK!\n", seq);
        }

        ackpkt.seqnum = 0;
        ackpkt.acknum = seq;
        for (i = 0; i < 20; i++)
            ackpkt.payload[i] = '0';
        ackpkt.checksum = ComputeChecksum(ackpkt);
        tolayer3(B, ackpkt);

        while (received[expectedseqnum]) {
            tolayer5(B, recv_buffer[expectedseqnum].payload);
            received[expectedseqnum] = false;
            expectedseqnum = (expectedseqnum + 1) % SEQSPACE;
        }
    } else {
        if (TRACE > 0)
            printf("----B: corrupted packet received, ignored!\n");
    }
}

void B_init(void) {
    int i;
    expectedseqnum = 0;
    for (i = 0; i < SEQSPACE; i++)
        received[i] = false;
}

void B_output(struct msg message) {}
void B_timerinterrupt(void) {}
