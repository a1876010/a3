#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "gbn.h"

/* ******************************************************************
   Go Back N protocol.  Adapted from J.F.Kurose
   ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.2  

   Network properties:
   - one way network delay averages five time units (longer if there
   are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
   or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
   (although some can be lost).

   Modifications: 
   - removed bidirectional GBN code and other code not used by prac. 
   - fixed C style to adhere to current programming style
   - added GBN implementation
**********************************************************************/

#define RTT  16.0       /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6    /* the maximum number of buffered unacked packet */
#define SEQSPACE 7       /* the min sequence space for GBN must be at least windowsize + 1 */
#define NOTINUSE (-1)    /* used to fill header fields that are not being used */

int ComputeChecksum(struct pkt packet) {
  int checksum = 0;
  for (int i = 0; i < 20; i++)
    checksum += (int)(packet.payload[i]);
  checksum += packet.seqnum + packet.acknum;
  return checksum;
}

bool IsCorrupted(struct pkt packet) {
  return packet.checksum != ComputeChecksum(packet);
}

/********* Sender (A) variables and functions ************/
static struct pkt buffer[WINDOWSIZE];
static int windowfirst, windowlast;
static int windowcount;
static int A_nextseqnum;

/* Tuning related variables */
static float estimatedRTT = RTT;
static float devRTT = 0.0;
static float alpha = 0.125;
static float beta = 0.25;
static float timeoutInterval = RTT;
static float send_times[WINDOWSIZE];
static int retransmit_count[WINDOWSIZE];
static int MAX_RETRIES = 5;

extern float get_sim_time();  

void SetMaxRetries(int level) {
  int retries[3] = {5, 10, 15};  
  MAX_RETRIES = retries[level];
  for (int i = 0; i < WINDOWSIZE; i++)
    retransmit_count[i] = 0;
}

void A_output(struct msg message) {
  struct pkt sendpkt;
  int i;
  if (windowcount < WINDOWSIZE) {
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new message to layer3!\n");

    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for (i = 0; i < 20; i++)
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt);

    windowlast = (windowlast + 1) % WINDOWSIZE;
    buffer[windowlast] = sendpkt;
    windowcount++;
    retransmit_count[windowlast] = 0;
    send_times[windowlast] = get_sim_time();

    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3(A, sendpkt);

    if (windowcount == 1)
      starttimer(A, timeoutInterval);

    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;
  } else {
    if (TRACE > 0)
      printf("----A: New message arrives, send window is full\n");
    window_full++;
  }
}

void A_input(struct pkt packet) {
  int ackcount = 0;
  int i;

  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
    total_ACKs_received++;

    if (windowcount != 0) {
      int seqfirst = buffer[windowfirst].seqnum;
      int seqlast = buffer[windowlast].seqnum;
      if (((seqfirst <= seqlast) && (packet.acknum >= seqfirst && packet.acknum <= seqlast)) ||
          ((seqfirst > seqlast) && (packet.acknum >= seqfirst || packet.acknum <= seqlast))) {

        if (TRACE > 0)
          printf("----A: ACK %d is not a duplicate\n", packet.acknum);
        new_ACKs++;

        float sampleRTT = get_sim_time() - send_times[windowfirst];
        estimatedRTT = (1 - alpha) * estimatedRTT + alpha * sampleRTT;
        devRTT = (1 - beta) * devRTT + beta * fabs(sampleRTT - estimatedRTT);
        timeoutInterval = estimatedRTT + 4 * devRTT;

        if (packet.acknum >= seqfirst)
          ackcount = packet.acknum + 1 - seqfirst;
        else
          ackcount = SEQSPACE - seqfirst + packet.acknum;

        windowfirst = (windowfirst + ackcount) % WINDOWSIZE;

        for (i = 0; i < ackcount; i++)
          windowcount--;

        stoptimer(A);
        if (windowcount > 0)
          starttimer(A, timeoutInterval);
      }
    } else {
      if (TRACE > 0)
        printf("----A: duplicate ACK received, do nothing!\n");
    }
  } else if (TRACE > 0) {
    printf("----A: corrupted ACK is received, do nothing!\n");
  }
}

void A_timerinterrupt(void) {
  if (TRACE > 0)
    printf("----A: timeout, resend packets!\n");

  for (int i = 0; i < windowcount; i++) {
    int index = (windowfirst + i) % WINDOWSIZE;
    if (retransmit_count[index] < MAX_RETRIES) {
      if (TRACE > 0)
        printf("---A: resending packet %d\n", buffer[index].seqnum);
      tolayer3(A, buffer[index]);
      retransmit_count[index]++;
      send_times[index] = get_sim_time();
      packets_resent++;
    } else {
      if (TRACE > 0)
        printf("---A: packet %d exceeds retry limit, dropped\n", buffer[index].seqnum);
    }
  }
  starttimer(A, timeoutInterval);
}

void A_init(void) {
  A_nextseqnum = 0;
  windowfirst = 0;
  windowlast = -1;
  windowcount = 0;

  estimatedRTT = RTT;
  devRTT = 0.0;
  timeoutInterval = RTT;

  for (int i = 0; i < WINDOWSIZE; i++) {
    retransmit_count[i] = 0;
    send_times[i] = 0.0;
  }

  SetMaxRetries(0);  
}

/********* Receiver (B)  variables and procedures ************/
static int expectedseqnum;
static int B_nextseqnum;

void B_input(struct pkt packet) {
  struct pkt sendpkt;
  int i;

  if ((!IsCorrupted(packet)) && (packet.seqnum == expectedseqnum)) {
    if (TRACE > 0)
      printf("----B: packet %d is correctly received, send ACK!\n", packet.seqnum);
    packets_received++;

    tolayer5(B, packet.payload);
    sendpkt.acknum = expectedseqnum;
    expectedseqnum = (expectedseqnum + 1) % SEQSPACE;
  } else {
    if (TRACE > 0)
      printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");
    sendpkt.acknum = (expectedseqnum == 0) ? SEQSPACE - 1 : expectedseqnum - 1;
  }

  sendpkt.seqnum = B_nextseqnum;
  B_nextseqnum = (B_nextseqnum + 1) % 2;

  for (i = 0; i < 20; i++)
    sendpkt.payload[i] = '0';

  sendpkt.checksum = ComputeChecksum(sendpkt);
  tolayer3(B, sendpkt);
}

void B_init(void) {
  expectedseqnum = 0;
  B_nextseqnum = 1;
}

void B_output(struct msg message) {}
void B_timerinterrupt(void) {}