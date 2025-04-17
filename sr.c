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
  int i;
  checksum = packet.seqnum;
  checksum += packet.acknum;
  for (i = 0; i < 20; i++) 
    checksum += (int)(packet.payload[i]);
  return checksum;
}

bool IsCorrupted(struct pkt packet) {
  return packet.checksum != ComputeChecksum(packet);
}

/********* Sender (A) variables and functions ************/

static struct pkt buffer[WINDOWSIZE];
static float send_times[WINDOWSIZE];
static int retry_count[WINDOWSIZE];

static int windowfirst, windowlast;
static int windowcount;
static int A_nextseqnum;

static float estimatedRTT = RTT;
static float devRTT = 0;
static float timeoutInterval = RTT;
static float alpha = 0.125;
static float beta = 0.25;
#define MAX_RETRIES 10

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

    tolayer3(A, sendpkt);
    send_times[windowlast] = get_sim_time();
    retry_count[windowlast] = 0;

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

        if (packet.acknum >= seqfirst)
          ackcount = packet.acknum + 1 - seqfirst;
        else
          ackcount = SEQSPACE - seqfirst + packet.acknum;

        float sampleRTT = get_sim_time() - send_times[windowfirst];
        estimatedRTT = (1 - alpha) * estimatedRTT + alpha * sampleRTT;
        devRTT = (1 - beta) * devRTT + beta * fabs(sampleRTT - estimatedRTT);
        timeoutInterval = estimatedRTT + 4 * devRTT;

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
  } else {
    if (TRACE > 0)
      printf("----A: corrupted ACK is received, do nothing!\n");
  }
}

void A_timerinterrupt(void) {
  int i;
  if (TRACE > 0)
    printf("----A: time out, resend packets!\n");

  for (i = 0; i < windowcount; i++) {
    int idx = (windowfirst + i) % WINDOWSIZE;
    if (retry_count[idx] < MAX_RETRIES) {
      tolayer3(A, buffer[idx]);
      retry_count[idx]++;
      packets_resent++;
      send_times[idx] = get_sim_time();
      if (i == 0) starttimer(A, timeoutInterval);
    } else {
      if (TRACE > 0)
        printf("---A: packet %d reached max retries, giving up.\n", buffer[idx].seqnum);
    }
  }
}

void A_init(void) {
  A_nextseqnum = 0;
  windowfirst = 0;
  windowlast = -1;
  windowcount = 0;
  timeoutInterval = RTT;
}

/********* Receiver (B) ************/

static int expectedseqnum;
static int B_nextseqnum;

void B_input(struct pkt packet) {
  struct pkt sendpkt;
  int i;

  if (!IsCorrupted(packet) && packet.seqnum == expectedseqnum) {
    if (TRACE > 0)
      printf("----B: packet %d is correctly received, send ACK!\n", packet.seqnum);
    packets_received++;

    tolayer5(B, packet.payload);
    sendpkt.acknum = expectedseqnum;
    expectedseqnum = (expectedseqnum + 1) % SEQSPACE;
  } else {
    if (TRACE > 0)
      printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");
    if (expectedseqnum == 0)
      sendpkt.acknum = SEQSPACE - 1;
    else
      sendpkt.acknum = expectedseqnum - 1;
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

/********* Additional Tuning API ************/

void SetAlphaBeta(float new_alpha, float new_beta) {
  alpha = new_alpha;
  beta = new_beta;
}

void SetMaxRetries(int retries) {
  for (int i = 0; i < WINDOWSIZE; i++)
    retry_count[i] = 0;
  #undef MAX_RETRIES
  #define MAX_RETRIES retries
}
