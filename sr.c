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
#define SEQSPACE 7      /* the min sequence space for GBN must be at least windowsize + 1 */
#define NOTINUSE (-1)   /* used to fill header fields that are not being used */

/* generic procedure to compute the checksum of a packet.  Used by both sender and receiver  
   the simulator will overwrite part of your packet with 'z's.  It will not overwrite your 
   original checksum.  This procedure must generate a different checksum to the original if
   the packet is corrupted.
*/
int ComputeChecksum(struct pkt packet)
{
  int checksum = 0;
  int i;

  checksum = packet.seqnum;
  checksum += packet.acknum;
  for ( i=0; i<20; i++ ) 
    checksum += (int)(packet.payload[i]);

  return checksum;
}

bool IsCorrupted(struct pkt packet)
{
  return packet.checksum != ComputeChecksum(packet);
}

/********* Sender (A) ************/

struct sender_packet {
  struct pkt packet;
  bool acked;
  bool valid;
  float send_time;
};

static struct sender_packet send_window[SEQSPACE];
static int base;
static int nextseqnum;

void A_output(struct msg message)
{
  if ((nextseqnum + SEQSPACE - base) % SEQSPACE < WINDOWSIZE) {
    struct pkt sendpkt;
    int i;

    sendpkt.seqnum = nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for (i = 0; i < 20; i++) 
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt);

    send_window[nextseqnum].packet = sendpkt;
    send_window[nextseqnum].acked = false;
    send_window[nextseqnum].valid = true;
    send_window[nextseqnum].send_time = get_sim_time();

    tolayer3(A, sendpkt);

    starttimer(A, TIMEOUT);

    nextseqnum = (nextseqnum + 1) % SEQSPACE;
  } else {
    window_full++;
  }
}

void A_input(struct pkt packet)
{
  if (!IsCorrupted(packet)) {
    total_ACKs_received++;

    int acknum = packet.acknum;
    if (send_window[acknum].valid && !send_window[acknum].acked) {
      send_window[acknum].acked = true;
      new_ACKs++;

      while (send_window[base].acked && send_window[base].valid) {
        send_window[base].valid = false;
        base = (base + 1) % SEQSPACE;
      }
    }
  }
}

void A_timerinterrupt(void)
{
  float current_time = get_sim_time();
  int i;

  for (i = 0; i < SEQSPACE; i++) {
    if (send_window[i].valid && !send_window[i].acked &&
        (current_time - send_window[i].send_time >= TIMEOUT)) {

      tolayer3(A, send_window[i].packet);
      send_window[i].send_time = current_time;
      packets_resent++;
    }
  }
  starttimer(A, TIMEOUT);
}

void A_init(void)
{
  base = 0;
  nextseqnum = 0;
  for (int i = 0; i < SEQSPACE; i++) {
    send_window[i].valid = false;
    send_window[i].acked = false;
  }
}

/********* Receiver (B) ************/

struct receiver_packet {
  struct pkt packet;
  bool received;
};

static struct receiver_packet recv_window[SEQSPACE];
static int expectedseqnum;

void B_input(struct pkt packet)
{
  struct pkt ackpkt;
  int i;

  if (!IsCorrupted(packet)) {
    int seq = packet.seqnum;

    ackpkt.acknum = seq;
    ackpkt.seqnum = 0;
    for (i = 0; i < 20; i++) 
      ackpkt.payload[i] = '0';
    ackpkt.checksum = ComputeChecksum(ackpkt);
    tolayer3(B, ackpkt);

    if (!recv_window[seq].received) {
      recv_window[seq].packet = packet;
      recv_window[seq].received = true;
    }

    while (recv_window[expectedseqnum].received) {
      tolayer5(B, recv_window[expectedseqnum].packet.payload);
      recv_window[expectedseqnum].received = false;
      expectedseqnum = (expectedseqnum + 1) % SEQSPACE;
    }
  } else {
    ackpkt.acknum = (expectedseqnum + SEQSPACE - 1) % SEQSPACE;
    ackpkt.seqnum = 0;
    for (i = 0; i < 20; i++) 
      ackpkt.payload[i] = '0';
    ackpkt.checksum = ComputeChecksum(ackpkt);
    tolayer3(B, ackpkt);
  }
}

void B_init(void)
{
  expectedseqnum = 0;
  for (int i = 0; i < SEQSPACE; i++) {
    recv_window[i].received = false;
  }
}

void B_output(struct msg message) {}
void B_timerinterrupt(void) {}
{
}

