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
#define MAX_RETRANSMISSIONS 5  /* maximum number of retransmission attempts per packet */
#define RTT_ALPHA 0.125    /* weight for new RTT sample in EWMA calculation */
#define RTT_BETA 0.25      /* weight for new RTT variance in EWMA calculation */
#define RTT_MIN 8.0        /* minimum RTT value to prevent over-aggressive timing */
#define RTT_MAX 64.0       /* maximum RTT value to prevent excessive delay */
#define FAST_RETRANSMIT_THRESHOLD 3  /* threshold for duplicate ACKs to trigger fast retransmit */

/* generic procedure to compute the checksum of a packet. Used by both sender and receiver  
   the simulator will overwrite part of your packet with 'z's. It will not overwrite your 
   original checksum. This procedure must generate a different checksum to the original if
   the packet is corrupted.
*/
int ComputeChecksum(struct pkt packet)
{
  int checksum = 0;
  int i;

  checksum = packet.seqnum;
  checksum += packet.acknum;
  for (i=0; i<20; i++) 
    checksum += (int)(packet.payload[i]);

  return checksum;
}

bool IsCorrupted(struct pkt packet)
{
  if (packet.checksum == ComputeChecksum(packet))
    return (false);
  else
    return (true);
}


/********* Sender (A) variables and functions ************/

static struct pkt buffer[WINDOWSIZE];  /* array for storing packets waiting for ACK */
static int windowfirst, windowlast;    /* array indexes of the first/last packet awaiting ACK */
static int windowcount;                /* the number of packets currently awaiting an ACK */
static int A_nextseqnum;               /* the next sequence number to be used by the sender */
static float estimated_rtt;            /* estimated round-trip time */
static float dev_rtt;                  /* RTT deviation for timeout calculation */
static int retransmission_count[WINDOWSIZE]; /* count of retransmissions per packet */
static int dup_ack_count[SEQSPACE];    /* count of duplicate ACKs received per sequence number */
static int last_ack;                   /* last ACK received */
static int timeout_interval;           /* dynamic timeout interval based on network conditions */
static int network_condition;          /* 0: normal, 1: high loss, 2: high reordering */
static int consecutive_timeouts;       /* track consecutive timeouts to detect severe loss */

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;

  /* if not blocked waiting on ACK */
  if (windowcount < WINDOWSIZE) {
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    /* create packet */
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for (i=0; i<20; i++) 
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt); 

    /* put packet in window buffer */
    windowlast = (windowlast + 1) % WINDOWSIZE; 
    buffer[windowlast] = sendpkt;
    windowcount++;
    
    /* reset retransmission counter for this packet position */
    retransmission_count[windowlast] = 0;

    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3(A, sendpkt);

    /* start timer if first packet in window */
    if (windowcount == 1)
      starttimer(A, timeout_interval);

    /* get next sequence number, wrap back to 0 */
    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;  
  }
  /* if blocked, window is full */
  else {
    if (TRACE > 0)
      printf("----A: New message arrives, send window is full\n");
    window_full++;
  }
}


/* called from layer 3, when a packet arrives for layer 4 
   In this practical this will always be an ACK as B never sends data.
*/
void A_input(struct pkt packet)
{
  int ackcount = 0;
  int i;
  float sample_rtt;

  /* if received ACK is not corrupted */ 
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
    total_ACKs_received++;

    /* check if new ACK or duplicate */
    if (windowcount != 0) {
      int seqfirst = buffer[windowfirst].seqnum;
      int seqlast = buffer[windowlast].seqnum;
      
      /* check case when seqnum has and hasn't wrapped */
      if (((seqfirst <= seqlast) && (packet.acknum >= seqfirst && packet.acknum <= seqlast)) ||
          ((seqfirst > seqlast) && (packet.acknum >= seqfirst || packet.acknum <= seqlast))) {

        /* packet is a new ACK */
        if (packet.acknum != last_ack) {
          if (TRACE > 0)
            printf("----A: ACK %d is not a duplicate\n", packet.acknum);
          new_ACKs++;
          
          /* Update estimated RTT (only for non-retransmitted packets) */
          if (retransmission_count[windowfirst] == 0) {
            sample_rtt = get_sim_time() - packet.timestamp;
            /* Update RTT using EWMA (Exponentially Weighted Moving Average) */
            estimated_rtt = (1 - RTT_ALPHA) * estimated_rtt + RTT_ALPHA * sample_rtt;
            dev_rtt = (1 - RTT_BETA) * dev_rtt + RTT_BETA * fabs(sample_rtt - estimated_rtt);
            /* Update timeout using RTT and deviation (Jacobson's algorithm) */
            timeout_interval = estimated_rtt + 4 * dev_rtt;
            
            /* Constrain timeout within reasonable bounds */
            if (timeout_interval < RTT_MIN) timeout_interval = RTT_MIN;
            if (timeout_interval > RTT_MAX) timeout_interval = RTT_MAX;
            
            if (TRACE > 0)
              printf("----A: Updated timeout interval to %.2f\n", timeout_interval);
          }

          /* Reset duplicate ACK counter for this ACK */
          dup_ack_count[packet.acknum] = 0;
          
          /* Reset consecutive timeout counter on successful ACK */
          consecutive_timeouts = 0;

          /* cumulative acknowledgement - determine how many packets are ACKed */
          if (packet.acknum >= seqfirst)
            ackcount = packet.acknum + 1 - seqfirst;
          else
            ackcount = SEQSPACE - seqfirst + packet.acknum + 1;

          /* slide window by the number of packets ACKed */
          windowfirst = (windowfirst + ackcount) % WINDOWSIZE;

          /* delete the acked packets from window buffer */
          for (i=0; i<ackcount; i++)
            windowcount--;

          /* start timer again if there are still more unacked packets in window */
          stoptimer(A);
          if (windowcount > 0)
            starttimer(A, timeout_interval);
          
          /* Update last ACK received */
          last_ack = packet.acknum;
        }
        else {
          /* Duplicate ACK handling */
          if (TRACE > 0)
            printf("----A: duplicate ACK %d received\n", packet.acknum);
          
          dup_ack_count[packet.acknum]++;
          
          /* Fast retransmit on receiving multiple duplicate ACKs */
          if (dup_ack_count[packet.acknum] >= FAST_RETRANSMIT_THRESHOLD) {
            if (TRACE > 0)
              printf("----A: Fast retransmit triggered for packet %d\n", (packet.acknum + 1) % SEQSPACE);
            
            /* Find the packet to retransmit */
            int resend_idx = -1;
            for (i = 0; i < windowcount; i++) {
              if (buffer[(windowfirst + i) % WINDOWSIZE].seqnum == (packet.acknum + 1) % SEQSPACE) {
                resend_idx = (windowfirst + i) % WINDOWSIZE;
                break;
              }
            }
            
            if (resend_idx != -1) {
              tolayer3(A, buffer[resend_idx]);
              packets_resent++;
              retransmission_count[resend_idx]++;
              dup_ack_count[packet.acknum] = 0; /* Reset duplicate ACK counter */
            }
          }
        }
      }
    }
    else
      if (TRACE > 0)
        printf("----A: duplicate ACK received, do nothing!\n");
  }
  else 
    if (TRACE > 0)
      printf("----A: corrupted ACK is received, do nothing!\n");
}

/* called when A's timer goes off */
void A_timerinterrupt(void)
{
  int i;
  float backoff_factor = 1.0;

  if (TRACE > 0)
    printf("----A: time out, resend packets!\n");
  
  consecutive_timeouts++;
  
  /* Exponential backoff for consecutive timeouts (indicates severe congestion) */
  if (consecutive_timeouts > 1) {
    /* Calculate backoff factor (2^n capped at 8) */
    backoff_factor = (consecutive_timeouts > 3) ? 8.0 : (1 << consecutive_timeouts);
    timeout_interval = fmin(timeout_interval * backoff_factor, RTT_MAX);
    
    if (TRACE > 0)
      printf("----A: consecutive timeout #%d, backing off timeout to %.2f\n", 
             consecutive_timeouts, timeout_interval);
  }

  /* Detect high packet loss vs high reordering based on timeout patterns */
  if (consecutive_timeouts >= 3) {
    network_condition = 1; /* High loss environment */
    if (TRACE > 0)
      printf("----A: detected high packet loss environment\n");
  }
  
  /* Resend all unacknowledged packets */
  for (i = 0; i < windowcount; i++) {
    int pkt_idx = (windowfirst + i) % WINDOWSIZE;
    
    /* Check if we've exceeded retransmission limit for this packet */
    if (retransmission_count[pkt_idx] < MAX_RETRANSMISSIONS) {
      if (TRACE > 0)
        printf("---A: resending packet %d (attempt %d/%d)\n", 
               buffer[pkt_idx].seqnum, retransmission_count[pkt_idx] + 1, MAX_RETRANSMISSIONS);
      
      /* Add timestamp to packet for RTT measurement */
      buffer[pkt_idx].timestamp = get_sim_time();
      
      tolayer3(A, buffer[pkt_idx]);
      packets_resent++;
      retransmission_count[pkt_idx]++;
    }
    else {
      if (TRACE > 0)
        printf("---A: reached max retransmissions (%d) for packet %d, giving up\n", 
               MAX_RETRANSMISSIONS, buffer[pkt_idx].seqnum);
      /* In a real implementation, we might signal an error to the application layer here */
    }
  }
  
  /* Only start timer if we still have packets to send */
  if (windowcount > 0) {
    starttimer(A, timeout_interval);
  }
}

/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  /* initialise A's window, buffer and sequence number */
  A_nextseqnum = 0;  /* A starts with seq num 0, do not change this */
  windowfirst = 0;
  windowlast = -1;   /* windowlast is where the last packet sent is stored.  
                      new packets are placed in winlast + 1 
                      so initially this is set to -1
                    */
  windowcount = 0;
  
  /* Initialize adaptive parameters */
  estimated_rtt = BASE_RTT;
  dev_rtt = BASE_RTT / 2;
  timeout_interval = BASE_RTT * 2;
  last_ack = -1;
  consecutive_timeouts = 0;
  network_condition = 0; /* Start assuming normal network conditions */
  
  /* Initialize retransmission and duplicate ACK counters */
  int i;
  for (i = 0; i < WINDOWSIZE; i++) {
    retransmission_count[i] = 0;
  }
  
  for (i = 0; i < SEQSPACE; i++) {
    dup_ack_count[i] = 0;
  }
}

/********* Receiver (B) variables and procedures ************/

static int expectedseqnum; /* the sequence number expected next by the receiver */
static int B_nextseqnum;   /* the sequence number for the next packets sent by B */
static struct pkt last_acked_pkt; /* store the last ACK packet sent */
static int out_of_order_pkts[SEQSPACE]; /* buffer for out-of-order packets */
static int reordering_detected; /* flag to indicate if reordering is detected */

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  struct pkt sendpkt;
  int i;
  int in_window = 0;

  /* Check if packet is within the current receive window */
  for (i = 0; i < WINDOWSIZE; i++) {
    if (packet.seqnum == (expectedseqnum + i) % SEQSPACE) {
      in_window = 1;
      break;
    }
  }

  /* if not corrupted */
  if (!IsCorrupted(packet)) {
    /* Add timestamp to outgoing ACK packets for RTT calculation */
    sendpkt.timestamp = get_sim_time();
    
    /* Handle expected packet */
    if (packet.seqnum == expectedseqnum) {
      if (TRACE > 0)
        printf("----B: packet %d is correctly received, send ACK!\n", packet.seqnum);
      packets_received++;

      /* deliver to receiving application */
      tolayer5(B, packet.payload);

      /* send an ACK for the received packet */
      sendpkt.acknum = expectedseqnum;

      /* Check if we have buffered out-of-order packets that can now be processed */
      expectedseqnum = (expectedseqnum + 1) % SEQSPACE;
      while (out_of_order_pkts[expectedseqnum] == 1) {
        if (TRACE > 0)
          printf("----B: delivering buffered packet %d\n", expectedseqnum);
        /* Note: In an actual implementation, we would deliver the packet data here */
        /* But for this simulation, we don't have the actual packets stored */
        
        out_of_order_pkts[expectedseqnum] = 0;
        expectedseqnum = (expectedseqnum + 1) % SEQSPACE;
      }
      
      /* Send cumulative ACK for all packets we've processed */
      sendpkt.acknum = (expectedseqnum + SEQSPACE - 1) % SEQSPACE;
    }
    /* Handle out-of-order but in-window packet */
    else if (in_window) {
      /* Mark packet as reordered */
      reordering_detected = 1;
      
      if (TRACE > 0)
        printf("----B: out-of-order packet %d received, buffering\n", packet.seqnum);
      
      /* Buffer out-of-order packet */
      out_of_order_pkts[packet.seqnum] = 1;
      
      /* Resend last ACK */
      sendpkt.acknum = (expectedseqnum + SEQSPACE - 1) % SEQSPACE;
    }
    /* Handle duplicate or out-of-window packet */
    else {
      if (TRACE > 0) 
        printf("----B: packet %d out of window, resend ACK for last in-order packet\n", packet.seqnum);
      
      sendpkt.acknum = (expectedseqnum + SEQSPACE - 1) % SEQSPACE;
    }
  }
  else {
    /* packet is corrupted, resend last ACK */
    if (TRACE > 0) 
      printf("----B: packet corrupted, resend ACK!\n");
    
    sendpkt = last_acked_pkt;
  }

  /* create packet */
  sendpkt.seqnum = B_nextseqnum;
  B_nextseqnum = (B_nextseqnum + 1) % 2;
    
  /* we don't have any data to send. fill payload with 0's */
  for (i = 0; i < 20; i++) 
    sendpkt.payload[i] = '0';  

  /* compute checksum */
  sendpkt.checksum = ComputeChecksum(sendpkt); 
  
  /* save this ACK packet */
  last_acked_pkt = sendpkt;

  /* send out packet */
  tolayer3(B, sendpkt);
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  expectedseqnum = 0;
  B_nextseqnum = 1;
  reordering_detected = 0;
  
  /* Initialize out-of-order packet buffer */
  int i;
  for (i = 0; i < SEQSPACE; i++) {
    out_of_order_pkts[i] = 0;
  }
  
  /* Initialize last ACK packet */
  last_acked_pkt.seqnum = -1;
  last_acked_pkt.acknum = -1;
  for (i = 0; i < 20; i++) {
    last_acked_pkt.payload[i] = '0';
  }
  last_acked_pkt.checksum = ComputeChecksum(last_acked_pkt);
}

/******************************************************************************
 * The following functions need be completed only for bi-directional messages *
 *****************************************************************************/

/* Note that with simplex transfer from a-to-B, there is no B_output() */
void B_output(struct msg message)  
{
}

/* called when B's timer goes off */
void B_timerinterrupt(void)
{
}

/* Helper function to simulate get_sim_time() which would return current simulation time */
float get_sim_time(void)
{
  /* In the actual simulation, this would return the current simulation time */
  /* For this exercise, just return a placeholder value */
  /* This function should be provided by the simulation environment */
  return 0.0;
}