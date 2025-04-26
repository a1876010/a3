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
static bool acked[WINDOWSIZE];         /* tracking which packets have been ACKed */

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;

  /* if not blocked waiting on ACK */
  if ( windowcount < WINDOWSIZE) {
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    /* create packet */
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for ( i=0; i<20 ; i++ ) 
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt); 

    /* put packet in window buffer */
    windowlast = (windowfirst + windowcount) % WINDOWSIZE; 
    buffer[windowlast] = sendpkt;
    acked[windowlast] = false;  /* Mark as not yet acknowledged */
    windowcount++;

    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3 (A, sendpkt);

    /* start timer if first packet in window */
    if (windowcount == 1)
      starttimer(A, RTT);

    /* get next sequence number, wrap back to 0 */
    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;  
  }
  /* if blocked,  window is full */
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
  int i;
  int seqnum;
  int bufIdx = -1;

  /* if received ACK is not corrupted */ 
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
    total_ACKs_received++;
    
    seqnum = packet.acknum;

    /* Find the packet with this sequence number in our window */
    for (i = 0; i < windowcount; i++) {
      int idx = (windowfirst + i) % WINDOWSIZE;
      if (buffer[idx].seqnum == seqnum) {
        bufIdx = idx;
        break;
      }
    }

    /* If we found the packet and it hasn't been ACKed yet */
    if (bufIdx != -1 && !acked[bufIdx]) {
      if (TRACE > 0)
        printf("----A: ACK %d is not a duplicate\n", seqnum);
      
      /* Mark this packet as acknowledged */
      acked[bufIdx] = true;
      new_ACKs++;
      
      /* Check if we can slide the window */
      if (bufIdx == windowfirst) {
        /* Stop the current timer */
        stoptimer(A);
        
        /* Slide window past all consecutively ACKed packets */
        while (windowcount > 0 && acked[windowfirst]) {
          windowfirst = (windowfirst + 1) % WINDOWSIZE;
          windowcount--;
        }
        
        /* If there are still unACKed packets in the window, restart the timer */
        if (windowcount > 0) {
          starttimer(A, RTT);
        }
      }
    } 
    else {
      if (TRACE > 0)
        printf("----A: duplicate ACK received, do nothing!\n");
    }
  }
  else {
    if (TRACE > 0)
      printf("----A: corrupted ACK is received, do nothing!\n");
  }
}

/* called when A's timer goes off */
void A_timerinterrupt(void)
{
  int i;

  if (TRACE > 0)
    printf("----A: time out,resend packets!\n");

  /* Resend the first unACKed packet */
  for (i = 0; i < windowcount; i++) {
    int idx = (windowfirst + i) % WINDOWSIZE;
    if (!acked[idx]) {
      if (TRACE > 0)
        printf("---A: resending packet %d\n", buffer[idx].seqnum);
      
      tolayer3(A, buffer[idx]);
      packets_resent++;
      break; /* Only resend one packet - this is key for SR */
    }
  }
  
  /* Always restart the timer */
  starttimer(A, RTT);
}       

/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  int i;
  
  /* initialise A's window, buffer and sequence number */
  A_nextseqnum = 0;  /* A starts with seq num 0, do not change this */
  windowfirst = 0;
  windowlast = -1;   /* windowlast is where the last packet sent is stored.  
                     new packets are placed in winlast + 1 
                     so initially this is set to -1
                   */
  windowcount = 0;
  
  /* Initialize the acked array */
  for (i = 0; i < WINDOWSIZE; i++) {
    acked[i] = false;
  }
}



/********* Receiver (B) variables and procedures ************/

static int expectedseqnum;             /* the sequence number expected next by the receiver */
static int B_nextseqnum;               /* the sequence number for the next packets sent by B */
static bool received[SEQSPACE];        /* tracking which packets have been received */
static struct pkt packet_buffer[SEQSPACE]; /* buffer for out-of-order packets */
static int recv_base;                  /* base of the receive window */

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  struct pkt sendpkt;
  int i;
  int seqnum;

  /* if not corrupted */
  if (!IsCorrupted(packet)) {
    seqnum = packet.seqnum;
    
    /* Check if packet is within receive window */
    if ((seqnum >= recv_base) && (seqnum < recv_base + WINDOWSIZE)) {
      if (TRACE > 0)
        printf("----B: packet %d is correctly received, send ACK!\n", seqnum);
      
      /* Buffer the packet */
      packet_buffer[seqnum] = packet;
      received[seqnum] = true;
      
      /* If this is the expected packet, deliver it and any buffered in-order packets */
      if (seqnum == expectedseqnum) {
        /* Deliver all consecutive packets that have been received */
        while ((expectedseqnum < recv_base + WINDOWSIZE) && received[expectedseqnum]) {
          tolayer5(B, packet_buffer[expectedseqnum].payload);
          packets_received++;
          
          /* Mark as not received anymore */
          received[expectedseqnum] = false;
          
          /* Move expected sequence number */
          expectedseqnum = (expectedseqnum + 1) % SEQSPACE;
        }
        
        /* Move receive window base */
        recv_base = expectedseqnum;
      }
    }
    /* Check if it's a retransmission of an already delivered packet */
    else if ((seqnum < recv_base) && (seqnum >= ((recv_base + SEQSPACE - WINDOWSIZE) % SEQSPACE))) {
      if (TRACE > 0)
        printf("----B: packet %d is correctly received, send ACK!\n", seqnum);
      /* It's a duplicate of an already ACKed packet, just send ACK again */
    }
    else {
      /* Packet outside our window, don't buffer it */
      if (TRACE > 0)
        printf("----B: packet %d is outside receive window\n", seqnum);
      return;
    }
    
    /* Always send ACK for correctly received packets */
    sendpkt.acknum = seqnum;
  }
  else {
    /* Packet is corrupted, don't send ACK */
    if (TRACE > 0) 
      printf("----B: packet corrupted or not expected sequence number, do nothing!\n");
    return;
  }

  /* create ACK packet */
  sendpkt.seqnum = B_nextseqnum;
  B_nextseqnum = (B_nextseqnum + 1) % 2;
    
  /* we don't have any data to send. fill payload with 0's */
  for (i = 0; i < 20; i++) 
    sendpkt.payload[i] = '0';  

  /* compute checksum */
  sendpkt.checksum = ComputeChecksum(sendpkt); 

  /* send out ACK packet */
  tolayer3(B, sendpkt);
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  int i;
  
  expectedseqnum = 0;
  B_nextseqnum = 1;
  recv_base = 0;
  
  /* Initialize receiver buffer */
  for (i = 0; i < SEQSPACE; i++) {
    received[i] = false;
  }
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
