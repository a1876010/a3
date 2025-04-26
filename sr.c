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
static int windowfirst;                /* array index of the first packet awaiting ACK */
static int windowcount;                /* the number of packets currently awaiting an ACK */
static int A_nextseqnum;               /* the next sequence number to be used by the sender */
static bool acked[WINDOWSIZE];         /* tracking which packets have been ACKed */
static int timers[WINDOWSIZE];         /* individual timers for each packet slot */

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;
  int next_slot;

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

    /* Find the next available slot in the window */
    next_slot = (windowfirst + windowcount) % WINDOWSIZE;
    
    /* put packet in window buffer */
    buffer[next_slot] = sendpkt;
    acked[next_slot] = false;  /* Mark as not yet acknowledged */
    windowcount++;

    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3(A, sendpkt);

    /* start timer for this specific packet */
    timers[next_slot] = A_nextseqnum;
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

  /* if received ACK is not corrupted */ 
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
    total_ACKs_received++;

    seqnum = packet.acknum;
    
    /* Determine if this is a new ACK */
    for (i = 0; i < windowcount; i++) {
      int idx = (windowfirst + i) % WINDOWSIZE;
      if (buffer[idx].seqnum == seqnum && !acked[idx]) {
        if (TRACE > 0)
          printf("----A: ACK %d is not a duplicate\n", seqnum);
        
        /* Mark packet as acknowledged */
        acked[idx] = true;
        new_ACKs++;
        
        /* If it's the base, try to slide window */
        if (idx == windowfirst) {
          /* Slide window past all ACKed packets at the beginning */
          while (windowcount > 0 && acked[windowfirst]) {
            windowfirst = (windowfirst + 1) % WINDOWSIZE;
            windowcount--;
          }
        }
        break;
      }
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
  bool found_unacked = false;

  if (TRACE > 0)
    printf("----A: time out, resend packets!\n");

  /* Find the unacknowledged packet whose timer expired */
  for (i = 0; i < windowcount; i++) {
    int idx = (windowfirst + i) % WINDOWSIZE;
    if (!acked[idx]) {
      if (TRACE > 0)
        printf("---A: resending packet %d\n", buffer[idx].seqnum);
      
      tolayer3(A, buffer[idx]);
      packets_resent++;
      
      /* Start a new timer for this packet */
      starttimer(A, RTT);
      found_unacked = true;
      break;  /* Only resend one packet on timeout in SR */
    }
  }
  
  /* If no unacknowledged packets found, don't restart the timer */
  if (!found_unacked && windowcount > 0) {
    starttimer(A, RTT);  /* Keep the timer running if we still have packets */
  }
}       

/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  int i;
  
  /* initialise A's window, buffer and sequence number */
  A_nextseqnum = 0;  /* A starts with seq num 0, do not change this */
  windowfirst = 0;
  windowcount = 0;
  
  /* Initialize the acked array and timers */
  for (i = 0; i < WINDOWSIZE; i++) {
    acked[i] = false;
    timers[i] = NOTINUSE;
  }
}



/********* Receiver (B) variables and procedures ************/

static int expectedseqnum;            /* the sequence number expected next by the receiver */
static int B_nextseqnum;              /* the sequence number for the next packets sent by B */
static int recv_base;                 /* base of receiver window */
static struct pkt recv_buffer[WINDOWSIZE]; /* buffer for out-of-order packets */
static bool received[WINDOWSIZE];     /* tracking which packets have been received */

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  struct pkt sendpkt;
  int i;
  int seqnum;
  int offset;

  /* if not corrupted */
  if (!IsCorrupted(packet)) {
    seqnum = packet.seqnum;
    
    /* Calculate where in our receive window this packet belongs */
    offset = (seqnum - recv_base + SEQSPACE) % SEQSPACE;
    
    /* Check if packet is within the receive window */
    if (offset < WINDOWSIZE) {
      if (TRACE > 0)
        printf("----B: packet %d is correctly received, send ACK!\n", seqnum);
      
      /* Store packet in the receive buffer */
      recv_buffer[offset] = packet;
      received[offset] = true;
      
      /* If this was the base packet, deliver it and any consecutive packets */
      if (offset == 0) {
        /* Deliver all consecutively received packets */
        while (received[0]) {
          /* Deliver to upper layer */
          tolayer5(B, recv_buffer[0].payload);
          packets_received++;
          
          /* Shift window forward */
          for (i = 0; i < WINDOWSIZE - 1; i++) {
            received[i] = received[i + 1];
            recv_buffer[i] = recv_buffer[i + 1];
          }
          received[WINDOWSIZE - 1] = false;
          
          /* Advance receive window base */
          recv_base = (recv_base + 1) % SEQSPACE;
          expectedseqnum = recv_base;  /* Next expected is always the base */
        }
      }
      
      /* Send ACK for this specific packet */
      sendpkt.acknum = seqnum;
    }
    else {
      /* Packet is outside our window - might be a duplicate */
      if (TRACE > 0)
        printf("----B: duplicate packet %d received, send ACK!\n", seqnum);
      
      /* ACK it anyway to handle potential lost ACKs */
      sendpkt.acknum = seqnum;
    }
    
    /* Create ACK packet */
    sendpkt.seqnum = B_nextseqnum;
    B_nextseqnum = (B_nextseqnum + 1) % 2;
      
    /* We don't have any data to send. Fill payload with 0's */
    for (i = 0; i < 20; i++) 
      sendpkt.payload[i] = '0';  

    /* Compute checksum */
    sendpkt.checksum = ComputeChecksum(sendpkt); 

    /* Send ACK */
    tolayer3(B, sendpkt);
  }
  else {
    /* Packet is corrupted - in SR, we don't send any ACK */
    if (TRACE > 0) 
      printf("----B: corrupted packet received, do nothing!\n");
  }
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
  for (i = 0; i < WINDOWSIZE; i++) {
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