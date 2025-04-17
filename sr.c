
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
static int windowlast;                 /* array index of the last packet awaiting ACK */
static int windowcount;                /* the number of packets currently awaiting an ACK */
static int A_nextseqnum;               /* the next sequence number to be used by the sender */
static int send_timer[WINDOWSIZE];     /* individual timer for each packet in window */
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
    windowlast = (windowlast + 1) % WINDOWSIZE; 
    buffer[windowlast] = sendpkt;
    acked[windowlast] = false;  // Mark as not yet acknowledged
    windowcount++;

    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3(A, sendpkt);

    /* start timer for this specific packet */
    send_timer[windowlast] = A_nextseqnum;  // Store sequence number for timer identification
    starttimer(A, RTT);

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
  int i, pos = -1;
  
  /* if received ACK is not corrupted */ 
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
    total_ACKs_received++;

    /* check if the ACK is for a packet in our current window */
    for (i = 0; i < windowcount; i++) {
      int bufIndex = (windowfirst + i) % WINDOWSIZE;
      if (buffer[bufIndex].seqnum == packet.acknum) {
        pos = bufIndex;
        break;
      }
    }

    /* if ACK is for a packet in our window */
    if (pos != -1) {
      /* If not already ACKed, mark it as ACKed */
      if (!acked[pos]) {
        if (TRACE > 0)
          printf("----A: ACK %d is not a duplicate\n", packet.acknum);
        new_ACKs++;
        acked[pos] = true;
        
        /* If it's the first packet in the window, try to slide window */
        if (pos == windowfirst) {
          /* Slide window past all consecutively ACKed packets */
          while (windowcount > 0 && acked[windowfirst]) {
            windowfirst = (windowfirst + 1) % WINDOWSIZE;
            windowcount--;
          }
        }
      }
      else {
        if (TRACE > 0)
          printf("----A: duplicate ACK received, do nothing!\n");
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
  
  if (TRACE > 0)
    printf("----A: time out, resend packet!\n");
  
  /* Find the packet whose timer expired and resend only that packet */
  /* In a real implementation, we would identify exactly which timer expired,
     but for this simulation we will resend all unacknowledged packets */
  for (i = 0; i < windowcount; i++) {
    int bufIndex = (windowfirst + i) % WINDOWSIZE;
    if (!acked[bufIndex]) {
      if (TRACE > 0)
        printf("---A: resending packet %d\n", buffer[bufIndex].seqnum);
      
      tolayer3(A, buffer[bufIndex]);
      packets_resent++;
      starttimer(A, RTT);  // Restart timer for this packet
    }
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
  
  /* Initialize status tracking arrays */
  int i;
  for (i = 0; i < WINDOWSIZE; i++) {
    acked[i] = false;
    send_timer[i] = NOTINUSE;
  }
}

/********* Receiver (B) variables and procedures ************/

static int expectedseqnum;           /* the sequence number expected next by the receiver */
static int B_nextseqnum;             /* the sequence number for the next packets sent by B */
static struct pkt recv_buffer[WINDOWSIZE]; /* buffer for out-of-order packets */
static bool received[WINDOWSIZE];    /* tracking which packets have been received */
static int recv_base;                /* base of receiver window */

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  struct pkt sendpkt;
  int i;
  int seqnum = packet.seqnum;
  
  /* if not corrupted */
  if (!IsCorrupted(packet)) {
    /* Compute the position in the buffer where this packet belongs */
    int offset = (seqnum - recv_base + SEQSPACE) % SEQSPACE;
    
    /* Check if packet is within the window */
    if (offset < WINDOWSIZE) {
      if (TRACE > 0)
        printf("----B: packet %d is correctly received, send ACK!\n", seqnum);
      
      /* Store the packet and mark it as received */
      int bufIndex = (recv_base + offset) % WINDOWSIZE;
      recv_buffer[bufIndex] = packet;
      received[bufIndex] = true;
      
      /* If it's the expected packet (first in window), deliver it and advance window */
      if (seqnum == expectedseqnum) {
        /* Deliver this packet and any consecutive buffered packets */
        while (received[recv_base % WINDOWSIZE]) {
          /* Deliver to receiving application */
          tolayer5(B, recv_buffer[recv_base % WINDOWSIZE].payload);
          packets_received++;
          
          /* Mark as not received to reuse the buffer slot */
          received[recv_base % WINDOWSIZE] = false;
          
          /* Move window forward */
          recv_base = (recv_base + 1) % SEQSPACE;
          expectedseqnum = (expectedseqnum + 1) % SEQSPACE;
        }
      }
    }
    
    /* Send ACK for the received packet */
    sendpkt.acknum = seqnum;
  }
  else {
    /* packet is corrupted, don't send ACK */
    if (TRACE > 0) 
      printf("----B: packet corrupted, no ACK sent!\n");
    return;
  }

  /* create packet */
  sendpkt.seqnum = B_nextseqnum;
  B_nextseqnum = (B_nextseqnum + 1) % 2;
    
  /* we don't have any data to send. fill payload with 0's */
  for (i = 0; i < 20; i++) 
    sendpkt.payload[i] = '0';  

  /* compute checksum */
  sendpkt.checksum = ComputeChecksum(sendpkt); 

  /* send out packet */
  tolayer3(B, sendpkt);
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  expectedseqnum = 0;
  B_nextseqnum = 1;
  recv_base = 0;
  
  /* Initialize receiver buffer */
  int i;
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