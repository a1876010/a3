#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "sr.h"

/* ******************************************************************
   Selective Repeat protocol. Adapted from GBN implementation.

   Network properties:
   - one way network delay averages five time units (longer if there
   are other messages in the channel for SR), but can be larger
   - packets can be corrupted (either the header or the data portion)
   or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
   (although some can be lost).
**********************************************************************/

#define RTT  16.0       /* round trip time. MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 3    /* the window size for SR should be at most SEQSPACE/2 */
#define SEQSPACE 7      /* the sequence space size */
#define NOTINUSE (-1)   /* used to fill header fields that are not being used */

/* Data structure for timer management */
struct timer_info {
    int active;         /* Whether this timer is currently active */
    int seqnum;         /* Sequence number associated with this timer */
};

/* Generic procedure to compute the checksum of a packet. Used by both sender and receiver */
int ComputeChecksum(struct pkt packet)
{
    int checksum = 0;
    int i;

    checksum = packet.seqnum;
    checksum += packet.acknum;
    for (i = 0; i < 20; i++)
        checksum += (int)(packet.payload[i]);

    return checksum;
}

bool IsCorrupted(struct pkt packet)
{
    if (packet.checksum == ComputeChecksum(packet))
        return false;
    else
        return true;
}

/********* Sender (A) variables and functions ************/

static struct pkt buffer[SEQSPACE];    /* Array for storing packets waiting for ACK */
static int acked[SEQSPACE];            /* Array to track which packets have been ACKed */
static int windowfirst;                /* Base of the window */
static int windowlast;                 /* Last sequence number in the window */
static int windowcount;                /* Number of packets currently in the window */
static int A_nextseqnum;               /* Next sequence number to be used by the sender */
static struct timer_info timer_array[SEQSPACE]; /* For tracking multiple timers */

/* Check if sequence number n is within the current window */
bool IsInWindow(int n, int first, int last)
{
    if (first <= last)
        return (n >= first && n <= last);
    else
        return (n >= first || n <= last);
}

/* Start a timer for a specific sequence number */
void StartTimerFor(int seqnum)
{
    int i;
    
    /* Find an available timer slot */
    for (i = 0; i < SEQSPACE; i++) {
        if (!timer_array[i].active) {
            timer_array[i].active = 1;
            timer_array[i].seqnum = seqnum;
            starttimer(A, RTT);
            if (TRACE > 1)
                printf("----A: Starting timer for packet %d\n", seqnum);
            return;
        }
    }
    
    /* This should not happen if we manage timers correctly */
    printf("ERROR: No timer slots available\n");
}

/* Stop and clear a timer for a specific sequence number */
void StopTimerFor(int seqnum)
{
    int i;
    
    for (i = 0; i < SEQSPACE; i++) {
        if (timer_array[i].active && timer_array[i].seqnum == seqnum) {
            timer_array[i].active = 0;
            stoptimer(A);
            if (TRACE > 1)
                printf("----A: Stopping timer for packet %d\n", seqnum);
            return;
        }
    }
    
    /* This can happen if ACK arrives after timeout */
    if (TRACE > 1)
        printf("----A: No timer found for packet %d\n", seqnum);
}

/* Get the sequence number of the next packet to time out */
int GetNextTimeout()
{
    int i;
    
    for (i = 0; i < SEQSPACE; i++) {
        if (timer_array[i].active) {
            return timer_array[i].seqnum;
        }
    }
    
    return -1; /* No active timers */
}

/* Called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
    struct pkt sendpkt;
    int i;

    /* If window is not full */
    if (windowcount < WINDOWSIZE) {
        if (TRACE > 1)
            printf("----A: New message arrives, send window is not full, send new message to layer3!\n");

        /* Create packet */
        sendpkt.seqnum = A_nextseqnum;
        sendpkt.acknum = NOTINUSE;
        for (i = 0; i < 20; i++)
            sendpkt.payload[i] = message.data[i];
        sendpkt.checksum = ComputeChecksum(sendpkt);

        /* Store packet in buffer */
        buffer[A_nextseqnum] = sendpkt;
        acked[A_nextseqnum] = 0;  /* Mark as not yet acknowledged */
        windowcount++;

        /* Update window boundaries */
        windowlast = A_nextseqnum;
        
        /* Send out packet */
        if (TRACE > 0)
            printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
        tolayer3(A, sendpkt);

        /* Start timer for this packet */
        StartTimerFor(A_nextseqnum);

        /* Update next sequence number */
        A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;
    }
    /* If window is full */
    else {
        if (TRACE > 0)
            printf("----A: New message arrives, send window is full\n");
        window_full++;
    }
}

/* Called from layer 3, when a packet arrives for layer 4 */
void A_input(struct pkt packet)
{
    /* If ACK is not corrupted */
    if (!IsCorrupted(packet)) {
        if (TRACE > 0)
            printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
        total_ACKs_received++;

        /* Check if ACK is for a packet in the current window */
        if (IsInWindow(packet.acknum, windowfirst, windowlast)) {
            /* Check if this is a new ACK */
            if (!acked[packet.acknum]) {
                if (TRACE > 0)
                    printf("----A: ACK %d is not a duplicate\n", packet.acknum);
                new_ACKs++;

                /* Mark packet as acknowledged */
                acked[packet.acknum] = 1;
                
                /* Stop timer for this ACKed packet */
                StopTimerFor(packet.acknum);
                
                /* If base packet is ACKed, slide window */
                if (packet.acknum == windowfirst) {
                    /* Slide window until next unACKed packet or window is empty */
                    while (windowcount > 0 && acked[windowfirst]) {
                        windowfirst = (windowfirst + 1) % SEQSPACE;
                        windowcount--;
                    }
                    
                    /* If window is not empty, make sure a timer is running */
                    if (windowcount > 0 && GetNextTimeout() == -1) {
                        /* Find the first unACKed packet and start timer */
                        int i = windowfirst;
                        do {
                            if (!acked[i]) {
                                StartTimerFor(i);
                                break;
                            }
                            i = (i + 1) % SEQSPACE;
                        } while (i != (windowfirst + windowcount) % SEQSPACE);
                    }
                }
            }
            else {
                if (TRACE > 0)
                    printf("----A: duplicate ACK %d, ignore\n", packet.acknum);
            }
        }
        else {
            if (TRACE > 0)
                printf("----A: ACK %d outside window, ignore\n", packet.acknum);
        }
    }
    else {
        if (TRACE > 0)
            printf("----A: corrupted ACK received, ignore\n");
    }
}

/* Called when A's timer goes off */
void A_timerinterrupt(void)
{
    int timedout_seq;
    int i; /* Moved declaration to the beginning of the function */
    
    timedout_seq = GetNextTimeout();
    
    if (timedout_seq != -1) {
        if (TRACE > 0)
            printf("----A: timeout for packet %d, resend\n", timedout_seq);
        
        /* Resend the specific packet that timed out */
        tolayer3(A, buffer[timedout_seq]);
        packets_resent++;
        
        /* Mark this timer as inactive and start a new one */
        for (i = 0; i < SEQSPACE; i++) {
            if (timer_array[i].active && timer_array[i].seqnum == timedout_seq) {
                timer_array[i].active = 0;
                break;
            }
        }
        
        /* Start a new timer for this packet */
        StartTimerFor(timedout_seq);
    }
    else {
        if (TRACE > 0)
            printf("----A: timer interrupted but no active timers found\n");
    }
}

/* Called once before any other entity A routines */
void A_init(void)
{
    int i;
    
    /* Initialize A's window, buffer and sequence number */
    A_nextseqnum = 0;  /* A starts with seq num 0, do not change this */
    windowfirst = 0;
    windowlast = -1;   /* No packets in window initially */
    windowcount = 0;
    
    /* Initialize ACK tracking array */
    for (i = 0; i < SEQSPACE; i++) {
        acked[i] = 0;
    }
    
    /* Initialize timer array */
    for (i = 0; i < SEQSPACE; i++) {
        timer_array[i].active = 0;
    }
}

/********* Receiver (B) variables and procedures ************/

static int expectedseqnum;        /* Next in-order sequence number expected */
static int B_nextseqnum;          /* Next sequence number for B's packets */
static int received[SEQSPACE];    /* Track which packets have been received */
static struct pkt rcvbuffer[SEQSPACE]; /* Buffer for out-of-order packets */
static int rcv_base;              /* Base of receiver window */

/* Called from layer 3, when a packet arrives for layer 4 at B */
void B_input(struct pkt packet)
{
    struct pkt sendpkt;
    int i;

    /* If packet is not corrupted */
    if (!IsCorrupted(packet)) {
        if (TRACE > 0)
            printf("----B: packet %d is correctly received\n", packet.seqnum);
        
        /* Check if packet is within the receiver window */
        if (IsInWindow(packet.seqnum, rcv_base, (rcv_base + WINDOWSIZE - 1) % SEQSPACE)) {
            /* Store packet and mark as received */
            if (!received[packet.seqnum]) {
                rcvbuffer[packet.seqnum] = packet;
                received[packet.seqnum] = 1;
                packets_received++;
                
                /* If packet is the expected one, deliver it and any consecutive buffered packets */
                if (packet.seqnum == rcv_base) {
                    do {
                        /* Deliver to application layer */
                        tolayer5(B, rcvbuffer[rcv_base].payload);
                        
                        /* Mark as not received (for wrap-around) */
                        received[rcv_base] = 0;
                        
                        /* Advance rcv_base */
                        rcv_base = (rcv_base + 1) % SEQSPACE;
                    } while (received[rcv_base]);
                }
            }
            
            /* Send ACK for the received packet */
            sendpkt.acknum = packet.seqnum;
        }
        else {
            /* Packet outside the receiver window */
            if (TRACE > 0)
                printf("----B: packet %d outside receive window\n", packet.seqnum);
            
            /* If it's before rcv_base, it's a duplicate, ack it again */
            if (((rcv_base > packet.seqnum) && (rcv_base - packet.seqnum <= SEQSPACE/2)) ||
                ((rcv_base < packet.seqnum) && (packet.seqnum - rcv_base > SEQSPACE/2))) {
                sendpkt.acknum = packet.seqnum;
            }
            else {
                /* Otherwise ignore packet */
                return;
            }
        }
    }
    else {
        /* Corrupted packet */
        if (TRACE > 0)
            printf("----B: packet corrupted\n");
        return;  /* For SR, don't ACK corrupted packets */
    }

    /* Create ACK packet */
    sendpkt.seqnum = B_nextseqnum;
    B_nextseqnum = (B_nextseqnum + 1) % SEQSPACE;
        
    /* We don't have any data to send. Fill payload with 0's */
    for (i = 0; i < 20; i++)
        sendpkt.payload[i] = '0';
        
    /* Compute checksum */
    sendpkt.checksum = ComputeChecksum(sendpkt);
        
    /* Send ACK packet */
    tolayer3(B, sendpkt);
}

/* Called once before any other entity B routines */
void B_init(void)
{
    int i;
    
    expectedseqnum = 0;
    B_nextseqnum = 1;
    rcv_base = 0;
    
    /* Initialize received array */
    for (i = 0; i < SEQSPACE; i++) {
        received[i] = 0;
    }
}

/******************************************************************************
 * The following functions need be completed only for bi-directional messages *
 *****************************************************************************/

/* Note that with simplex transfer from a-to-B, there is no B_output() */
void B_output(struct msg message)
{
}

/* Called when B's timer goes off */
void B_timerinterrupt(void)
{
}