/*
 * This is part of the Sequans SQN1130 driver.
 * Copyright 2009 SEQUANS Communications
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#ifndef _SQN_THP2_H
#define _SQN_THP2_H


struct sqn_thp_header {
	/** Transport protocol version - must be 1 for now. */
	u8  transport_version;

	/* Flags Field is used to relay control information between THP peers
	 * | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
	 * | 0 |DAK| 0 |ACK|EOF|MOF|BOF|NOF|
	 *
	 *	NOF: No fragmentation
	 *	BOF: Begining of fragmentation
	 *	MOF: Middle of fragmentation
	 *	EOF: End of fragmentation
	 *	ACK: The sender acknowledge the reception of the "AckNumber"
	 *	sequence number.  DAK: The sender ask the receiver to
	 *	acknowledge the seqence number "seqNumber".*/
	u8 flags;

	/** Length of the transported payload message, (without header). */
	u16 length;

	/** Sequence Number
	 *  Which shall be incremented for each fragment (or no fragmented
	 *  command). */
	u16 seq_number;

	/** Acknowledgment Number
	 *  When ACK=DAK=NAK=0, the ackNumber is equal to the last sequence
	 *  received  number. */
	u16 ack_number;

	/** Length of the payload message before fragmentation.
	 *  Note: In case of no fragmentation totalLength is equal to length.*/
	u32 total_length;
};


int init_thp(struct net_device* dev);
int thp_wimax_uart_switch(int on);
void cleanup_thp(void);

#endif  /* _SQN_THP2_H */

