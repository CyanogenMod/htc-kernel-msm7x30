/*
 * This is part of the Sequans SQN1130 driver.
 * Copyright 2008 SEQUANS Communications
 * Written by Andy Shevchenko <andy@smile.org.ua>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#ifndef _SQN_MAIN_H
#define _SQN_MAIN_H


extern char	*firmware_name;
extern int	load_firmware;


struct sqn_private *sqn_add_card(void *card, struct device *realdev);
int sqn_remove_card(struct sqn_private *priv);

int sqn_start_card(struct sqn_private *priv);
int sqn_stop_card(struct sqn_private *priv);

int sqn_start_tx_thread(struct sqn_private *priv);
int sqn_stop_tx_thread(struct sqn_private *priv);

int sqn_rx_process(struct net_device *dev, struct sk_buff *skb);

#endif /* _SQN_MAIN_H */
