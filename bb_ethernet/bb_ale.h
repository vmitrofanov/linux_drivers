#ifndef _BB_ALE_H
#define _BB_ALE_H

#include "bb_eth.h"
#include <linux/types.h>

/* ALE Registers */
#define ALE_CONTROL             0x08
#define ALE_UNKNOWNVLAN         0x18
#define ALE_TABLE_CONTROL       0x20
#define ALE_TABLE               0x34
#define ALE_PORTCTL             0x40

#define ALE_TABLE_WRITE         BIT(31)

#define ALE_TYPE_FREE                   0
#define ALE_TYPE_ADDR                   1
#define ALE_TYPE_VLAN                   2
#define ALE_TYPE_VLAN_ADDR              3

#define ALE_UCAST_PERSISTANT            0
#define ALE_UCAST_UNTOUCHED             1
#define ALE_UCAST_OUI                   2
#define ALE_UCAST_TOUCHED               3

#define ALE_MCAST_FWD                   0
#define ALE_MCAST_BLOCK_LEARN_FWD       1
#define ALE_MCAST_FWD_LEARN             2
#define ALE_MCAST_FWD_2                 3

/* Address lookup engine */
#define ALE_ENTRY_BITS		68
#define ALE_ENTRY_WORDS	DIV_ROUND_UP(ALE_ENTRY_BITS, 32)
#define ALE_SECURE			BIT(0)
#define ALE_BLOCKED			BIT(1)
#define ALE_SUPER			BIT(2)
#define ALE_VLAN			BIT(3)
#define BITMASK(bits)		(BIT(bits) - 1)
#define ALE_TABLE_CONTROL	0x20
#define ALE_TABLE		0x34


#define ALE_PCTRL0		0x40

void bb_ale_init(struct gemac_private *gemac);
int bb_ale_add_mcast(struct gemac_private *gemac, const u8 *addr, int port_mask);

#endif
