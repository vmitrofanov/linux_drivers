#ifndef _BB_DRV_H
#define _BB_DRV_H

#include <linux/netdevice.h>

/**
 * GEMAC NAPI rx polling method
 */
int poll_rx(struct napi_struct *napi, int weight);

/**
 * GEMAC NAPI tx polling method
 */
int poll_tx(struct napi_struct *napi, int weight);

#endif
