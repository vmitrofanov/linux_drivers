#ifndef _AF_XDP_H_
#define _AF_XDP_H_

#include <net/xdp_sock.h>
#include <net/xdp.h>
#include <linux/bpf_trace.h>
#include "stmmac.h"

//#define DEBUG_AF_XDP
#ifdef DEBUG_AF_XDP
#define DBG(...)	pr_info(__VA_ARGS__)
#else
#define DBG(...)
#endif

#define AF_XDP_PASS			0
#define AF_XDP_CONSUMED			BIT(0)
#define AF_XDP_TX			BIT(1)
#define AF_XDP_REDIR			BIT(2)

#define AF_XDP_TYPE_RESET		0
#define AF_XDP_TYPE_SKB			0
#define AF_XDP_TYPE_UMEM		1
#define AF_XDP_TYPE_FRAME		2

/* Minimal amount of frames that can be used by network stack instead UMEM.
 * Than higher value than more UMEM throughput but less network throughput.
 * in case of heavy traffic */
#define AF_XDP_MIN_FRAMES		40

/* Refill timer restart time */
#define AF_XDP_REFILL_MS		500
#define AF_XDP_INITIAL_REFILL_MS	500

#define DMA_ATTR \
	(DMA_ATTR_SKIP_CPU_SYNC | DMA_ATTR_WEAK_ORDERING)

/* NDO prototypes */
int af_xdp_bpf(struct net_device *dev, struct netdev_bpf *bpf);
int af_xdp_xmit(struct net_device *dev, int n, struct xdp_frame **xdp, u32 flags);
int af_xdp_wakeup(struct net_device *dev, u32 queue_id, u32 flags);

/* Inner usage */
inline u32 stmmac_tx_avail(struct stmmac_priv *priv, u32 queue);
int af_xdp_poll_rx(struct stmmac_priv *priv, int limit, u32 queue);
int af_xdp_xmit_umem(struct stmmac_priv *priv, int napi_budget);
int af_xdp_initialize_dma(struct stmmac_priv *priv);

#endif
