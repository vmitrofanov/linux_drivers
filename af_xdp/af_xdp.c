#include "af_xdp.h"

#include <linux/kernel.h>
#include <linux/skbuff.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/prefetch.h>
#include <net/xdp_sock.h>
#include <net/xdp.h>

/**
 * use_umem() - do we use UMEM for a particular queue
 * @priv: common structure
 * @queue: queue index
 *
 * RETURN: 0 - don't use	1 - use
 */
static inline int use_umem(struct stmmac_priv *priv, u32 queue)
{
	if ((priv->umems_storage == NULL) ||
	    (priv->umems_storage[queue] == NULL))
		return 0;
	else return 1;
}

/**
 * af_xdp_alloc_umem_storage() - allocate memory for a UMEM storage
 * @priv: common structure
 *
 * RETURN: 0 - ok	-ENOMEM - fail
 */
static int af_xdp_alloc_umem_storage(struct stmmac_priv *priv)
{
	/* If we have already allocated it */
	if (priv->umems_storage)
		return 0;

	DBG("%s-->\n", __FUNCTION__);

	/* Now it supports only 1 UMEM for queue 0 */
	priv->umems_storage = kcalloc(priv->umem_number,
				      sizeof(*priv->umems_storage),
				      GFP_KERNEL);
	if (!priv->umems_storage)
		return -ENOMEM;

	priv->umem_users = 0;
	priv->umem_number = 1;

	DBG("%s<--\n", __FUNCTION__);

	return 0;
}

/**
 * af_xdp_remove_umem() - remove UMEM from a queue
 * @priv: common structure
 * @queue: queue index
 *
 * If there are no queues delete storage structure
 */
static void af_xdp_remove_umem(struct stmmac_priv *priv, u16 queue)
{
	DBG("%s-->\n", __FUNCTION__);

	priv->umems_storage[queue] = NULL;
	priv->umem_users--;

	if (priv->umem_number == 0) {
		kfree(priv->umems_storage);
		priv->umems_storage = NULL;
		priv->umem_number = 0;
	}

	DBG("%s<--\n", __FUNCTION__);
}

/**
 * af_xdp_insert_umem() - insert a new one UMEM to common storage
 * @priv: common structure
 * @umem: a new UMEM to add
 * @queu: queue index
 *
 * RETURN: 0 - ok	-ENOMEM - fail
 */
static int af_xdp_insert_umem(struct stmmac_priv *priv, struct xdp_umem *umem,
			       u16 queue)
{
	int err;

	DBG("%s-->\n", __FUNCTION__);

	err = af_xdp_alloc_umem_storage(priv);
	if (err)
		return err;

	priv->umems_storage[queue] = umem;
	priv->umem_users++;

	DBG("%s<--\n", __FUNCTION__);

	return 0;
}

/**
 * af_xdp_umem_map_resources() - map UMEM descriptors memory
 * @priv: common structure
 * @umem: associated UMEM
 *
 * RETURN: 0 - ok	-ENOMEM - fail
 */
static int af_xdp_umem_map_resources(struct stmmac_priv *priv,
				       struct xdp_umem *umem)
{
	struct device *dev = priv->device;
	unsigned int i, j;
	dma_addr_t dma;

	DBG("%s-->\n", __FUNCTION__);

	for (i = 0; i < umem->npgs; i++) {
		dma = dma_map_page_attrs(dev, umem->pgs[i], 0, PAGE_SIZE,
					 DMA_BIDIRECTIONAL, DMA_ATTR);
		if (dma_mapping_error(dev, dma))
			goto out_unmap;

		umem->pages[i].dma = dma;
	}

	DBG("%s<--\n", __FUNCTION__);

	return 0;

out_unmap:
	for (j = 0; j < i; j++) {
		dma_unmap_page_attrs(dev, umem->pages[i].dma, PAGE_SIZE,
				     DMA_BIDIRECTIONAL, DMA_ATTR);
		umem->pages[i].dma = 0;
	}

	return -ENOMEM;
}


/**
 * af_xdp_umem_unmap_resources() - unmap UMEM descriptors memory
 * @priv: common strucutre
 * @umem: associated UMEM
 */
static void af_xdp_umem_unmap_resources(struct stmmac_priv *priv,
				      struct xdp_umem *umem)
{
	struct device *dev = priv->device;
	unsigned int i;

	DBG("%s-->\n", __FUNCTION__);

	for (i = 0; i < umem->npgs; i++) {
		dma_unmap_page_attrs(dev, umem->pages[i].dma, PAGE_SIZE,
				     DMA_BIDIRECTIONAL, DMA_ATTR);
		umem->pages[i].dma = 0;
	}

	DBG("%s<--\n", __FUNCTION__);
}


/**
 * af_xdp_reuse_frame() - use frame descriptor memory to refill dirty frame
 * @alloc: copy allocator structure
 * @handle: UMEM handle
 */
static void af_xdp_reuse_frame(struct zero_copy_allocator *allocator,
			      unsigned long frame_handle)
{
	struct stmmac_priv *priv = container_of(allocator, struct stmmac_priv,
						zca);
	struct stmmac_rx_queue *rx_q = &priv->rx_queue[0];
	unsigned int entry = rx_q->dirty_rx;
	struct stmmac_xsk_desc_map *buf = &rx_q->desc_map[entry];
	struct xdp_umem *umem = priv->umems_storage[0];
	struct dma_desc *p;
	u64 head_room = umem->headroom + XDP_PACKET_HEADROOM;
	u32 chunk_len = priv->umems_storage[0]->chunk_size_nohr;
	size_t frame_len;

	DBG("%s-->\n", __FUNCTION__);

	/* Clear mask */
	frame_handle &= umem->chunk_mask;

	if (priv->extend_desc)
		p = (struct dma_desc *)(rx_q->dma_erx + entry);
	else
		p = rx_q->dma_rx + entry;

	/* Update frame settings */
	buf->dma_addr += head_room;
	buf->cpu_addr += head_room;
	buf->dma_addr = xdp_umem_get_dma(umem, frame_handle);
	buf->cpu_addr = xdp_umem_get_data(umem, frame_handle);
	buf->handle = xsk_umem_adjust_offset(umem, (u64)frame_handle,
					     umem->headroom);

	frame_len = chunk_len - XDP_PACKET_HEADROOM;
	dma_sync_single_range_for_device(priv->device, buf->dma_addr,
					 buf->page_offset, frame_len,
					 DMA_BIDIRECTIONAL);
	if (dma_mapping_error(priv->device, buf->dma_addr)) {
		netdev_err(priv->dev, "Rx DMA map failed\n");
		return;
	}

	/* Charge up frame descriptor */
	stmmac_set_desc_addr(priv, p, buf->dma_addr);
	stmmac_refill_desc3(priv, rx_q, p);
	dma_wmb();
	stmmac_set_rx_owner(priv, p, priv->use_riwt);

	/* Advance dirty pointer */
	rx_q->dirty_rx = STMMAC_GET_ENTRY(entry, DMA_RX_SIZE);

	DBG("%s<--\n", __FUNCTION__);
}

/**
 * af_xdp_alloc_frames() - allocate resources for queue's frames
 * @priv: common structure
 * @queue: queue index
 *
 * Any XSK packet form UMEM can be converted to frame for use in network stack
 * or in case of retransmit so we have to allocate memory in advance.
 */
static int af_xdp_alloc_frames(struct stmmac_priv *priv, int queue)
{
	struct stmmac_tx_queue *tx_q = &priv->tx_queue[queue];
	size_t len = priv->umems_storage[queue]->chunk_size_nohr - XDP_PACKET_HEADROOM;
	int err_i;
	int i;

	DBG("%s-->\n", __FUNCTION__);

	for (i = 0; i < DMA_TX_SIZE; i++) {
		tx_q->tx_skbuff_dma[i].page = kcalloc(1, len, GFP_KERNEL);
		if (!tx_q->tx_skbuff_dma[i].page)
			goto err;
	}

	DBG("%s<--\n", __FUNCTION__);

	return 0;

err:
	err_i = i;
	for (i = 0; i < err_i; ++i)
		kfree(tx_q->tx_skbuff_dma[i].page);

	return -ENOMEM;
}

/**
 * af_xdp_free_frames() - free resources for queue's frames
 * @priv: common structure
 * @queue: queue index
 */
static void af_xdp_free_frames(struct stmmac_priv *priv, int queue)
{
	struct stmmac_tx_queue *tx_q = &priv->tx_queue[queue];
	int i;

	DBG("%s-->\n", __FUNCTION__);

	for (i = 0; i < DMA_TX_SIZE; i++)
		kfree(tx_q->tx_skbuff_dma[i].page);

	DBG("%s<--\n", __FUNCTION__);
}

/**
 * af_xdp_xmit_frame() - transmit a frame
 * @priv: common structure
 * @xdpf: transmitted frame
 *
 * RETURN: AF_XDP_TX - ok	AF_XDP_CONSUMED - failed
 */
static int af_xdp_xmit_frame(struct stmmac_priv *priv, struct xdp_frame *xdpf)
{
	struct stmmac_tx_queue *tx_q = priv->tx_queue + 0;
	struct dma_desc *desc;
	int entry = tx_q->cur_tx;
	dma_addr_t dma;
	u32 tx_avail = stmmac_tx_avail(priv, 0);

	if (!tx_avail)
		return AF_XDP_CONSUMED;

	if (priv->extend_desc)
		desc = (struct dma_desc *)(tx_q->dma_etx + entry);
	else
		desc = tx_q->dma_tx + entry;

	/* Charge up tx frame for transmission */
	memcpy(tx_q->tx_skbuff_dma[entry].page, xdpf->data, xdpf->len);
	dma = dma_map_single(priv->device, tx_q->tx_skbuff_dma[entry].page,
			     xdpf->len, DMA_TO_DEVICE);
	if (dma_mapping_error(priv->device, dma))
		return AF_XDP_CONSUMED;

	tx_q->cur_tx = STMMAC_GET_ENTRY(entry, DMA_TX_SIZE);
	tx_q->tx_skbuff_dma[entry].tx_source_type = AF_XDP_TYPE_FRAME;
	tx_q->tx_skbuff_dma[entry].buf = dma;
	tx_q->tx_skbuff_dma[entry].len = xdpf->len;
	tx_q->tx_skbuff_dma[entry].page_addr = xdpf->data;

	stmmac_set_desc_addr(priv, desc, dma);
	stmmac_prepare_tx_desc(priv, desc, 1, xdpf->len, 0, priv->mode, 1, 1,
			       xdpf->len);
	dma_wmb();
	stmmac_enable_dma_transmission(priv, priv->ioaddr);

	return AF_XDP_TX;
}

/**
 * af_xdp_xmit_umem() - transmit a packet from UMEM
 * @priv: common structure
 * @napi_budget: retransmitting budget
 *
 * RETURN: number of packets has been transmitted.
 */
int af_xdp_xmit_umem(struct stmmac_priv *priv, int napi_budget)
{
	struct stmmac_tx_queue *tx_q = priv->tx_queue;
	struct stmmac_tx_meta *meta;
	struct dma_desc *desc;
	struct xdp_desc xdp_desc;
	int cur = tx_q->cur_tx;
	u32 processed = 0;
	u32 avaliable = stmmac_tx_avail(priv, 0);
	dma_addr_t dma_addr;

	/* Save batch of descriptors for transmitting by the network stack */
	if (avaliable < AF_XDP_MIN_FRAMES)
		return 0;

	/* Prepare all available frames to transmit */
	while ((napi_budget-- > 0) && (avaliable-- > 0)) {
		/* Try to acquire available frame */
		if (!xsk_umem_consume_tx(priv->umems_storage[0], &xdp_desc))
			break;

		if (likely(priv->extend_desc))
			desc = (struct dma_desc *)(tx_q->dma_etx + cur);
		else
			desc = tx_q->dma_tx + cur;

		meta = tx_q->tx_skbuff_dma + cur;
		meta->tx_source_type = AF_XDP_TYPE_UMEM;

		dma_addr = xdp_umem_get_dma(priv->umems_storage[0],
					    xdp_desc.addr);
		dma_sync_single_for_device(priv->device, dma_addr, xdp_desc.len,
					   DMA_BIDIRECTIONAL);

		stmmac_set_desc_addr(priv, desc, dma_addr);
		stmmac_prepare_tx_desc(priv, desc, 1, xdp_desc.len, 0,
				       priv->mode, 1, 1, xdp_desc.len);

		cur = STMMAC_GET_ENTRY(cur, DMA_TX_SIZE);
		tx_q->cur_tx = cur;

		++processed;
	}

	/* Notify socket in user space about written data */
	if (processed)
		xsk_umem_consume_tx_done(priv->umems_storage[0]);

	/* Start transmission */
	stmmac_enable_dma_transmission(priv, priv->ioaddr);

	return processed;
}

/**
 * af_xdp_rx_refill() - refill used descriptors
 * @priv: common structure
 * @queue: queue index
 *
 * RETURN: number refilled descriptors
 */
static int af_xdp_rx_refill(struct stmmac_priv *priv, u32 queue)
{
	struct stmmac_rx_queue *rx_q = &priv->rx_queue[queue];
	struct stmmac_xsk_desc_map *buf;
	struct xdp_umem *umem = priv->umems_storage[queue];
	unsigned int entry = rx_q->dirty_rx;
	int dirty = stmmac_rx_dirty(priv, queue);
	int cleaned = 0;
	u64 hr = umem->headroom + XDP_PACKET_HEADROOM;
	u64 frame_handle;

	DBG("%s-->\n", __FUNCTION__);

	/* Sanity check */
	if (!use_umem(priv, queue))
		return -EPERM;

	if (atomic_read(&rx_q->lock))
		return -EBUSY;

	atomic_set(&rx_q->lock, 1);

	while (dirty-- > 0) {
		struct dma_desc *p;
		size_t buf_size;

		/* AF_XDP socket's meta data */
		buf = &rx_q->desc_map[entry];

		if (priv->extend_desc)
			p = (struct dma_desc *)(rx_q->dma_erx + entry);
		else
			p = rx_q->dma_rx + entry;

		/* Get UMEM frame handle */
		if (!xsk_umem_peek_addr(priv->umems_storage[queue],
					&frame_handle)) {
			if (likely(!rx_q->rx_empty))
				break;

			/* Send socket notification to refill FQ queue */
			if (xsk_umem_uses_need_wakeup(priv->umems_storage[queue]))
				xsk_set_rx_need_wakeup(priv->umems_storage[queue]);
		}

		dirty--;

		buf->dma_addr += hr;
		buf->cpu_addr += hr;
		buf->dma_addr = xdp_umem_get_dma(umem, frame_handle);
		buf->cpu_addr = xdp_umem_get_data(umem, frame_handle);
		buf->handle = frame_handle + umem->headroom;

		/* Confirm acquiring an element */
		xsk_umem_discard_addr(priv->umems_storage[queue]);

		rx_q->rx_empty = false;

		buf_size = priv->umems_storage[queue]->chunk_size_nohr - XDP_PACKET_HEADROOM;
		dma_sync_single_range_for_device(priv->device, buf->dma_addr,
						 buf->page_offset, buf_size,
						 DMA_BIDIRECTIONAL);
		if (dma_mapping_error(priv->device, buf->dma_addr)) {
			netdev_err(priv->dev, "Rx DMA map failed\n");
			break;
		}

		stmmac_set_desc_addr(priv, p, buf->dma_addr);
		stmmac_refill_desc3(priv, rx_q, p);

		dma_wmb();
		stmmac_set_rx_owner(priv, p, priv->use_riwt);
		entry = STMMAC_GET_ENTRY(entry, DMA_RX_SIZE);

		++cleaned;
	}
	rx_q->dirty_rx = entry;

	atomic_set(&rx_q->lock, 0);

	DBG("%s<--\n", __FUNCTION__);

	return cleaned;
}

/**
 * af_xdp_refill_timer() - run refill procedure periodically
 * @t: timer structure
 */
static void af_xdp_refill_timer(struct timer_list *t)
{
	struct stmmac_rx_queue *rx_q = from_timer(rx_q, t, rx_refill_timer);

	af_xdp_rx_refill(rx_q->priv_data, rx_q->queue_index);

	mod_timer(&rx_q->rx_refill_timer,
		  jiffies + msecs_to_jiffies(AF_XDP_REFILL_MS));
}

/**
 * af_xdp_service_timer() - make deferred initialization if necessary
 * @t: timer structure
 *
 * It is possible that there are no descriptors during prime initialization so
 * try to acquire it later and initialize rings again
 */
static void af_xdp_service_timer(struct timer_list *t)
{
        struct stmmac_rx_queue *rx_q = from_timer(rx_q, t, rx_init_timer);
        struct stmmac_priv *priv = rx_q->priv_data;
        int is_refilled = 0;

        DBG("%s-->\n", __FUNCTION__);

        /* If we can't own UMEM frames do it a little bit later */
        is_refilled = af_xdp_rx_refill(priv, rx_q->queue_index);
        if (!is_refilled) {
        	mod_timer(&rx_q->rx_init_timer,
        		  jiffies + msecs_to_jiffies(AF_XDP_INITIAL_REFILL_MS));
            return;
        }

        af_xdp_initialize_rx_rings(priv->dev);
    	stmmac_mac_set(priv, priv->ioaddr, true);
    	stmmac_start_all_dma(priv);

    	/* Set up timer for periodical checking Rx queue state */
    	timer_setup(&rx_q->rx_refill_timer, af_xdp_refill_timer, 0);
    	mod_timer(&rx_q->rx_refill_timer,
    		  jiffies + msecs_to_jiffies(AF_XDP_REFILL_MS));

    	DBG("%s<--\n", __FUNCTION__);
}

/**
 * af_xdp_run_rx_timers() - just set up required timers
 * @priv: common structure
 */
static void af_xdp_run_rx_timers(struct stmmac_priv *priv)
{
	struct stmmac_rx_queue *rx_q = &priv->rx_queue[0];

	/* Now only a timer here */
	timer_setup(&rx_q->rx_init_timer, af_xdp_service_timer, 0);
	mod_timer(&rx_q->rx_init_timer,
		  jiffies + msecs_to_jiffies(AF_XDP_INITIAL_REFILL_MS));
}

/**
 * af_xdp_run_filter() - run XDP filter and make actions based on the result
 * @priv: common structure
 * @xdp: information for filter
 *
 * RETURN: action return code
 */
static int af_xdp_run_filter(struct stmmac_priv *priv, struct xdp_buff *xdp)
{
	struct bpf_prog *xdp_prog = READ_ONCE(priv->xdp_prog);
	struct xdp_frame *xdpf;
	int result = AF_XDP_PASS;
	int err;
	u64 offset;
	u32 act;

	rcu_read_lock();

	if (likely(xdp_prog))
		act = bpf_prog_run_xdp(xdp_prog, xdp);
	else
		return -1;

	offset = xdp->data - xdp->data_hard_start;
	xdp->handle = xsk_umem_adjust_offset(priv->umems_storage[0],
					     xdp->handle,
					     offset);

	switch (act) {
	case XDP_PASS:
		break;
	case XDP_TX:
		xdpf = convert_to_xdp_frame(xdp);
		if (unlikely(!xdpf)) {
			result = AF_XDP_CONSUMED;
			break;
		}
		result = af_xdp_xmit_frame(priv, xdpf);
		break;
	case XDP_REDIRECT:
		err = xdp_do_redirect(priv->dev, xdp, xdp_prog);
		result = !err ? AF_XDP_REDIR : AF_XDP_CONSUMED;
		break;
	default:
		bpf_warn_invalid_xdp_action(act);
	case XDP_ABORTED:
	case XDP_DROP:
		result = AF_XDP_CONSUMED;
		break;
	}

	rcu_read_unlock();

	return result;
}

/**
 * af_xdp_poll_rx() - NAPI poll routine
 * @priv: common structure
 * @limit: NAPI budget
 * @queue: queue index
 *
 * RETURN: numbers of received packets
 */
int af_xdp_poll_rx(struct stmmac_priv *priv, int budget, u32 queue)
{
	struct stmmac_rx_queue *rx_q = &priv->rx_queue[queue];
	struct stmmac_channel *ch = &priv->channel[queue];
	struct sk_buff *skb = NULL;
	struct xdp_buff xdp;
	unsigned int next_entry = rx_q->cur_rx;
	unsigned int count = 0;
	unsigned int error = 0;
	unsigned int len = 0;
	unsigned int xdp_res;
	int status = 0;
	int coe = priv->hw->rx_csum;
	bool do_flush = false;

	while ((count < budget) && !rx_q->rx_empty) {
		struct stmmac_xsk_desc_map *buf;
		struct dma_desc *np, *p;
		unsigned int sec_len;
		unsigned int hlen = 0, prev_len = 0;
		enum pkt_hash_types hash_type;
		int entry;
		u32 hash;

		if (!count && rx_q->state_saved) {
			skb = rx_q->state.skb;
			error = rx_q->state.error;
			len = rx_q->state.len;
		} else {
			rx_q->state_saved = false;
			skb = NULL;
			error = 0;
			len = 0;
		}

		if (count >= budget)
			break;

read_again:
		sec_len = 0;
		entry = next_entry;
		buf = rx_q->desc_map + entry;

		if (priv->extend_desc)
			p = (struct dma_desc *)(rx_q->dma_erx + entry);
		else
			p = rx_q->dma_rx + entry;

		status = stmmac_rx_status(priv, &priv->dev->stats,
					  &priv->xstats, p);

		/* Check if descriptor is ready to use */
		if (unlikely(status & dma_own))
			break;

		if (likely(STMMAC_GET_ENTRY(rx_q->cur_rx, DMA_RX_SIZE) != rx_q->dirty_rx)) {
			rx_q->cur_rx = STMMAC_GET_ENTRY(rx_q->cur_rx, DMA_RX_SIZE);
			next_entry = rx_q->cur_rx;
		} else {
			/* Queue is empty. Notify refill function greedily */
			rx_q->rx_empty = true;
		}


		if (priv->extend_desc)
			np = (struct dma_desc *)(rx_q->dma_erx + next_entry);
		else
			np = rx_q->dma_rx + next_entry;

		prefetch(np);
		prefetch(buf->cpu_addr);

		if (priv->extend_desc)
			stmmac_rx_extended_status(priv, &priv->dev->stats,
						  &priv->xstats,
						  rx_q->dma_erx + entry);

		if (unlikely(status == discard_frame)) {
			error = 1;
			if (!priv->hwts_rx_en)
				priv->dev->stats.rx_errors++;
		}

		if (unlikely(error && (status & rx_not_ls)))
			goto read_again;

		if (unlikely(error)) {
			dev_kfree_skb(skb);
			count++;
			continue;
		}

		/* Buffer is good. Go on. */
		if (likely(status & rx_not_ls)) {
			len += priv->dma_buf_sz;
		} else {
			prev_len = len;
			len = stmmac_get_rx_frame_len(priv, p, coe);

			/* ACS is set; GMAC core strips PAD/FCS for IEEE 802.3
			 * Type frames (LLC/LLC-SNAP)
			 *
			 * llc_snap is never checked in GMAC >= 4, so this ACS
			 * feature is always disabled and packets need to be
			 * stripped manually.
			 */
			if (unlikely(priv->synopsys_id >= DWMAC_CORE_4_00) ||
				unlikely(status != llc_snap)) {
					len -= ETH_FCS_LEN;
			}
		}

		/* Sanity check */
		if (!len)
			continue;

		/* It's time to run XDP program */
		dma_sync_single_range_for_cpu(priv->device, buf->dma_addr,
					      buf->page_offset, len,
					      DMA_BIDIRECTIONAL);

		/* Prepare frame data for filter */
		xdp.rxq = &rx_q->xdp_rxq;
		xdp.data = buf->cpu_addr;
		xdp.data_meta = xdp.data;
		xdp.data_hard_start = xdp.data - XDP_PACKET_HEADROOM;
		xdp.data_end = xdp.data + len;
		xdp.handle = buf->handle;

		xdp_res = af_xdp_run_filter(priv, &xdp);
		if ((xdp_res == AF_XDP_REDIR)) {
			count++;
			do_flush = true;
			continue;
		} else if ((xdp_res == AF_XDP_TX) || (xdp_res == AF_XDP_CONSUMED)) {
			count++;
			continue;
		}

		/* Nothing to redirect. Due to filter result we pass a frame
		 * to the network stack */

		/* Allocate SKB if necessary */
		if (!skb) {
			int ret = stmmac_get_rx_header_len(priv, p, &hlen);

			if (priv->sph && !ret && (hlen > 0)) {
				sec_len = len;
				if (!(status & rx_not_ls))
					sec_len = sec_len - hlen;
				len = hlen;

				priv->xstats.rx_split_hdr_pkt_n++;
			}

			skb = napi_alloc_skb(&ch->rx_napi, len);
			if (!skb) {
				priv->dev->stats.rx_dropped++;
				count++;
				continue;
			}

			dma_sync_single_range_for_cpu(priv->device,
						      buf->dma_addr,
						      buf->page_offset, len,
						      DMA_BIDIRECTIONAL);

			skb_copy_to_linear_data(skb, buf->cpu_addr, len);
			skb_put(skb, len);
		} else {
			unsigned int buf_len = len - prev_len;

			if (likely(status & rx_not_ls))
				buf_len = priv->dma_buf_sz;

			dma_sync_single_range_for_cpu(priv->device,
						      buf->dma_addr,
						      buf->page_offset, len,
						      DMA_BIDIRECTIONAL);

			skb_add_rx_frag(skb, skb_shinfo(skb)->nr_frags,
					buf->cpu_addr, 0, buf_len,
					priv->dma_buf_sz);
		}

		if (likely(status & rx_not_ls))
			goto read_again;

		/* Got entire packet into SKB. Finish it. */
		skb->protocol = eth_type_trans(skb, priv->dev);

		if (unlikely(!coe))
			skb_checksum_none_assert(skb);
		else
			skb->ip_summed = CHECKSUM_UNNECESSARY;

		if (!stmmac_get_rx_hash(priv, p, &hash, &hash_type))
			skb_set_hash(skb, hash, hash_type);

		skb_record_rx_queue(skb, queue);
		napi_gro_receive(&ch->rx_napi, skb);

		priv->dev->stats.rx_packets++;
		priv->dev->stats.rx_bytes += len;
		count++;
	}

	if (status & rx_not_ls) {
		rx_q->state_saved = true;
		rx_q->state.skb = skb;
		rx_q->state.error = error;
		rx_q->state.len = len;
	}

	if (do_flush)
		xdp_do_flush_map();

	af_xdp_rx_refill(priv, queue);

	priv->xstats.rx_pkt_n += count;

	return count;
}

/**
 * af_xdp_setup_filter() - setup a new XDP filter
 * @dev: common structure
 * @prog: filter program
 *
 * RETURN: 0 - ok
 */
static int af_xdp_setup_filter(struct net_device *dev, struct bpf_prog *filter)
{
	struct stmmac_priv *priv = netdev_priv(dev);
	struct bpf_prog *old_prog;
	bool need_reset;

	DBG("%s-->\n", __FUNCTION__);

	old_prog = xchg(&priv->xdp_prog, filter);
	need_reset = (!!filter != !!old_prog);

	if (old_prog)
		bpf_prog_put(old_prog);

	DBG("%s<--\n", __FUNCTION__);

	return 0;
}

/**
 * af_xdp_free_dma_resources() - free DMA resources for each ring
 * @priv: common structure
 */
void af_xdp_free_dma_resources(struct stmmac_priv *priv)
{
	u32 rx_count = priv->plat->rx_queues_to_use;
	u32 queue;

	DBG("%s-->\n", __FUNCTION__);

	/* Free RX queue resources */
	for (queue = 0; queue < rx_count; queue++) {
		struct stmmac_rx_queue *rx_q = &priv->rx_queue[queue];

		/* Free DMA regions of consistent memory previously allocated */
		if (!priv->extend_desc)
			dma_free_coherent(priv->device,
					  DMA_RX_SIZE * sizeof(struct dma_desc),
					  rx_q->dma_rx, rx_q->dma_rx_phy);
		else
			dma_free_coherent(priv->device,
					  DMA_RX_SIZE * sizeof(struct dma_extended_desc),
					  rx_q->dma_erx, rx_q->dma_rx_phy);
	}

	DBG("%s<--\n", __FUNCTION__);
}

/**
 * af_xdp_alloc_dma_resources() - allocate DMA resources for each ring
 * @priv: common strucutre
 *
 * RETURN: 0 - ok	-ENOMEM - fail
 */
int af_xdp_alloc_dma_resources(struct stmmac_priv *priv)
{
	u32 rx_count = priv->plat->rx_queues_to_use;
	u32 queue;
	u32 err_queue;
	size_t len;

	DBG("%s-->\n", __FUNCTION__);

	/* RX queues buffers and DMA */
	for (queue = 0; queue < rx_count; ++queue) {
		struct stmmac_rx_queue *rx_q = &priv->rx_queue[queue];

		rx_q->queue_index = queue;
		rx_q->priv_data = priv;

		if (priv->extend_desc) {
			len = DMA_RX_SIZE * sizeof(struct dma_extended_desc);
			rx_q->dma_erx = dma_alloc_coherent(priv->device,
							   len,
							   &rx_q->dma_rx_phy,
							   GFP_KERNEL);
			if (!rx_q->dma_erx)
				goto err_dma;
		} else {
			len = DMA_RX_SIZE * sizeof(struct dma_desc);
			rx_q->dma_rx = dma_alloc_coherent(priv->device,
							  len,
							  &rx_q->dma_rx_phy,
							  GFP_KERNEL);
			if (!rx_q->dma_rx)
				goto err_dma;
		}
	}

	DBG("%s<--\n", __FUNCTION__);

	return 0;

err_dma:
	err_queue = queue;
	for (queue = 0; queue < err_queue; ++queue) {
		struct stmmac_rx_queue *rx_q = &priv->rx_queue[queue];

		if (priv->extend_desc) {
			len = DMA_RX_SIZE * sizeof(struct dma_extended_desc);
			dma_free_coherent(priv->device,
					 len, rx_q->dma_erx, rx_q->dma_rx_phy);
		} else {
			len = DMA_RX_SIZE * sizeof(struct dma_desc);
			dma_free_coherent(priv->device,
					  len,  rx_q->dma_rx, rx_q->dma_rx_phy);
		}
	}

	return -ENOMEM;
}

/**
 * af_xdp_txrx_ring_disable() - disable ring
 * @priv: gemac main structure
 * @ring: number of associated ring
 *
 * Stop ring, free its resources, stop DMA
 */
static void af_xdp_txrx_ring_disable(struct stmmac_priv *priv, int ring)
{
	struct stmmac_channel *ch = &priv->channel[ring];

	DBG("%s-->\n", __FUNCTION__);

	netif_tx_stop_queue(netdev_get_tx_queue(priv->dev, ring));

	/* Stop GEMAC engine */
	stmmac_mac_set(priv, priv->ioaddr, false);
	stmmac_stop_all_dma(priv);

	napi_disable(&ch->rx_napi);
	napi_disable(&ch->tx_napi);

	if (use_umem(priv, ring)) {
		/* Disable UMEMs resources */
		xdp_do_flush_map();
		xdp_rxq_info_unreg(&priv->rx_queue[ring].xdp_rxq);
		af_xdp_free_frames(priv, ring);
		af_xdp_free_dma_resources(priv);
	} else {
		/* Disable pool of pages resources */
		free_dma_rx_desc_resources(priv);
	}

	DBG("%s<--\n", __FUNCTION__);
}

/**
 * af_xdp_txrx_ring_enable() - enable ring
 * @priv: gemac main structure
 * @ring: number of associated ring
 *
 * Prepare a ring for active state. Allocate resources, run DMA
 *
 * RETURN: 0 - ok	-ENOMEM - failure
 */
static int af_xdp_txrx_ring_enable(struct stmmac_priv *priv, int ring)
{
	struct stmmac_channel *ch = &priv->channel[ring];
	struct stmmac_rx_queue *rx_q = &priv->rx_queue[ring];

	DBG("%s-->\n", __FUNCTION__);

	if (use_umem(priv, ring)) {
		priv->zca.free = af_xdp_reuse_frame;
		xdp_rxq_info_reg_mem_model(&priv->rx_queue[ring].xdp_rxq,
					   MEM_TYPE_ZERO_COPY,
					   &priv->zca);

		if (af_xdp_alloc_dma_resources(priv))
			goto err;
		af_xdp_alloc_frames(priv, ring);
		af_xdp_run_rx_timers(priv);
		atomic_set(&rx_q->lock, 0);
	} else {
		if (alloc_dma_rx_desc_resources(priv))
			goto err;
		init_dma_desc_rings(priv->dev, GFP_KERNEL);
		af_xdp_gemac_setup(priv->dev, true);
	}

	netif_tx_start_queue(netdev_get_tx_queue(priv->dev, ring));

	stmmac_init_rx_chan(priv, priv->ioaddr, priv->plat->dma_cfg,
			    rx_q->dma_rx_phy, ring);

	rx_q->rx_tail_addr = rx_q->dma_rx_phy +
			     (DMA_RX_SIZE * sizeof(struct dma_desc));
	stmmac_set_rx_tail_ptr(priv, priv->ioaddr,
			       rx_q->rx_tail_addr, ring);

	rx_q->cur_rx = 0;
	rx_q->dirty_rx = 0;

	/* Ready to start NAPIs */
	napi_enable(&ch->rx_napi);
	napi_enable(&ch->tx_napi);

	stmmac_mac_set(priv, priv->ioaddr, true);
	stmmac_start_all_dma(priv);

	DBG("%s<--\n", __FUNCTION__);

	return 0;

err:
	return -ENOMEM;
}

/**
 * af_xdp_umem_enable() - allocate resources and enable UMEM
 * @priv: common structure
 * @umem: UMEM pointer
 * @qid: queue index
 *
 * RETURN: 0 - ok	< 0 - fail
 */
static int af_xdp_umem_enable(struct stmmac_priv *priv, struct xdp_umem *umem,
			      u16 qid)
{
	struct xdp_umem_fq_reuse *reuseq;
	int err;
	bool if_running;

	DBG("%s-->\n", __FUNCTION__);


	if (qid >= priv->plat->rx_queues_to_use)
		return -EINVAL;

	err = xdp_rxq_info_reg(&priv->rx_queue[0].xdp_rxq, priv->dev, 0);
	if (err)
		return err;

	reuseq = xsk_reuseq_prepare(DMA_RX_SIZE);
	if (!reuseq)
		return -ENOMEM;

	if_running = netif_running(priv->dev);
	if (if_running)
		af_xdp_txrx_ring_disable(priv, qid);

	/* Setup UMEM and XDP auxiliary data */
	if (af_xdp_insert_umem(priv, umem, qid))
		return err;

	xsk_reuseq_free(xsk_reuseq_swap(umem, reuseq));

	err = af_xdp_umem_map_resources(priv, umem);
	if (err)
		return err;

	priv->umems_storage[qid] = umem;

	/* Enable rings */
	if (if_running)
		af_xdp_txrx_ring_enable(priv, qid);

	DBG("%s<--\n", __FUNCTION__);

	return 0;
}

/**
 * af_xdp_umem_disable() - free resources and disable UMEM
 * @priv: common structure
 * @qid: Index of a queue containing UMEM
 *
 * RETURN: 0 - ok	< 0 - fail
 */
static int af_xdp_umem_disable(struct stmmac_priv *priv, u16 qid)
{
	struct xdp_umem *umem = xdp_get_umem_from_qid(priv->dev, qid);
	struct stmmac_rx_queue *rx_q = &priv->rx_queue[qid];
	bool if_running;
	
	DBG("%s-->\n", __FUNCTION__);

	if (!umem)
		return -EINVAL;

	if_running = netif_running(priv->dev);

	if (if_running)
		af_xdp_txrx_ring_disable(priv, qid);

	af_xdp_umem_unmap_resources(priv, umem);
	af_xdp_remove_umem(priv, qid);

	del_timer_sync(&rx_q->rx_init_timer);
	del_timer_sync(&rx_q->rx_refill_timer);

	if (if_running)
		af_xdp_txrx_ring_enable(priv, qid);

	DBG("%s<--\n", __FUNCTION__);

	return 0;
}

/**
 * af_xdp_umem_setup() - wrapper for enable/disable UMEM
 * @priv: common structure
 * @umem: UMEM pointer
 * @qid: queue index
 *
 * RETURN: 0 - ok	< 0 - fail
 */
static int af_xdp_umem_setup(struct stmmac_priv *priv, struct xdp_umem *umem,
			     u16 qid)
{
	return umem ? af_xdp_umem_enable(priv, umem, qid) : \
		      af_xdp_umem_disable(priv, qid);
}

/**
 * af_xdp_initialize_dma() - initialize DMA engine
 * @priv: common structure
 */
int af_xdp_initialize_dma(struct stmmac_priv *priv)
{
	u32 rx_channels_count = priv->plat->rx_queues_to_use;
	u32 tx_channels_count = priv->plat->tx_queues_to_use;
	u32 dma_csr_ch_number = max(rx_channels_count, tx_channels_count);
	struct stmmac_rx_queue *rx_q;
	struct stmmac_tx_queue *tx_q;
	u32 chan = 0;
	int atds = 0;
	int ret = 0;

	stmmac_dma_init(priv, priv->ioaddr, priv->plat->dma_cfg, atds);

	if (priv->plat->axi)
		stmmac_axi(priv, priv->ioaddr, priv->plat->axi);

	/* DMA CSR Channel configuration */
	for (chan = 0; chan < dma_csr_ch_number; chan++)
		stmmac_init_chan(priv, priv->ioaddr, priv->plat->dma_cfg, chan);

	/* DMA RX Channel Configuration */
	for (chan = 0; chan < rx_channels_count; chan++) {
		rx_q = &priv->rx_queue[chan];

		stmmac_init_rx_chan(priv, priv->ioaddr, priv->plat->dma_cfg,
				    rx_q->dma_rx_phy, chan);

		rx_q->rx_tail_addr = rx_q->dma_rx_phy +
				     (DMA_RX_SIZE * sizeof(struct dma_desc));
		stmmac_set_rx_tail_ptr(priv, priv->ioaddr,
				       rx_q->rx_tail_addr, chan);
	}

	/* DMA TX Channel Configuration */
	for (chan = 0; chan < tx_channels_count; chan++) {
		tx_q = &priv->tx_queue[chan];

		stmmac_init_tx_chan(priv, priv->ioaddr, priv->plat->dma_cfg,
				    tx_q->dma_tx_phy, chan);

		tx_q->tx_tail_addr = tx_q->dma_tx_phy;
		stmmac_set_tx_tail_ptr(priv, priv->ioaddr,
				       tx_q->tx_tail_addr, chan);
	}

	return ret;
}

/**
 * af_xdp_bpf() - Network stack callback. Setup filter or enable/disable UMEM
 * @priv: common structure
 * @bpf: filter pointer
 *
 * RETURN: 0 - ok	< 0 - fail
 */
int af_xdp_bpf(struct net_device *dev, struct netdev_bpf *bpf)
{
	struct stmmac_priv *priv = netdev_priv(dev);

	DBG("%s-->\n", __FUNCTION__);

	switch (bpf->command) {
		case XDP_SETUP_PROG:
			if (!priv->umems_storage)
				return -EPERM;
			return af_xdp_setup_filter(dev, bpf->prog);
		case XDP_QUERY_PROG:
			bpf->prog_id = priv->xdp_prog ? priv->xdp_prog->aux->id : 0;
			return 0;
		case XDP_SETUP_XSK_UMEM:
			return af_xdp_umem_setup(priv, bpf->xsk.umem,
						 bpf->xsk.queue_id);
		default:
			return -EINVAL;
	}

	DBG("%s<--\n", __FUNCTION__);

	return -EPERM;
}

/**
 * af_xdp_xmit() - redirect to other network device
 * @dev: pointer to a redirected network device
 * @n: number of frames
 * @xdp: frames array
 * @flags: network stack flags
 *
 * RETURN: number of redirected frames
 */
int af_xdp_xmit(struct net_device *dev, int n, struct xdp_frame **xdp,
		    u32 flags)
{
	struct stmmac_priv *priv = netdev_priv(dev);
	int drops = 0;
	int result;
	int i;

	DBG("%s-->\n", __FUNCTION__);

	for (i = 0; i < n; ++i) {
		result = af_xdp_xmit_frame(priv, xdp[i]);
		if (result != AF_XDP_TX) {
			xdp_return_frame_rx_napi(xdp[i]);
			drops++;
		}
	}

	DBG("%s<--\n", __FUNCTION__);

	return n - drops;
}

/**
 * af_xdp_wakeup() - Wake up Rx or/and Tx queue
 * @dev: associated network device
 * @queue_id: queue index
 * @flags: action
 *
 * Socket notifies driver in case of lack resources
 *
 * RETURN: 0 - ok
 */
int af_xdp_wakeup(struct net_device *dev, u32 queue_id, u32 flags)
{
	struct stmmac_priv *priv = netdev_priv(dev);

	DBG("%s-->\n", __FUNCTION__);

	/* Wake up request can be sent from poll of socket */

	if (flags & XDP_WAKEUP_TX)
		/* Run NAPI tx engine to kick transfer or clean descriptors */
		if (likely(napi_schedule_prep(&priv->channel[0].tx_napi))) {
			__napi_schedule(&priv->channel[queue_id].tx_napi);
			xsk_clear_tx_need_wakeup(priv->umems_storage[queue_id]);
		}

	if (flags & XDP_WAKEUP_RX)
		/* Run NAPI rx engine to start receiving or clean descriptors */
		if (likely(napi_schedule_prep(&priv->channel[0].rx_napi))) {
			__napi_schedule(&priv->channel[queue_id].rx_napi);
			xsk_clear_rx_need_wakeup(priv->umems_storage[queue_id]);
		}

	DBG("%s<--\n", __FUNCTION__);

	return 0;
}
