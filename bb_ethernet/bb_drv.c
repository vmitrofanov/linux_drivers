#include <linux/netdevice.h>

#include "bb_drv.h"
#include "bb_eth.h"

#ifdef DBG
static void bb_print_buf(void *buf, int len)
{
	char print_buf[256];
	int i;

	if (!len || !buf || (len > 1500))
		return;

	pr_err(">> buf_len: %d \n", len);
	for (i = 0; i < len; i += 32) {
		unsigned int print_len = min(len - i, 32);

		hex_dump_to_buffer(buf + i, print_len, 32, 1, print_buf, 256, false);
		pr_err("%s \n", print_buf);
	}
}
#endif

/**
 * bb_gemac_start_xmit() - start transferring net-data gotten from the stack
 * @sk: net-data
 * @gemac: associated net-device
 */
netdev_tx_t bb_gemac_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct gemac_private *gemac = netdev_priv(ndev);
	struct ring *tx_ring = &gemac->tx_ring;
	struct dma_desc *sw_desc;
	struct hw_desc *hw_desc;
	struct netdev_queue *txq;
	int out = 2000;
	int circles = 0;
	int q_idx = skb_get_queue_mapping(skb);
	txq = netdev_get_tx_queue(ndev, q_idx);

//	DBG("-->%s\n", __FUNCTION__);
	sw_desc = &tx_ring->desc_ring[tx_ring->cur];

	sw_desc->skb = skb;
	sw_desc->buf_size = skb->len;
	sw_desc->buf_dma = dma_map_single(&gemac->ndev->dev, skb->data,
					  sw_desc->buf_size, DMA_TO_DEVICE);
	if (dma_mapping_error(&gemac->ndev->dev, sw_desc->buf_dma)) {
		pr_err("dma_map_single failed, dropping packet\n");
		return NETDEV_TX_BUSY;
	}

	hw_desc = sw_desc->desc_cpu;
	/* 1 element queue */
	hw_desc->next = 0;
	hw_desc->buf = sw_desc->buf_dma;
	hw_desc->len = sw_desc->buf_size;
	hw_desc->opt = CPDMA_DESC_OWNER | CPDMA_DESC_SOP | CPDMA_DESC_EOP |
		       sw_desc->buf_size | BIT(20) | 0x10000; //TODO:
	wmb();

	/* Start transmission */
	writel(sw_desc->desc_dma, &gemac->dma.stream->tx_hdp[0]);

	//TODO: delete after debug
	while ((readl(&gemac->dma.stream->tx_hdp[0]) != 0) && --out) {
		++circles;
	}
	if (!out)
		pr_err("run out of out!!!!!!!!!!\n");

	writel(sw_desc->desc_dma, &gemac->dma.stream->tx_cp[0]);
	DBG("TX(%d)   circles: %d   d0:d1:d2:d3 = %08x:%08x:%08x:%08x\n",
		tx_ring->cur, circles, hw_desc->next, hw_desc->buf,
		hw_desc->len, hw_desc->opt);

//	bb_print_buf(skb->data, skb->len);
	gemac->ndev->stats.tx_packets++;
	gemac->ndev->stats.tx_bytes += skb->len;

	tx_ring->cur = NEXT_DESC(tx_ring->cur);

//	DBG("<--%s\n", __FUNCTION__);

	return NETDEV_TX_OK;
}

static int process_rx_ring(struct ring *rx_ring, int budget)
{
	struct gemac_private *gemac = container_of(rx_ring, struct gemac_private,
						   rx_ring);
	struct hw_desc *hw_desc;
	struct dma_desc *desc = rx_ring->desc_ring + rx_ring->cur;
	struct sk_buff *skb;
	u32 *buf;
	int processed = 0;
	int active_channels_mask = readl(&gemac->eth_wr.regs->c0_rx_stat);

	DBG("> channel: %08x\n", active_channels_mask);

	DBG("process_rx_ring >>>>> CUR: %d DIRTY: %d\n", rx_ring->cur, rx_ring->dirty);

	while(processed < budget) {
		hw_desc = desc->desc_cpu;

		/* Is it ready or not */
		if (hw_desc->opt & CPDMA_DESC_OWNER)
			break;

		if (hw_desc->opt & CPDMA_DESC_EOP)
			desc->buf_size = hw_desc->len & CPDMA_PKT_MAX_LEN;

		desc->buf_size -= 4;

		dma_sync_single_for_cpu(&gemac->pdev->dev, desc->buf_dma,
					desc->buf_size,
					DMA_FROM_DEVICE);

		buf = desc->buf_cpu;
		skb = netdev_alloc_skb(gemac->ndev,
				       MTU_TO_BUF_SIZE(gemac->ndev->mtu));
		if (!skb)
			break;

		desc->skb = skb;

		skb_reserve(skb, NET_IP_ALIGN);
		skb_copy_to_linear_data(skb, buf, desc->buf_size);
		skb_put(skb, desc->buf_size);
		skb->protocol = eth_type_trans(skb, gemac->ndev);
		gemac->ndev->stats.rx_packets++;
		gemac->ndev->stats.rx_bytes += desc->buf_size;

#if 0
		skb_checksum_none_assert(skb);
		skb->dev = gemac->ndev;
		skb->ip_summed = CHECKSUM_UNNECESSARY;
#endif

		if (NET_RX_SUCCESS != netif_receive_skb(skb))
			DBG("packet dropped by stack\n");

		DBG("RX(%d): d0:d1:d2:d3 = %08x:%08x:%08x:%08x\n",
		       rx_ring->cur, hw_desc->next, hw_desc->buf, hw_desc->len,
		       hw_desc->opt);
//		bb_print_buf(buf/*skb->data*/, desc->buf_size);

		/* Try to clear interrupt */
		writel(desc->desc_dma, &gemac->dma.stream->rx_cp[0]);
		writel(CPDMA_EOI_RX, &gemac->dma.regs->cpdma_eoi_vector);

		++processed;
		rx_ring->cur = NEXT_DESC(rx_ring->cur);
		desc = rx_ring->desc_ring + rx_ring->cur;

		DBG("process_rx_ring >>>>> +++ CUR: %d DIRTY: %d\n", rx_ring->cur, rx_ring->dirty);
	}

	return processed;
}

inline int get_dirty_num(struct gemac_private *gemac, struct ring *ring)
{
	size_t ring_size = gemac->ring_size;
	int cur = ring->cur;
	int dirty = ring->dirty;

	return (cur >= dirty) ? cur - dirty : ring_size - dirty + cur;
}

int refill_rx(struct gemac_private *gemac)
{
	struct ring *rx = &gemac->rx_ring;
	struct dma_desc *sw_desc;
	struct hw_desc *hw_desc;
	int dirty_num = get_dirty_num(gemac, rx);
	int refilled = 0;
	u32 opt = 0;

	while (dirty_num--) {
		++refilled;
		sw_desc = rx->desc_ring + rx->dirty;
		hw_desc = (struct hw_desc *)sw_desc->desc_cpu;

		//todo: has it to be here or not
//		dev_kfree_skb_any(sw_desc->skb);

		dma_sync_single_for_device(&gemac->pdev->dev,
					   sw_desc->buf_dma,
					   gemac->buf_size,
					   DMA_FROM_DEVICE);

		opt = hw_desc->opt;
		hw_desc->buf = sw_desc->buf_dma;
		hw_desc->len = gemac->buf_size;
		hw_desc->opt = CPDMA_DESC_OWNER;
		if (opt & CPDMA_DESC_EOQ) {
			hw_desc->next = 0;
			if (readl(&gemac->dma.stream->rx_hdp[0]) != 0)
				pr_err("ERR: ETH: DMA: rx_hdp != 0\n");
			writel(rx->desc_ring->desc_dma, &gemac->dma.stream->rx_hdp[0]);
			break;
		} else {
			rx->dirty = NEXT_DESC(rx->dirty);
			sw_desc = rx->desc_ring + rx->dirty;
			hw_desc->next = sw_desc->desc_dma;
		}
	}

	return refilled;
}

int refill_tx(struct gemac_private *gemac)
{
	struct ring *tx = &gemac->tx_ring;
	struct dma_desc *sw_desc;
	struct hw_desc *hw_desc;
	int dirty_num = get_dirty_num(gemac, tx);
	int refilled = 0;

//	DBG("-->%s\n", __FUNCTION__);
//	DBG("tx_dirty_num: %d\n", dirty_num);

	while (dirty_num--) {
		sw_desc = tx->desc_ring + tx->dirty;
		hw_desc = (struct hw_desc *)sw_desc->desc_cpu;

		if (hw_desc->opt & CPDMA_DESC_OWNER)
			break;

		dev_kfree_skb_any(sw_desc->skb);

		writel(sw_desc->desc_dma, &gemac->dma.stream->tx_cp[0]);

		tx->dirty = NEXT_DESC(tx->dirty);
		++refilled;
	}

	writel(CPDMA_EOI_TX, &gemac->dma.regs->cpdma_eoi_vector);

//	DBG("<--%s\n", __FUNCTION__);

	return refilled;
}

int poll_rx(struct napi_struct *rx_napi, int weight)
{
	struct gemac_private *gemac = netdev_priv(rx_napi->dev);
	struct ring *rx_ring = container_of(rx_napi, struct ring, napi);
	int processed;
	int refilled;


//	DBG("-->%s\n", __FUNCTION__);
	processed = process_rx_ring(rx_ring, weight);
	if ((processed < weight) && napi_complete_done(rx_napi, processed)) {
		DBG("stop RX NAPI & enable RX interrupts\n");
		writel(0xFF, &gemac->eth_wr.regs->c0_rx_en);
		enable_irq(gemac->dt_irq_rx);
	}

	refilled = refill_rx(gemac);
//	DBG("refilled rx: %d\n", refilled);

//	DBG("<--%s\n", __FUNCTION__);

	return processed;
}

int poll_tx(struct napi_struct *tx_napi, int weight)
{
	struct gemac_private *gemac = netdev_priv(tx_napi->dev);
	int refilled;

	DBG("---------------------------------------------->%s\n", __FUNCTION__);

	refilled = refill_tx(gemac);
	if ((refilled < weight) && napi_complete_done(tx_napi, refilled)) {
		DBG("stop TX NAPI & enable TX interrupts\n");
		enable_irq(gemac->dt_irq_tx);
	}

	DBG("<--%s\n", __FUNCTION__);

	return refilled;
}

/*
 * TODO: rewrite interrupt clearing due to page 2015 (14.3.1.3)
 */
irqreturn_t rx_interrupt(int irq, void *dev_id)
{
	struct gemac_private *gemac = dev_id;

	DBG("-->%s \n", __FUNCTION__);

//	writel(0x0, &gemac->dma.regs->dma_intmask_clear);
//	writel(0x0, &gemac->dma.regs->dma_intmask_set);

//	DBG("> channel: %08x\n", readl(&gemac->eth_wr.regs->c0_rx_stat));

//	writel(CPDMA_EOI_RX, &gemac->dma.regs->cpdma_eoi_vector);
//	writel(0, &gemac->eth_wr.regs->c0_rx_en); not necessary

	disable_irq_nosync(gemac->dt_irq_rx);
	napi_schedule_irqoff(&gemac->rx_ring.napi);

	DBG("<--%s\n", __FUNCTION__);

	return IRQ_HANDLED;
}

/*
 * TODO: rewrite interrupt clearing due to page 2015 (14.3.1.3)
 */
irqreturn_t tx_interrupt(int irq, void *dev_id)
{
	struct gemac_private *gemac = dev_id;

	DBG("-->%s \n", __FUNCTION__);

//	writel(CPDMA_EOI_TX, &gemac->dma.regs->cpdma_eoi_vector);
//	writel(0, &gemac->eth_wr.regs->c0_tx_en);

	disable_irq_nosync(gemac->dt_irq_tx);
	napi_schedule_irqoff(&gemac->tx_ring.napi);

	DBG("<--%s\n", __FUNCTION__);

	return IRQ_HANDLED;
}

/*
 * TODO: rewrite interrupt clearing due to page 2015 (14.3.1.3)
 */
void bb_enable_interrupts(struct gemac_private *gemac, int channels_mask)
{
	/* Enable pacing interrupts */
	writel(channels_mask & BB_INT_CHAN_MASK, &gemac->eth_wr.regs->c0_tx_en);
	writel(channels_mask & BB_INT_CHAN_MASK, &gemac->eth_wr.regs->c0_rx_en);

	/* Unmask interrupts channels */
	writel(channels_mask & BB_INT_CHAN_MASK,
	       &gemac->dma.regs->tx_intmask_set);
	writel(channels_mask & BB_INT_CHAN_MASK,
	       &gemac->dma.regs->rx_intmask_set);

	enable_irq(gemac->dt_irq_tx);
	enable_irq(gemac->dt_irq_rx);
}

int bb_alloc_ring(struct gemac_private *gemac, struct ring *ring, int alloc_buffers)
{
	struct ring *r = ring;
	struct dma_desc *desc;
	int i;
	int i_err;

	DBG("-->%s\n", __FUNCTION__);

	r->desc_ring = kmalloc(sizeof(struct dma_desc) * gemac->ring_size,
				  GFP_KERNEL);

	/* Allocate memory for HW descriptors */
	desc = r->desc_ring;
	for (i = 0; i < gemac->ring_size; ++i) {
		desc[i].desc_cpu = dma_alloc_coherent(&gemac->pdev->dev,
						      sizeof(struct hw_desc),
						      &desc[i].desc_dma,
						      GFP_KERNEL);
		if (!desc[i].desc_cpu) {
			pr_err("Error, can't allocate descriptor\n");
			i_err = i;
			goto err_alloc_coherent;
		}
	}

	if (!alloc_buffers)
		return 0;

	//TODO: it's possible to allocate memory from sk_buf from the beginning or from page_pool !!!
	/* Allocate memory for descriptor buffer */
	for (i = 0; i < gemac->ring_size; ++i) {
		desc[i].buf_cpu = kmalloc(gemac->buf_size, GFP_KERNEL);
		if (!desc[i].buf_cpu) {
			pr_err("Error, can't allocate buffer\n");
			i_err = i;
			goto err_alloc_buf;
		}
	}

	/* Map buffer memory to use with DMA */
	for (i = 0; i < gemac->ring_size; ++i) {
		desc[i].buf_dma = dma_map_single(&gemac->pdev->dev,
						 desc[i].buf_cpu,
						 gemac->buf_size,
						 DMA_FROM_DEVICE);

		if (dma_mapping_error(&gemac->pdev->dev, desc[i].buf_dma)) {
			pr_err("Error, can't map buffer\n");
			i_err = i;
			goto err_map_buf;
		}

		dma_sync_single_for_device(&gemac->pdev->dev,
					   desc[i].buf_dma,
					   gemac->buf_size,
					   DMA_FROM_DEVICE);
	}

	DBG("<--%s\n", __FUNCTION__);

	return 0;

err_map_buf:
	DBG("Failed on %d index\n", i_err);
	for (i = 0; i < i_err; ++i)
		; //TODO:

	/* Prepare full ring free for next step */
	i_err = gemac->ring_size;

err_alloc_buf:
	for (i = 0; i < i_err; ++i)
		; //TODO:

	/* Prepare full ring free for next step */
	i_err = gemac->ring_size;

err_alloc_coherent:
	for (i = 0; i < i_err; ++i)
		; //TODO: free coherent

	return -1;
}

int bb_init_rings(struct gemac_private *gemac)
{
	struct ring *ring;
	struct dma_desc *desc;
	struct hw_desc *hw_desc;
	int timeout_cnt = 50;
	int i;

	DBG("-->%s\n", __FUNCTION__);

	/* Reset DMA */
	writel(1, &gemac->dma.regs->cpdma_soft_reset);
	while (readl(&gemac->dma.regs->cpdma_soft_reset) && --timeout_cnt)
		msleep(1);

	if (readl(&gemac->dma.regs->cpdma_soft_reset))
		return -1;

	/* Zero ring head pointer */
	for (i = 0; i < BB_DMA_CHANNELS_NUMBER; ++i) {
		writel(0, &gemac->dma.stream->rx_hdp[i]);
		writel(0, &gemac->dma.stream->tx_hdp[i]);
	}

	/* Now only for Rx ring */
	ring = &gemac->rx_ring;
	desc = ring->desc_ring;

	for (i = 0; i < gemac->ring_size; ++i) {
		hw_desc = (struct hw_desc *)desc[i].desc_cpu;

		if (i + 1 < gemac->ring_size)
			hw_desc->next = desc[i + 1].desc_dma;
		else
			hw_desc->next = 0;
		hw_desc->buf = desc[i].buf_dma;
		hw_desc->len = gemac->buf_size;
		hw_desc->opt = CPDMA_DESC_OWNER;
	}

	ring->cur = ring->dirty = 0;

	/* Enable Rx queue */
	writel(desc[0].desc_dma, &gemac->dma.stream->rx_hdp[0]);

	DBG("<--%s\n", __FUNCTION__);

	return 0;
}

void bb_start_dma_engine(struct gemac_private *gemac)
{
	__raw_writel(1, &gemac->dma.regs->rx_control);
	__raw_writel(1, &gemac->dma.regs->tx_control);
}

void bb_stop_dma_engine(struct gemac_private *gemac)
{
	__raw_writel(0, &gemac->dma.regs->rx_control);
	__raw_writel(0, &gemac->dma.regs->tx_control);
}


