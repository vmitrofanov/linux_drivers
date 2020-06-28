#include <linux/netdevice.h>

#include "bb_drv.h"
#include "bb_eth.h"

#ifdef DEBUG
static void bb_print_buf(void *buf, int len)
{
	char print_buf[256];
	int i;

	if (!len || !buf || (len > 1500))
		return;

	pr_err(">> buf_len: %d \n", len);
	for (i = 0; i < len; i += 32) {
		unsigned int print_len = min(len - i, 32);

		hex_dump_to_buffer(buf + i, print_len, 32, 1, print_buf, 256,
				   false);
		pr_err("%s \n", print_buf);
	}
}
#else
static void bb_print_buf(void *buf, int len)
{
	return;
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
	int timeout_cls = 2000;
	int q_idx = skb_get_queue_mapping(skb);
	txq = netdev_get_tx_queue(ndev, q_idx);

	DBG("-->%s\n", __FUNCTION__);

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
	hw_desc->opt = sw_desc->buf_size | CPDMA_DESC_OWNER | CPDMA_DESC_SOP |
		       CPDMA_DESC_EOP | CPDMA_DESC_TO_PORT_EN |
		       CPDMA_ACTIVE_PORT;
	wmb();

	/* Start transmission */
	writel(sw_desc->desc_dma, &gemac->dma.stream->tx_hdp[0]);

	/* Wait until stop transmitting */
	while ((readl(&gemac->dma.stream->tx_hdp[0]) != 0) && --timeout_cls) {
		;
	}
	if (!timeout_cls)
		pr_err("-->%s: xmit descriptor timeout\n", __FUNCTION__);

	/* Notify GEMAC about last sending descriptor to clear interrupt */
	writel(sw_desc->desc_dma, &gemac->dma.stream->tx_cp[0]);

	DBG("TX(%d)   circles: %d   d0:d1:d2:d3 = %08x:%08x:%08x:%08x\n",
		tx_ring->cur, circles, hw_desc->next, hw_desc->buf,
		hw_desc->len, hw_desc->opt);

	/* Print if debug enabled */
	bb_print_buf(skb->data, skb->len);

	gemac->ndev->stats.tx_packets++;
	gemac->ndev->stats.tx_bytes += skb->len;

	tx_ring->cur = NEXT_DESC(tx_ring->cur);

//	DBG("<--%s\n", __FUNCTION__);

	return NETDEV_TX_OK;
}

static int process_rx_ring(struct ring *rx_ring, int budget)
{
	struct gemac_private *gemac = container_of(rx_ring,
						   struct gemac_private,
						   rx_ring);
	struct hw_desc *hw_desc;
	struct dma_desc *desc = rx_ring->desc_ring + rx_ring->cur;
	struct sk_buff *skb;
	u32 *buf;
	int processed = 0;

	DBG("-->%s: cur: %d dirty: %d\n", __FUNCTION__, rx_ring->cur,
					 rx_ring->dirty);

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

	DBG("-->%s\n", __FUNCTION__);

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

	DBG("<--%s\n", __FUNCTION__);

	return refilled;
}

int poll_rx(struct napi_struct *rx_napi, int weight)
{
	struct gemac_private *gemac = netdev_priv(rx_napi->dev);
	struct ring *rx_ring = container_of(rx_napi, struct ring, napi);
	int active_channels_mask = readl(&gemac->eth_wr.regs->c0_rx_stat);
	int processed = 0;
	int i;

	DBG("%-->s\n", __FUNCTION__);

	for (i = 0; i < CPDMA_MAX_CHAN; ++i) {
		//TODO: implement rx_ring as main pointer for the structure
		rx_ring = rx_ring + i;
		/* If channel has incoming packets */
		if ((active_channels_mask >> i) & 0x1)
			processed = process_rx_ring(rx_ring, weight);

		/* It's enough for this napi */
		if (likely(processed >= weight))
			break;
	}

	if ((processed < weight) && napi_complete_done(rx_napi, processed)) {
		DBG("%-->s: stop RX NAPI\n", __FUNCTION__);
		enable_irq(gemac->dt_irq_rx);
	}

	refill_rx(gemac);

	DBG("<--%s\n", __FUNCTION__);

	return processed;
}

int poll_tx(struct napi_struct *tx_napi, int weight)
{
	struct gemac_private *gemac = netdev_priv(tx_napi->dev);
	int refilled;

	DBG("%-->s\n", __FUNCTION__);

	refilled = refill_tx(gemac);
	if ((refilled < weight) && napi_complete_done(tx_napi, refilled)) {
		DBG("%s: stop TX NAPI\n", __FUNCTION__);
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

	disable_irq_nosync(gemac->dt_irq_rx);
	napi_schedule_irqoff(&gemac->rx_ring.napi);

	DBG("<--%s\n", __FUNCTION__);

	return IRQ_HANDLED;
}

/**
 *
 */
irqreturn_t tx_interrupt(int irq, void *dev_id)
{
	struct gemac_private *gemac = dev_id;

	DBG("-->%s \n", __FUNCTION__);

	disable_irq_nosync(gemac->dt_irq_tx);
	napi_schedule_irqoff(&gemac->tx_ring.napi);

	DBG("<--%s\n", __FUNCTION__);

	return IRQ_HANDLED;
}

/**
 *
 */
void bb_disable_interrupts(struct gemac_private *gemac, int channels_mask)
{
	u32 reg;

	/* Disable pacing interrupts */
	reg = readl(&gemac->eth_wr.regs->c0_tx_en);
	reg &= ~(channels_mask & BB_INT_CHAN_MASK);
	writel(reg, &gemac->eth_wr.regs->c0_tx_en);

	reg = readl(&gemac->eth_wr.regs->c0_rx_en);
	reg &= ~(channels_mask & BB_INT_CHAN_MASK);
	writel(channels_mask & BB_INT_CHAN_MASK, &gemac->eth_wr.regs->c0_rx_en);

	/* mask interrupts channels */
	reg = readl(&gemac->dma.regs->tx_intmask_clear);
	reg |= channels_mask & BB_INT_CHAN_MASK;
	writel(reg, &gemac->dma.regs->tx_intmask_clear);

	reg = readl(&gemac->dma.regs->rx_intmask_clear);
	reg |= channels_mask & BB_INT_CHAN_MASK;
	writel(reg, &gemac->dma.regs->rx_intmask_clear);

	disable_irq(gemac->dt_irq_tx);
	disable_irq(gemac->dt_irq_rx);
}

/**
 *
 */
void bb_enable_interrupts(struct gemac_private *gemac, int channels_mask)
{
	u32 reg;

	/* Enable pacing interrupts */
	reg = readl(&gemac->eth_wr.regs->c0_tx_en);
	reg |= channels_mask & BB_INT_CHAN_MASK;
	writel(reg, &gemac->eth_wr.regs->c0_tx_en);

	reg = readl(&gemac->eth_wr.regs->c0_rx_en);
	reg |= channels_mask & BB_INT_CHAN_MASK;
	writel(channels_mask & BB_INT_CHAN_MASK, &gemac->eth_wr.regs->c0_rx_en);

	/* Unmask interrupts channels */
	reg = readl(&gemac->dma.regs->tx_intmask_set);
	reg |= channels_mask & BB_INT_CHAN_MASK;
	writel(reg, &gemac->dma.regs->tx_intmask_set);

	reg = readl(&gemac->dma.regs->rx_intmask_set);
	reg |= channels_mask & BB_INT_CHAN_MASK;
	writel(reg, &gemac->dma.regs->rx_intmask_set);

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
	for (i = 0; i < i_err; ++i)
		dma_unmap_single(&gemac->pdev->dev, desc[i].buf_dma,
				 gemac->buf_size, DMA_FROM_DEVICE);

	/* Prepare full ring free for next step */
	i_err = gemac->ring_size;

err_alloc_buf:
	for (i = 0; i < i_err; ++i)
		kfree(desc[i].buf_cpu);

	/* Prepare full ring free for next step */
	i_err = gemac->ring_size;

err_alloc_coherent:
	for (i = 0; i < i_err; ++i)
		dma_free_coherent(&gemac->pdev->dev, sizeof(struct hw_desc),
				  desc[i].buf_cpu, desc[i].desc_dma);

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
