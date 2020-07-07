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
	u32 queue = skb_get_queue_mapping(skb);
	struct ring *tx_ring = &gemac->tx_ring[queue];
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
	writel(sw_desc->desc_dma, &gemac->dma.stream->tx_hdp[queue]);

	/* Wait until stop transmitting */
	while ((readl(&gemac->dma.stream->tx_hdp[queue]) != 0) && --timeout_cls)
		continue;

	if (!timeout_cls)
		pr_err("-->%s: xmit descriptor timeout\n", __FUNCTION__);

	/* Notify GEMAC about last sending descriptor to clear interrupt */
	writel(sw_desc->desc_dma, &gemac->dma.stream->tx_cp[queue]);

	/* Print if debug enabled */
	bb_print_buf(skb->data, skb->len);

	gemac->ndev->stats.tx_packets++;
	gemac->ndev->stats.tx_bytes += skb->len;

	tx_ring->cur = NEXT_DESC(tx_ring->cur);

	DBG("<--%s\n", __FUNCTION__);

	return NETDEV_TX_OK;
}

static int process_rx_ring(struct ring *rx_ring, int budget)
{
	struct gemac_private *gemac = rx_ring->gemac;
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

		if (NET_RX_SUCCESS != netif_receive_skb(skb))
			DBG("packet dropped by stack\n");

		bb_print_buf(buf, desc->buf_size);

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

static int refill_rx(struct gemac_private *gemac, struct ring *rx_ring)
{
	struct ring *rx = rx_ring;
	struct dma_desc *sw_desc;
	struct hw_desc *hw_desc;
	int dirty_num = get_dirty_num(gemac, rx);
	int refilled = 0;
	u32 opt = 0;

	while (dirty_num--) {
		++refilled;
		sw_desc = rx->desc_ring + rx->dirty;
		hw_desc = (struct hw_desc *)sw_desc->desc_cpu;

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

static int refill_tx(struct gemac_private *gemac, struct ring *tx_ring)
{
	struct ring *tx = tx_ring;
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
	int processed = 0;

	DBG("%-->s\n", __FUNCTION__);

	processed = process_rx_ring(rx_ring, weight);

	if ((processed < weight) && napi_complete_done(rx_napi, processed)) {
		DBG("%-->s: stop RX NAPI\n", __FUNCTION__);
		enable_irq(gemac->dt_irq_rx);
	}

	refill_rx(gemac, rx_ring);

	DBG("<--%s\n", __FUNCTION__);

	return processed;
}

int poll_tx(struct napi_struct *tx_napi, int weight)
{
	struct gemac_private *gemac = netdev_priv(tx_napi->dev);
	struct ring *tx_ring = container_of(tx_napi, struct ring, napi);
	int refilled;

	DBG("%-->s\n", __FUNCTION__);

	refilled = refill_tx(gemac, tx_ring);
	if ((refilled < weight) && napi_complete_done(tx_napi, refilled)) {
		DBG("%s: stop TX NAPI\n", __FUNCTION__);
		enable_irq(gemac->dt_irq_tx);
	}

	DBG("<--%s\n", __FUNCTION__);

	return refilled;
}

irqreturn_t rx_interrupt(int irq, void *dev_id)
{
	struct gemac_private *gemac = dev_id;
	int active_channels_mask = readl(&gemac->eth_wr.regs->c0_rx_stat);
	int i;

	DBG("%-->s\n", __FUNCTION__);

	disable_irq_nosync(gemac->dt_irq_rx);

	for (i = 0; i < CPDMA_MAX_CHAN; ++i)
		if ((active_channels_mask >> i) & 0x1)
			napi_schedule_irqoff(&gemac->rx_ring[i].napi);

	DBG("<--%s\n", __FUNCTION__);

	return IRQ_HANDLED;
}

/**
 *
 */
irqreturn_t tx_interrupt(int irq, void *dev_id)
{
	struct gemac_private *gemac = dev_id;
	int active_channels_mask = readl(&gemac->eth_wr.regs->c0_tx_stat);
	int i;

	DBG("-->%s \n", __FUNCTION__);

	disable_irq_nosync(gemac->dt_irq_tx);

	for (i = 0; i < CPDMA_MAX_CHAN; ++i)
		if ((active_channels_mask >> i) & 0x1)
			napi_schedule_irqoff(&gemac->tx_ring[i].napi);

	DBG("<--%s\n", __FUNCTION__);

	return IRQ_HANDLED;
}

/**
 *
 */
void bb_disable_interrupts(struct gemac_private *gemac, int chan_num)
{
	u32 reg;
	u32 channel_mask = 0;
	int i;

	for (i = 0; i < chan_num; ++i)
		channel_mask |= 1UL << i;

	/* Disable pacing interrupts */
	reg = readl(&gemac->eth_wr.regs->c0_tx_en);
	reg &= ~(channel_mask & BB_INT_CHAN_MASK);
	writel(reg, &gemac->eth_wr.regs->c0_tx_en);

	reg = readl(&gemac->eth_wr.regs->c0_rx_en);
	reg &= ~(channel_mask & BB_INT_CHAN_MASK);
	writel(channel_mask & BB_INT_CHAN_MASK, &gemac->eth_wr.regs->c0_rx_en);

	/* mask interrupts channels */
	reg = readl(&gemac->dma.regs->tx_intmask_clear);
	reg |= channel_mask & BB_INT_CHAN_MASK;
	writel(reg, &gemac->dma.regs->tx_intmask_clear);

	reg = readl(&gemac->dma.regs->rx_intmask_clear);
	reg |= channel_mask & BB_INT_CHAN_MASK;
	writel(reg, &gemac->dma.regs->rx_intmask_clear);

	disable_irq(gemac->dt_irq_tx);
	disable_irq(gemac->dt_irq_rx);
}

/**
 * NOTE: nested function!
 */
void bb_enable_interrupts(struct gemac_private *gemac, int chan_num)
{
	u32 reg;
	u32 channel_mask = 0;
	int i;

	for (i = 0; i < chan_num; ++i)
		channel_mask |= 1UL << i;

	/* Enable pacing interrupts */
	reg = readl(&gemac->eth_wr.regs->c0_tx_en);
	reg |= channel_mask & BB_INT_CHAN_MASK;
	writel(reg, &gemac->eth_wr.regs->c0_tx_en);

	reg = readl(&gemac->eth_wr.regs->c0_rx_en);
	reg |= channel_mask & BB_INT_CHAN_MASK;
	writel(channel_mask & BB_INT_CHAN_MASK, &gemac->eth_wr.regs->c0_rx_en);

	/* Unmask interrupts channels */
	reg = readl(&gemac->dma.regs->tx_intmask_set);
	reg |= channel_mask & BB_INT_CHAN_MASK;
	writel(reg, &gemac->dma.regs->tx_intmask_set);

	reg = readl(&gemac->dma.regs->rx_intmask_set);
	reg |= channel_mask & BB_INT_CHAN_MASK;
	writel(reg, &gemac->dma.regs->rx_intmask_set);

	/* User, pay attention nested function */
	enable_irq(gemac->dt_irq_tx);
	enable_irq(gemac->dt_irq_rx);
}

void bb_free_ring(struct gemac_private *gemac, struct ring *ring, bool is_rx)
{
	int i;

	if (is_rx) {
		for (i = 0; i < gemac->ring_size; ++i) {
			/* Release Rx ring resources */
			dma_unmap_single(&gemac->pdev->dev,
					 ring->desc_ring[i].buf_dma,
					 ring->desc_ring[i].buf_size,
					 DMA_TO_DEVICE);

			kfree(ring->desc_ring[i].buf_cpu);

			dma_free_coherent(&gemac->pdev->dev,
					  sizeof(struct hw_desc),
					  ring->desc_ring[i].desc_cpu,
					  ring->desc_ring[i].desc_dma);
		}
	} else {
		for (i = 0; i < gemac->ring_size; ++i) {
			/* Release Tx ring resources */
			dma_free_coherent(&gemac->pdev->dev,
					  sizeof(struct hw_desc),
					  ring->desc_ring[i].desc_cpu,
					  ring->desc_ring[i].desc_dma);
		}
	}
}

int bb_alloc_ring(struct gemac_private *gemac, struct ring *ring, bool is_rx)
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

	if (!is_rx)
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

int bb_gemac_reset(struct gemac_private *gemac)
{
	int timeout_cnt = 50;

	writel(1, &gemac->dma.regs->cpdma_soft_reset);
	while (readl(&gemac->dma.regs->cpdma_soft_reset) && --timeout_cnt)
		msleep(1);

	if (readl(&gemac->dma.regs->cpdma_soft_reset))
		return -1;

	return 0;
}

int bb_init_tx_rx_rings(struct gemac_private *gemac, int channel)
{
	struct ring *ring;
	struct dma_desc *desc;
	struct hw_desc *hw_desc;
	int i;

	DBG("-->%s\n", __FUNCTION__);

	/* Zero ring head pointer */
	writel(0, &gemac->dma.stream->rx_hdp[channel]);
	writel(0, &gemac->dma.stream->tx_hdp[channel]);

	/* Now only for Rx ring */
	ring = &gemac->rx_ring[channel];
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
	writel(desc[0].desc_dma, &gemac->dma.stream->rx_hdp[channel]);

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
