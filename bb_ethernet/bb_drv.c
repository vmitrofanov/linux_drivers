#include <linux/netdevice.h>

#include "bb_drv.h"
#include "bb_eth.h"

static void bb_print_buf(void *buf, int len)
{
	char print_buf[256];
	int i;
	size_t read_bytes;

	if (!len || !buf || (len > 1500))
		return;

	pr_err(">> buf_len: %d \n", len);
	for (i = 0; i < len; i += 32) {
		unsigned int print_len = min(len - i, 32);

		hex_dump_to_buffer(buf + i, print_len, 32, 1, print_buf, 256, false);
		pr_err("%s \n", print_buf);
	}
}

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
	u32 cur = tx_ring->cur;
	int out = 200;
	int circles = 0;
	static int it = 0;


//	DBG("-->%s\n", __FUNCTION__);
	sw_desc = &tx_ring->desc_ring[cur];

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
	pr_err("TX(%d)   circles: %d   d0:d1:d2:d3 = %08x:%08x:%08x:%08x\n", cur,
	       circles, hw_desc->next, hw_desc->buf, hw_desc->len, hw_desc->opt);

//	bb_print_buf(skb->data, skb->len);

	tx_ring->cur = NEXT_DESC(cur);

//	DBG("<--%s\n", __FUNCTION__);

	return NETDEV_TX_OK;
}

static int process_rx_ring(struct ring *rx_ring, int budget)
{
	struct gemac_private *gemac = container_of(rx_ring, struct gemac_private,
						   rx_ring);
	int cur = rx_ring->cur;
	struct hw_desc *hw_desc;
	struct dma_desc *desc = rx_ring->desc_ring + cur;
	struct sk_buff *skb;
	u32 *buf;
	int processed = 0;

//	DBG("-->%s\n", __FUNCTION__);

	while(processed < budget) {
		hw_desc = desc->desc_cpu;

		/* Is it ready or not */
		if (hw_desc->opt & CPDMA_DESC_OWNER)
			break;

		if (hw_desc->opt & CPDMA_DESC_EOP)
			desc->buf_size = hw_desc->len & CPDMA_PKT_MAX_LEN;

		desc->buf_size -= 4;
#if 1
		dma_sync_single_for_cpu(&gemac->pdev->dev, desc->buf_dma,
					desc->buf_size,
					DMA_FROM_DEVICE);
#else
		dma_unmap_single(&gemac->ndev->dev, desc->buf_dma,
				 desc->buf_size, DMA_FROM_DEVICE);
#endif

#if 1
		buf = desc->buf_cpu;
		skb = netdev_alloc_skb(gemac->ndev,
				       MTU_TO_BUF_SIZE(gemac->ndev->mtu));
		if (!skb)
			break;
#else
		skb = desc->skb;
		desc->skb = NULL;
#endif
		skb_reserve(skb, NET_IP_ALIGN);
		skb_copy_to_linear_data(skb, buf, desc->buf_size);
		skb_put(skb, desc->buf_size);
		skb->protocol = eth_type_trans(skb, gemac->ndev);
		gemac->ndev->stats.rx_packets++;
		gemac->ndev->stats.rx_bytes += desc->buf_size;

//		skb_checksum_none_assert(skb);
//		skb->dev = gemac->ndev;
//		skb->ip_summed = CHECKSUM_UNNECESSARY;

		if (NET_RX_SUCCESS != netif_receive_skb(skb))
			pr_err("packet dropped          !!!!!!!!!!!!!!!!!!!!!!\n");
		else
			pr_err("packet sucess           ++++++++++++++++++++++\n");

		pr_err("RX(%d): d0:d1:d2:d3 = %08x:%08x:%08x:%08x\n",
		       rx_ring->cur, hw_desc->next, hw_desc->buf, hw_desc->len,
		       hw_desc->opt);
//		bb_print_buf(buf/*skb->data*/, desc->buf_size);

		/* Try to clear interrupt */
		writel(desc->desc_dma, &gemac->dma.stream->rx_cp[0]);

		++processed;
		rx_ring->cur = NEXT_DESC(cur);
		desc = rx_ring->desc_ring + rx_ring->cur;
	}

//	DBG("<--%s\n", __FUNCTION__);

	return processed;
}

int poll_rx(struct napi_struct *rx_napi, int weight)
{
	struct gemac_private *gemac = netdev_priv(rx_napi->dev);
	struct ring *rx_ring = container_of(rx_napi, struct ring, napi);
	int processed;

//	DBG("-->%s\n", __FUNCTION__);

	processed = process_rx_ring(rx_ring, weight);
	if ((processed < weight) && napi_complete_done(rx_napi, processed)) {
		writel(0xFF, &gemac->eth_wr.regs->rx_en);
		enable_irq(gemac->dt_irq_rx);
		pr_err("enable interrupts\n");
	}

//	DBG("<--%s\n", __FUNCTION__);

	return 0;
}

int poll_tx(struct napi_struct *napi, int weight)
{
	DBG("<--%s\n", __FUNCTION__);

	return 0;
}

irqreturn_t rx_interrupt(int irq, void *dev_id)
{
	struct gemac_private *gemac = dev_id;

//	DBG("-->%s \n", __FUNCTION__);

	writel(CPDMA_EOI_RX, &gemac->dma.regs->cpdma_eoi_vector);
	writel(0, &gemac->eth_wr.regs->rx_en);

	disable_irq_nosync(gemac->dt_irq_rx);
	napi_schedule_irqoff(&gemac->rx_ring.napi);

//	DBG("<--%s\n", __FUNCTION__);

	return IRQ_HANDLED;
}


void bb_enable_interrupts(struct gemac_private *gemac)
{
//	writel(0xFF, &gemac->eth_wr.regs->tx_en);
	writel(0xFF, &gemac->eth_wr.regs->rx_en);

	//TODO: now enable all interrupts
//	__raw_writel(0xFF, &gemac->dma.regs->tx_intmask_set);
	__raw_writel(0xFF, &gemac->dma.regs->rx_intmask_set);

	enable_irq(gemac->dt_irq_rx);
}

int bb_alloc_ring(struct gemac_private *gemac, struct ring *ring, int alloc_buffers)
{
	struct ring *r = ring;
	struct dma_desc *desc;
	int i;
	int i_err;
	int len;
	struct sk_buff *skb;

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
#if 1
		desc[i].buf_size = gemac->buf_size;
		desc[i].buf_cpu = kmalloc(desc[i].buf_size, GFP_KERNEL);
		if (!desc[i].buf_cpu) {
			pr_err("Error, can't allocate buffer\n");
			i_err = i;
			goto err_alloc_buf;
		}
#else
//		skb = netdev_alloc_skb(gemac->ndev, MTU_TO_BUF_SIZE(gemac->ndev->max_mtu));
		skb = netdev_alloc_skb_ip_align(gemac->ndev, MTU_TO_BUF_SIZE(gemac->ndev->max_mtu));
		if (unlikely(!skb))
			goto err_alloc_buf;

		len = skb_tailroom(skb);

		desc[i].buf_size = len;
		desc[i].skb = skb;
#endif
	}

	/* Map buffer memory to use with DMA */
	for (i = 0; i < gemac->ring_size; ++i) {
#if 1
		desc[i].buf_dma = dma_map_single(&gemac->pdev->dev,
						 desc[i].buf_cpu,
						 gemac->buf_size,
						 DMA_FROM_DEVICE);

		dma_sync_single_for_device(&gemac->pdev->dev,
					   desc[i].buf_dma,
					   desc[i].buf_size,
					   DMA_FROM_DEVICE);

		if (dma_mapping_error(&gemac->pdev->dev, desc[i].buf_dma)) {
			pr_err("Error, can't map buffer\n");
			i_err = i;
			goto err_map_buf;
		}
#else
		desc[i].buf_cpu = skb_tail_pointer(desc[i].skb);
		desc[i].buf_dma = dma_map_single(&gemac->ndev->dev,
						 skb_tail_pointer(desc[i].skb),
						 desc[i].buf_size,
						 DMA_FROM_DEVICE);

		if (dma_mapping_error(&gemac->ndev->dev, desc[i].buf_dma)) {
			dev_kfree_skb_any(desc[i].skb);
			netdev_warn(gemac->ndev, "dma_map_single failed!\n");
			goto err_map_buf;
		}
#endif
	}

	DBG("<--%s\n", __FUNCTION__);

	return 0;

err_map_buf:
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
	int i_err;

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
		hw_desc->opt = CPDMA_DESC_OWNER;// | CPDMA_DESC_SOP | CPDMA_DESC_EOP; //SOP EOP ??? Not necessary and set by gemac
	}

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


