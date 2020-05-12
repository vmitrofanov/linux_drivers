#include <linux/netdevice.h>

#include "bb_drv.h"
#include "bb_eth.h"

static void print_rx_ring(struct gemac_private *gemac)
{
	struct ring *ring = &gemac->rx_ring;
	struct hw_desc *desc;
	int i;

	for (i = 0; i < 10; ++i) {
		desc = ring->desc_ring[i].desc_cpu;
		pr_err("ring: %d    [d1: %08x, d2: %08x, d3: %08x, d4: %08x]\n",
				i,
				readl((__iomem void *)&desc->next),
				readl((__iomem void *)&desc->buf),
				readl((__iomem void *)&desc->len),
				readl((__iomem void *)&desc->opt));
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
	int out = 10;

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
	writel(CPDMA_DESC_OWNER | CPDMA_DESC_SOP | CPDMA_DESC_EOP, (__iomem void*)&hw_desc->opt);
//	hw_desc->opt = CPDMA_DESC_OWNER | CPDMA_DESC_SOP | CPDMA_DESC_EOP;

	/* Start transmission */
	writel(sw_desc->desc_dma, &gemac->dma.stream->tx_hdp[0]);

	//TODO: delete after debug
	while ((readl(&gemac->dma.stream->tx_hdp[0]) != 0) && out--) {
		pr_err("index: %d  [d1:%08x  d2:%08x  d3:%08x  d4:%08x]\n", cur,
				readl((__iomem void*)&hw_desc->next),
				readl((__iomem void*)&hw_desc->buf),
				readl((__iomem void*)&hw_desc->len),
				readl((__iomem void*)&hw_desc->opt));
	}

	tx_ring->cur = NEXT_DESC(cur);

//	DBG("<--%s\n", __FUNCTION__);

	return NETDEV_TX_OK;
}


int poll_rx(struct napi_struct *napi, int weight)
{
	DBG("-->%s\n", __FUNCTION__);
	DBG("<--%s\n", __FUNCTION__);

	return 0;
}

int poll_tx(struct napi_struct *napi, int weight)
{
	DBG("<--%s\n", __FUNCTION__);

	return 0;
}

irqreturn_t rx_interrupt(int irq, void *dev_id)
{
	DBG("-->%s \n", __FUNCTION__);
//	struct cpsw_common *cpsw = dev_id;
//
//	cpdma_ctlr_eoi(cpsw->dma, CPDMA_EOI_RX);
//	writel(0, &cpsw->wr_regs->rx_en);
//
//	if (cpsw->quirk_irq) {
//		disable_irq_nosync(cpsw->irqs_table[0]);
//		cpsw->rx_irq_disabled = true;
//	}
//
//	napi_schedule(&cpsw->napi_rx);
	DBG("<--%s\n", __FUNCTION__);

	return IRQ_HANDLED;
}


void bb_enable_interrupts(struct gemac_private *gemac)
{
	writel(0xFF, &gemac->eth_wr.regs->tx_en);
	writel(0xFF, &gemac->eth_wr.regs->rx_en);

	//TODO: now enable all interrupts
	__raw_writel(0xFF, &gemac->dma.regs->rx_intmask_set);


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

//	/* Move DMA to idle state */
//	writel(BIT(3), &gemac->dma.regs->dma_control);

	/* Reset DMA */
	writel(1, &gemac->dma.regs->cpdma_soft_reset);
	while (readl(&gemac->dma.regs->cpdma_soft_reset) && --timeout_cnt)
		msleep(1);

	if (readl(&gemac->dma.regs->cpdma_soft_reset))
		return -1;

//	/* Move DMA from idle state */
//	writel(0, &gemac->dma.regs->dma_control);

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

		hw_desc->next = 0;
		hw_desc->buf = desc[i].buf_dma;
		hw_desc->len = gemac->buf_size;
		hw_desc->opt = CPDMA_DESC_OWNER;// | CPDMA_DESC_SOP | CPDMA_DESC_EOP; //SOP EOP ??? Not necessary and set by gemac
	}


	//??
	writel(desc[0].desc_dma, &gemac->dma.stream->rx_hdp[0]);


	DBG("<--%s\n", __FUNCTION__);

	return 0;
}

void bb_start_dma_engine(struct gemac_private *gemac)
{
	print_rx_ring(gemac);

	__raw_writel(1, &gemac->dma.regs->rx_control);
	__raw_writel(1, &gemac->dma.regs->tx_control);

	pr_err("rx_control: %08x\n", readl(&gemac->dma.regs->rx_control));
	pr_err("tx_control: %08x\n", readl(&gemac->dma.regs->tx_control));
}


