#include "bb_ale.h"

/**
 * cpsw_ale_write_entry() - write ethernet address entry to ALE table
 * @gemac: common structure
 * @idx: index in an ALE table
 * @ale_entry: pointer to an entry containing information we would like to write
 */
static void cpsw_ale_write_entry(struct gemac_private *gemac, uint16_t idx,
				 uint32_t *ale_entry)
{
	writel(ale_entry[0], &gemac->ale.regs->tblw0);
	writel(ale_entry[1], &gemac->ale.regs->tblw1);
	writel(ale_entry[2], &gemac->ale.regs->tblw2);
	writel(1 << 31 | (idx & 1023), &gemac->ale.regs->tblctl);
}

/**
 * bb_ale_init() - initialize Address Lookup Engine
 * @gemac: common structure
 * Description: Turn on ALE, allow broadcast frames
 */
void bb_ale_init(struct gemac_private *gemac)
{
	int i;
	u32 reg;
	uint32_t ale_entry[3];

	/* Enable ALE */
	reg = readl(&gemac->ale.regs->control);
	reg |= BIT(31);
	reg |= BIT(30);
	reg |= BIT(4);
	__raw_writel(reg, &gemac->ale.regs->control);

	reg = readl(&gemac->ale.regs->control);
	DBG("REG: ALE->CTRL: %08x\n", reg);

	/* Enable ALE forwarding on all ports */
	for (i = 0; i < 6; ++i)
		writel(3, &gemac->ale.regs->portctl[i]);

	/* Setup link address at table entry 0 */
	ale_entry[0] = gemac->dt_mac[2] << 24 | gemac->dt_mac[3] << 16 |
		       gemac->dt_mac[4] << 8 | gemac->dt_mac[5];
	ale_entry[1] = 0x10 << 24 | gemac->dt_mac[0] << 8 | gemac->dt_mac[1];
	ale_entry[2] = 0;
	cpsw_ale_write_entry(gemac, 0, ale_entry);

	/* Keep the broadcast address at table entry 1. */
	/* Lower 32 bits of MAC */
	ale_entry[0] = 0xffffffff;
	/* FW (3 << 30), Addr entry (1 << 24), upper 16 bits of Mac */
	ale_entry[1] = 0xd000ffff;
	/* Forward to all ports */
	ale_entry[2] = 0x0000001c;

	cpsw_ale_write_entry(gemac, 1, ale_entry);

}
