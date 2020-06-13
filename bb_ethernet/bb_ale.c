#include "bb_ale.h"

const u8 net_bcast_ethaddr[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

static inline int cpsw_ale_get_field(u32 *ale_entry, u32 start, u32 bits)
{
        int idx;

        idx    = start / 32;
        start -= idx * 32;
        idx    = 2 - idx; /* flip */
        return (ale_entry[idx] >> start) & BITMASK(bits);
}

static inline void cpsw_ale_set_field(u32 *ale_entry, u32 start, u32 bits,
				      u32 value)
{
	int idx;

	value &= BITMASK(bits);
	idx    = start / 32;
	start -= idx * 32;
	idx    = 2 - idx; /* flip */
	ale_entry[idx] &= ~(BITMASK(bits) << start);
	ale_entry[idx] |=  (value << start);
}

/* The MAC address field in the ALE entry cannot be macroized as above */
static inline void cpsw_ale_get_addr(u32 *ale_entry, u8 *addr)
{
        int i;

        for (i = 0; i < 6; i++)
                addr[i] = cpsw_ale_get_field(ale_entry, 40 - 8*i, 8);
}

static inline void cpsw_ale_set_addr(u32 *ale_entry, u8 *addr)
{
        int i;

        for (i = 0; i < 6; i++)
                cpsw_ale_set_field(ale_entry, 40 - 8*i, 8, addr[i]);
}

#define DEFINE_ALE_FIELD(name, start, bits)				\
static inline int cpsw_ale_get_##name(u32 *ale_entry)			\
{									\
	return cpsw_ale_get_field(ale_entry, start, bits);		\
}									\
static inline void cpsw_ale_set_##name(u32 *ale_entry, u32 value)	\
{									\
	cpsw_ale_set_field(ale_entry, start, bits, value);		\
}

#define DEFINE_ALE_FIELD1(name, start)					\
static inline int cpsw_ale_get_##name(u32 *ale_entry, u32 bits)		\
{									\
	return cpsw_ale_get_field(ale_entry, start, bits);		\
}									\
static inline void cpsw_ale_set_##name(u32 *ale_entry, u32 value,	\
		u32 bits)						\
{									\
	cpsw_ale_set_field(ale_entry, start, bits, value);		\
}

DEFINE_ALE_FIELD(entry_type,            60,     2)
DEFINE_ALE_FIELD(vlan_id,               48,     12)
DEFINE_ALE_FIELD(mcast_state,           62,     2)
DEFINE_ALE_FIELD1(port_mask,            66)
DEFINE_ALE_FIELD(super,                 65,     1)
DEFINE_ALE_FIELD(ucast_type,            62,     2)
DEFINE_ALE_FIELD1(port_num,             66)
DEFINE_ALE_FIELD(blocked,               65,     1)
DEFINE_ALE_FIELD(secure,                64,     1)
DEFINE_ALE_FIELD1(vlan_untag_force,     24)
DEFINE_ALE_FIELD1(vlan_reg_mcast,       16)
DEFINE_ALE_FIELD1(vlan_unreg_mcast,     8)
DEFINE_ALE_FIELD1(vlan_member_list,     0)
DEFINE_ALE_FIELD(mcast,                 40,     1)
/* ALE NetCP nu switch specific */
DEFINE_ALE_FIELD(vlan_unreg_mcast_idx,  20,     3)
DEFINE_ALE_FIELD(vlan_reg_mcast_idx,    44,     3)


static int cpsw_ale_write(struct gemac_private *gemac, int idx, u32 *ale_entry)
{
        int i;

//        WARN_ON(idx > ale->params.ale_entries);

        for (i = 0; i < ALE_ENTRY_WORDS; i++)
                writel_relaxed(ale_entry[i], (void *)gemac->ale.regs + //ale->params.ale_regs +
                               ALE_TABLE + 4 * i);

        writel_relaxed(idx | ALE_TABLE_WRITE, (void *)gemac->ale.regs + //ale->params.ale_regs +
                       ALE_TABLE_CONTROL);

        return idx;
}

static int cpsw_ale_read(struct gemac_private *gemac, int idx, u32 *ale_entry)
{
        int i;

//        WARN_ON(idx > ale->params.ale_entries);

        writel_relaxed(idx, (void *)gemac->ale.regs + ALE_TABLE_CONTROL);

        for (i = 0; i < ALE_ENTRY_WORDS; i++)
                ale_entry[i] = readl_relaxed((void *)gemac->ale.regs + //ale->params.ale_regs +
                                             ALE_TABLE + 4 * i);

        return idx;
}



static int cpsw_ale_match_addr(struct gemac_private *priv, const u8 *addr)
{
        u32 ale_entry[ALE_ENTRY_WORDS];
        int type, idx;

        for (idx = 0; idx < 1024/*priv->data->ale_entries*/; idx++) {
                u8 entry_addr[6];

                cpsw_ale_read(priv, idx, ale_entry);
                type = cpsw_ale_get_entry_type(ale_entry);
                if (type != ALE_TYPE_ADDR && type != ALE_TYPE_VLAN_ADDR)
                        continue;
                cpsw_ale_get_addr(ale_entry, entry_addr);
                if (memcmp(entry_addr, addr, 6) == 0)
                        return idx;
        }
        return -ENOENT;
}

static int cpsw_ale_match_free(struct gemac_private *gemac)
{
        u32 ale_entry[ALE_ENTRY_WORDS];
        int type, idx;

        for (idx = 0; idx < 1024/*ale->params.ale_entries*/; idx++) {
                cpsw_ale_read(gemac, idx, ale_entry);
                type = cpsw_ale_get_entry_type(ale_entry);
                if (type == ALE_TYPE_FREE)
                        return idx;
        }
        return -ENOENT;
}

static int cpsw_ale_find_ageable(struct gemac_private *gemac)
{
        u32 ale_entry[ALE_ENTRY_WORDS];
        int type, idx;

        for (idx = 0; idx < 1024/*ale->params.ale_entries*/; idx++) {
                cpsw_ale_read(gemac, idx, ale_entry);
                type = cpsw_ale_get_entry_type(ale_entry);
                if (type != ALE_TYPE_ADDR && type != ALE_TYPE_VLAN_ADDR)
                        continue;
                if (cpsw_ale_get_mcast(ale_entry))
                        continue;
                type = cpsw_ale_get_ucast_type(ale_entry);
                if (type != ALE_UCAST_PERSISTANT &&
                    type != ALE_UCAST_OUI)
                        return idx;
        }
        return -ENOENT;
}


int bb_ale_add_mcast(struct gemac_private *gemac, const u8 *addr,
                              int port_mask)
{
        u32 ale_entry[ALE_ENTRY_WORDS] = {0, 0, 0};
        int idx, mask;

        idx = cpsw_ale_match_addr(gemac, addr);
        if (idx >= 0)
                cpsw_ale_read(gemac, idx, ale_entry);
        DBG("ALE INDEX: %d\n", idx);

        cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_ADDR);
        cpsw_ale_set_addr(ale_entry, addr);
        cpsw_ale_set_mcast_state(ale_entry, ALE_MCAST_FWD_2);

        mask = cpsw_ale_get_port_mask(ale_entry, 0xFFFFFFFF);
        port_mask |= mask;
        cpsw_ale_set_port_mask(ale_entry, port_mask, 0xFFFFFFFF);

        if (idx < 0)
                idx = cpsw_ale_match_free(gemac);
        if (idx < 0)
                idx = cpsw_ale_find_ageable(gemac);
        if (idx < 0)
                return -ENOMEM;

        cpsw_ale_write(gemac, idx, ale_entry);
        return 0;
}

static int cpsw_ale_add_ucast(struct gemac_private *priv, const u8 *addr,
                              int port, int flags)
{
        u32 ale_entry[ALE_ENTRY_WORDS] = {0, 0, 0};
        int idx;

        cpsw_ale_set_entry_type(ale_entry, ALE_TYPE_ADDR);
        cpsw_ale_set_addr(ale_entry, addr);
        cpsw_ale_set_ucast_type(ale_entry, ALE_UCAST_PERSISTANT);
        cpsw_ale_set_secure(ale_entry, (flags & ALE_SECURE) ? 1 : 0);
        cpsw_ale_set_blocked(ale_entry, (flags & ALE_BLOCKED) ? 1 : 0);
        cpsw_ale_set_port_num(ale_entry, port, 0xFFFFFFFF);

        idx = cpsw_ale_match_addr(priv, addr);
        if (idx < 0)
                idx = cpsw_ale_match_free(priv);
        if (idx < 0)
                idx = cpsw_ale_find_ageable(priv);
        if (idx < 0)
                return -ENOMEM;

        cpsw_ale_write(priv, idx, ale_entry);
        return 0;
}


static void
cpsw_ale_write_entry(struct gemac_private *gemac, uint16_t idx, uint32_t *ale_entry)
{
//	cpsw_write_4(sc, CPSW_ALE_TBLW0, ale_entry[0]);
//	cpsw_write_4(sc, CPSW_ALE_TBLW1, ale_entry[1]);
//	cpsw_write_4(sc, CPSW_ALE_TBLW2, ale_entry[2]);
//	cpsw_write_4(sc, CPSW_ALE_TBLCTL, 1 << 31 | (idx & 1023));

	writel(ale_entry[0], &gemac->ale.regs->tblw0);
	writel(ale_entry[1], &gemac->ale.regs->tblw1);
	writel(ale_entry[2], &gemac->ale.regs->tblw2);
	writel(1 << 31 | (idx & 1023), &gemac->ale.regs->tblctl);
}

int bb_ale_init(struct gemac_private *gemac)
{
	int result = 0;
	int i;
	u32 reg;
	uint32_t ale_entry[3];

	/* Enable ALE */
	reg = readl(&gemac->ale.regs->control);
	reg |= BIT(31);
	reg |= BIT(30);
	reg |= BIT(4);
//	reg |= BIT(1);
//	reg &= ~BIT(2);
	__raw_writel(reg, &gemac->ale.regs->control);

	reg = readl(&gemac->ale.regs->control);
	DBG("REG: ALE->CTRL: %08x\n", reg);

	/* Enable ALE forwarding on all ports */
	for (i = 0; i < 6; ++i)
		writel(3, &gemac->ale.regs->portctl[i]);

	/* Setup link address at table entry 0 */
//	mac = LLADDR((struct sockaddr_dl *)ifp->if_addr->ifa_addr);
//	ale_entry[0] = mac[2] << 24 | mac[3] << 16 | mac[4] << 8 | mac[5];
//	ale_entry[1] = 0x10 << 24 | mac[0] << 8 | mac[1]; /* addr entry + mac */
//	ale_entry[2] = 0; /* port = 0 */
//	cpsw_ale_write_entry(sc, 0, ale_entry);
//
//	/* Set outgoing MAC Address for Ports 1 and 2. */
//	for (i = 1; i < 3; ++i) {
//		cpsw_write_4(sc, CPSW_PORT_P_SA_HI(i),
//		    mac[3] << 24 | mac[2] << 16 | mac[1] << 8 | mac[0]);
//		cpsw_write_4(sc, CPSW_PORT_P_SA_LO(i),
//		    mac[5] << 8 | mac[4]);
//	}
//        if_addr_runlock(ifp);

	ale_entry[0] = gemac->dt_mac[2] << 24 | gemac->dt_mac[3] << 16 | gemac->dt_mac[4] << 8 | gemac->dt_mac[5];
	ale_entry[1] = 0x10 << 24 | gemac->dt_mac[0] << 8 | gemac->dt_mac[1]; /* addr entry + mac */
	ale_entry[2] = 0; /* port = 0 */
	cpsw_ale_write_entry(gemac, 0, ale_entry);


	/* Keep the broadcast address at table entry 1. */
	ale_entry[0] = 0xffffffff; /* Lower 32 bits of MAC */
	ale_entry[1] = 0xd000ffff; /* FW (3 << 30), Addr entry (1 << 24), upper 16 bits of Mac */
	ale_entry[2] = 0x0000001c; /* Forward to all ports */
	cpsw_ale_write_entry(gemac, 1, ale_entry);

	return result;
}
