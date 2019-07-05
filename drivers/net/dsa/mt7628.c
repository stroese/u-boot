// SPDX-License-Identifier: GPL-2.0-only
/*
 * Mediatek MT7530 DSA Switch driver
 * Copyright (C) 2017 Sean Wang <sean.wang@mediatek.com>
 */
#include <linux/etherdevice.h>
#include <linux/if_bridge.h>
#include <linux/iopoll.h>
#include <linux/mdio.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/gpio/consumer.h>
#include <net/dsa.h>
#include <linux/platform_device.h> // test-only

//#include "mt7628.h"

/* struct mt7628_priv -	This is the main data structure for holding the state
 *			of the driver
 * @dev:		The device pointer
 * @ds:			The pointer to the dsa core structure
 * @bus:		The bus used for the device and built-in PHY
 * @rstc:		The pointer to reset control used by MCM
 * @ethernet:		The regmap used for access TRGMII-based registers
 * @core_pwr:		The power supplied into the core
 * @io_pwr:		The power supplied into the I/O
 * @reset:		The descriptor for GPIO line tied to its reset pin
 * @mcm:		Flag for distinguishing if standalone IC or module
 *			coupling
 * @ports:		Holding the state among ports
 * @reg_mutex:		The lock for protecting among process accessing
 *			registers
 */
struct mt7628_priv {
	struct device		*dev;
	struct dsa_switch	*ds;
	struct mii_bus		*bus;
	struct reset_control	*rstc;
#if 0
	struct regmap		*ethernet;
	struct regulator	*core_pwr;
	struct regulator	*io_pwr;
	struct gpio_desc	*reset;
#endif
	unsigned int		id;
	bool			mcm;

//	struct mt7628_port	ports[MT7628_NUM_PORTS];
	/* protect among processes for registers access*/
	struct mutex reg_mutex;
};


#if 0
/* String, offset, and register size in bytes if different from 4 bytes */
static const struct mt7628_mib_desc mt7628_mib[] = {
	MIB_DESC(1, 0x00, "TxDrop"),
	MIB_DESC(1, 0x04, "TxCrcErr"),
	MIB_DESC(1, 0x08, "TxUnicast"),
	MIB_DESC(1, 0x0c, "TxMulticast"),
	MIB_DESC(1, 0x10, "TxBroadcast"),
	MIB_DESC(1, 0x14, "TxCollision"),
	MIB_DESC(1, 0x18, "TxSingleCollision"),
	MIB_DESC(1, 0x1c, "TxMultipleCollision"),
	MIB_DESC(1, 0x20, "TxDeferred"),
	MIB_DESC(1, 0x24, "TxLateCollision"),
	MIB_DESC(1, 0x28, "TxExcessiveCollistion"),
	MIB_DESC(1, 0x2c, "TxPause"),
	MIB_DESC(1, 0x30, "TxPktSz64"),
	MIB_DESC(1, 0x34, "TxPktSz65To127"),
	MIB_DESC(1, 0x38, "TxPktSz128To255"),
	MIB_DESC(1, 0x3c, "TxPktSz256To511"),
	MIB_DESC(1, 0x40, "TxPktSz512To1023"),
	MIB_DESC(1, 0x44, "Tx1024ToMax"),
	MIB_DESC(2, 0x48, "TxBytes"),
	MIB_DESC(1, 0x60, "RxDrop"),
	MIB_DESC(1, 0x64, "RxFiltering"),
	MIB_DESC(1, 0x6c, "RxMulticast"),
	MIB_DESC(1, 0x70, "RxBroadcast"),
	MIB_DESC(1, 0x74, "RxAlignErr"),
	MIB_DESC(1, 0x78, "RxCrcErr"),
	MIB_DESC(1, 0x7c, "RxUnderSizeErr"),
	MIB_DESC(1, 0x80, "RxFragErr"),
	MIB_DESC(1, 0x84, "RxOverSzErr"),
	MIB_DESC(1, 0x88, "RxJabberErr"),
	MIB_DESC(1, 0x8c, "RxPause"),
	MIB_DESC(1, 0x90, "RxPktSz64"),
	MIB_DESC(1, 0x94, "RxPktSz65To127"),
	MIB_DESC(1, 0x98, "RxPktSz128To255"),
	MIB_DESC(1, 0x9c, "RxPktSz256To511"),
	MIB_DESC(1, 0xa0, "RxPktSz512To1023"),
	MIB_DESC(1, 0xa4, "RxPktSz1024ToMax"),
	MIB_DESC(2, 0xa8, "RxBytes"),
	MIB_DESC(1, 0xb0, "RxCtrlDrop"),
	MIB_DESC(1, 0xb4, "RxIngressDrop"),
	MIB_DESC(1, 0xb8, "RxArlDrop"),
};

static int
mt7623_trgmii_write(struct mt7628_priv *priv,  u32 reg, u32 val)
{
	int ret;

	ret =  regmap_write(priv->ethernet, TRGMII_BASE(reg), val);
	if (ret < 0)
		dev_err(priv->dev,
			"failed to priv write register\n");
	return ret;
}

static u32
mt7623_trgmii_read(struct mt7628_priv *priv, u32 reg)
{
	int ret;
	u32 val;

	ret = regmap_read(priv->ethernet, TRGMII_BASE(reg), &val);
	if (ret < 0) {
		dev_err(priv->dev,
			"failed to priv read register\n");
		return ret;
	}

	return val;
}

static void
mt7623_trgmii_rmw(struct mt7628_priv *priv, u32 reg,
		  u32 mask, u32 set)
{
	u32 val;

	val = mt7623_trgmii_read(priv, reg);
	val &= ~mask;
	val |= set;
	mt7623_trgmii_write(priv, reg, val);
}

static void
mt7623_trgmii_set(struct mt7628_priv *priv, u32 reg, u32 val)
{
	mt7623_trgmii_rmw(priv, reg, 0, val);
}

static void
mt7623_trgmii_clear(struct mt7628_priv *priv, u32 reg, u32 val)
{
	mt7623_trgmii_rmw(priv, reg, val, 0);
}

static int
core_read_mmd_indirect(struct mt7628_priv *priv, int prtad, int devad)
{
	struct mii_bus *bus = priv->bus;
	int value, ret;

	/* Write the desired MMD Devad */
	ret = bus->write(bus, 0, MII_MMD_CTRL, devad);
	if (ret < 0)
		goto err;

	/* Write the desired MMD register address */
	ret = bus->write(bus, 0, MII_MMD_DATA, prtad);
	if (ret < 0)
		goto err;

	/* Select the Function : DATA with no post increment */
	ret = bus->write(bus, 0, MII_MMD_CTRL, (devad | MII_MMD_CTRL_NOINCR));
	if (ret < 0)
		goto err;

	/* Read the content of the MMD's selected register */
	value = bus->read(bus, 0, MII_MMD_DATA);

	return value;
err:
	dev_err(&bus->dev,  "failed to read mmd register\n");

	return ret;
}

static int
core_write_mmd_indirect(struct mt7628_priv *priv, int prtad,
			int devad, u32 data)
{
	struct mii_bus *bus = priv->bus;
	int ret;

	/* Write the desired MMD Devad */
	ret = bus->write(bus, 0, MII_MMD_CTRL, devad);
	if (ret < 0)
		goto err;

	/* Write the desired MMD register address */
	ret = bus->write(bus, 0, MII_MMD_DATA, prtad);
	if (ret < 0)
		goto err;

	/* Select the Function : DATA with no post increment */
	ret = bus->write(bus, 0, MII_MMD_CTRL, (devad | MII_MMD_CTRL_NOINCR));
	if (ret < 0)
		goto err;

	/* Write the data into MMD's selected register */
	ret = bus->write(bus, 0, MII_MMD_DATA, data);
err:
	if (ret < 0)
		dev_err(&bus->dev,
			"failed to write mmd register\n");
	return ret;
}

static void
core_write(struct mt7628_priv *priv, u32 reg, u32 val)
{
	struct mii_bus *bus = priv->bus;

	mutex_lock_nested(&bus->mdio_lock, MDIO_MUTEX_NESTED);

	core_write_mmd_indirect(priv, reg, MDIO_MMD_VEND2, val);

	mutex_unlock(&bus->mdio_lock);
}

static void
core_rmw(struct mt7628_priv *priv, u32 reg, u32 mask, u32 set)
{
	struct mii_bus *bus = priv->bus;
	u32 val;

	mutex_lock_nested(&bus->mdio_lock, MDIO_MUTEX_NESTED);

	val = core_read_mmd_indirect(priv, reg, MDIO_MMD_VEND2);
	val &= ~mask;
	val |= set;
	core_write_mmd_indirect(priv, reg, MDIO_MMD_VEND2, val);

	mutex_unlock(&bus->mdio_lock);
}

static void
core_set(struct mt7628_priv *priv, u32 reg, u32 val)
{
	core_rmw(priv, reg, 0, val);
}

static void
core_clear(struct mt7628_priv *priv, u32 reg, u32 val)
{
	core_rmw(priv, reg, val, 0);
}

static int
mt7628_mii_write(struct mt7628_priv *priv, u32 reg, u32 val)
{
	struct mii_bus *bus = priv->bus;
	u16 page, r, lo, hi;
	int ret;

	page = (reg >> 6) & 0x3ff;
	r  = (reg >> 2) & 0xf;
	lo = val & 0xffff;
	hi = val >> 16;

	/* MT7628 uses 31 as the pseudo port */
	ret = bus->write(bus, 0x1f, 0x1f, page);
	if (ret < 0)
		goto err;

	ret = bus->write(bus, 0x1f, r,  lo);
	if (ret < 0)
		goto err;

	ret = bus->write(bus, 0x1f, 0x10, hi);
err:
	if (ret < 0)
		dev_err(&bus->dev,
			"failed to write mt7628 register\n");
	return ret;
}

static u32
mt7628_mii_read(struct mt7628_priv *priv, u32 reg)
{
	struct mii_bus *bus = priv->bus;
	u16 page, r, lo, hi;
	int ret;

	page = (reg >> 6) & 0x3ff;
	r = (reg >> 2) & 0xf;

	/* MT7628 uses 31 as the pseudo port */
	ret = bus->write(bus, 0x1f, 0x1f, page);
	if (ret < 0) {
		dev_err(&bus->dev,
			"failed to read mt7628 register\n");
		return ret;
	}

	lo = bus->read(bus, 0x1f, r);
	hi = bus->read(bus, 0x1f, 0x10);

	return (hi << 16) | (lo & 0xffff);
}

static void
mt7628_write(struct mt7628_priv *priv, u32 reg, u32 val)
{
	struct mii_bus *bus = priv->bus;

	mutex_lock_nested(&bus->mdio_lock, MDIO_MUTEX_NESTED);

	mt7628_mii_write(priv, reg, val);

	mutex_unlock(&bus->mdio_lock);
}

static u32
_mt7628_read(struct mt7628_dummy_poll *p)
{
	struct mii_bus		*bus = p->priv->bus;
	u32 val;

	mutex_lock_nested(&bus->mdio_lock, MDIO_MUTEX_NESTED);

	val = mt7628_mii_read(p->priv, p->reg);

	mutex_unlock(&bus->mdio_lock);

	return val;
}

static u32
mt7628_read(struct mt7628_priv *priv, u32 reg)
{
	struct mt7628_dummy_poll p;

	INIT_MT7628_DUMMY_POLL(&p, priv, reg);
	return _mt7628_read(&p);
}

static void
mt7628_rmw(struct mt7628_priv *priv, u32 reg,
	   u32 mask, u32 set)
{
	struct mii_bus *bus = priv->bus;
	u32 val;

	mutex_lock_nested(&bus->mdio_lock, MDIO_MUTEX_NESTED);

	val = mt7628_mii_read(priv, reg);
	val &= ~mask;
	val |= set;
	mt7628_mii_write(priv, reg, val);

	mutex_unlock(&bus->mdio_lock);
}

static void
mt7628_set(struct mt7628_priv *priv, u32 reg, u32 val)
{
	mt7628_rmw(priv, reg, 0, val);
}

static void
mt7628_clear(struct mt7628_priv *priv, u32 reg, u32 val)
{
	mt7628_rmw(priv, reg, val, 0);
}

static int
mt7628_fdb_cmd(struct mt7628_priv *priv, enum mt7628_fdb_cmd cmd, u32 *rsp)
{
	u32 val;
	int ret;
	struct mt7628_dummy_poll p;

	/* Set the command operating upon the MAC address entries */
	val = ATC_BUSY | ATC_MAT(0) | cmd;
	mt7628_write(priv, MT7628_ATC, val);

	INIT_MT7628_DUMMY_POLL(&p, priv, MT7628_ATC);
	ret = readx_poll_timeout(_mt7628_read, &p, val,
				 !(val & ATC_BUSY), 20, 20000);
	if (ret < 0) {
		dev_err(priv->dev, "reset timeout\n");
		return ret;
	}

	/* Additional sanity for read command if the specified
	 * entry is invalid
	 */
	val = mt7628_read(priv, MT7628_ATC);
	if ((cmd == MT7628_FDB_READ) && (val & ATC_INVALID))
		return -EINVAL;

	if (rsp)
		*rsp = val;

	return 0;
}

static void
mt7628_fdb_read(struct mt7628_priv *priv, struct mt7628_fdb *fdb)
{
	u32 reg[3];
	int i;

	/* Read from ARL table into an array */
	for (i = 0; i < 3; i++) {
		reg[i] = mt7628_read(priv, MT7628_TSRA1 + (i * 4));

		dev_dbg(priv->dev, "%s(%d) reg[%d]=0x%x\n",
			__func__, __LINE__, i, reg[i]);
	}

	fdb->vid = (reg[1] >> CVID) & CVID_MASK;
	fdb->aging = (reg[2] >> AGE_TIMER) & AGE_TIMER_MASK;
	fdb->port_mask = (reg[2] >> PORT_MAP) & PORT_MAP_MASK;
	fdb->mac[0] = (reg[0] >> MAC_BYTE_0) & MAC_BYTE_MASK;
	fdb->mac[1] = (reg[0] >> MAC_BYTE_1) & MAC_BYTE_MASK;
	fdb->mac[2] = (reg[0] >> MAC_BYTE_2) & MAC_BYTE_MASK;
	fdb->mac[3] = (reg[0] >> MAC_BYTE_3) & MAC_BYTE_MASK;
	fdb->mac[4] = (reg[1] >> MAC_BYTE_4) & MAC_BYTE_MASK;
	fdb->mac[5] = (reg[1] >> MAC_BYTE_5) & MAC_BYTE_MASK;
	fdb->noarp = ((reg[2] >> ENT_STATUS) & ENT_STATUS_MASK) == STATIC_ENT;
}

static void
mt7628_fdb_write(struct mt7628_priv *priv, u16 vid,
		 u8 port_mask, const u8 *mac,
		 u8 aging, u8 type)
{
	u32 reg[3] = { 0 };
	int i;

	reg[1] |= vid & CVID_MASK;
	reg[2] |= (aging & AGE_TIMER_MASK) << AGE_TIMER;
	reg[2] |= (port_mask & PORT_MAP_MASK) << PORT_MAP;
	/* STATIC_ENT indicate that entry is static wouldn't
	 * be aged out and STATIC_EMP specified as erasing an
	 * entry
	 */
	reg[2] |= (type & ENT_STATUS_MASK) << ENT_STATUS;
	reg[1] |= mac[5] << MAC_BYTE_5;
	reg[1] |= mac[4] << MAC_BYTE_4;
	reg[0] |= mac[3] << MAC_BYTE_3;
	reg[0] |= mac[2] << MAC_BYTE_2;
	reg[0] |= mac[1] << MAC_BYTE_1;
	reg[0] |= mac[0] << MAC_BYTE_0;

	/* Write array into the ARL table */
	for (i = 0; i < 3; i++)
		mt7628_write(priv, MT7628_ATA1 + (i * 4), reg[i]);
}

static int
mt7628_pad_clk_setup(struct dsa_switch *ds, int mode)
{
	struct mt7628_priv *priv = ds->priv;
	u32 ncpo1, ssc_delta, trgint, i;

	switch (mode) {
	case PHY_INTERFACE_MODE_RGMII:
		trgint = 0;
		ncpo1 = 0x0c80;
		ssc_delta = 0x87;
		break;
	case PHY_INTERFACE_MODE_TRGMII:
		trgint = 1;
		ncpo1 = 0x1400;
		ssc_delta = 0x57;
		break;
	default:
		dev_err(priv->dev, "xMII mode %d not supported\n", mode);
		return -EINVAL;
	}

	mt7628_rmw(priv, MT7628_P6ECR, P6_INTF_MODE_MASK,
		   P6_INTF_MODE(trgint));

	/* Lower Tx Driving for TRGMII path */
	for (i = 0 ; i < NUM_TRGMII_CTRL ; i++)
		mt7628_write(priv, MT7628_TRGMII_TD_ODT(i),
			     TD_DM_DRVP(8) | TD_DM_DRVN(8));

	/* Setup core clock for MT7628 */
	if (!trgint) {
		/* Disable MT7628 core clock */
		core_clear(priv, CORE_TRGMII_GSW_CLK_CG, REG_GSWCK_EN);

		/* Disable PLL, since phy_device has not yet been created
		 * provided for phy_[read,write]_mmd_indirect is called, we
		 * provide our own core_write_mmd_indirect to complete this
		 * function.
		 */
		core_write_mmd_indirect(priv,
					CORE_GSWPLL_GRP1,
					MDIO_MMD_VEND2,
					0);

		/* Set core clock into 500Mhz */
		core_write(priv, CORE_GSWPLL_GRP2,
			   RG_GSWPLL_POSDIV_500M(1) |
			   RG_GSWPLL_FBKDIV_500M(25));

		/* Enable PLL */
		core_write(priv, CORE_GSWPLL_GRP1,
			   RG_GSWPLL_EN_PRE |
			   RG_GSWPLL_POSDIV_200M(2) |
			   RG_GSWPLL_FBKDIV_200M(32));

		/* Enable MT7628 core clock */
		core_set(priv, CORE_TRGMII_GSW_CLK_CG, REG_GSWCK_EN);
	}

	/* Setup the MT7628 TRGMII Tx Clock */
	core_set(priv, CORE_TRGMII_GSW_CLK_CG, REG_GSWCK_EN);
	core_write(priv, CORE_PLL_GROUP5, RG_LCDDS_PCW_NCPO1(ncpo1));
	core_write(priv, CORE_PLL_GROUP6, RG_LCDDS_PCW_NCPO0(0));
	core_write(priv, CORE_PLL_GROUP10, RG_LCDDS_SSC_DELTA(ssc_delta));
	core_write(priv, CORE_PLL_GROUP11, RG_LCDDS_SSC_DELTA1(ssc_delta));
	core_write(priv, CORE_PLL_GROUP4,
		   RG_SYSPLL_DDSFBK_EN | RG_SYSPLL_BIAS_EN |
		   RG_SYSPLL_BIAS_LPF_EN);
	core_write(priv, CORE_PLL_GROUP2,
		   RG_SYSPLL_EN_NORMAL | RG_SYSPLL_VODEN |
		   RG_SYSPLL_POSDIV(1));
	core_write(priv, CORE_PLL_GROUP7,
		   RG_LCDDS_PCW_NCPO_CHG | RG_LCCDS_C(3) |
		   RG_LCDDS_PWDB | RG_LCDDS_ISO_EN);
	core_set(priv, CORE_TRGMII_GSW_CLK_CG,
		 REG_GSWCK_EN | REG_TRGMIICK_EN);

	if (!trgint)
		for (i = 0 ; i < NUM_TRGMII_CTRL; i++)
			mt7628_rmw(priv, MT7628_TRGMII_RD(i),
				   RD_TAP_MASK, RD_TAP(16));
	else
		mt7623_trgmii_set(priv, GSW_INTF_MODE, INTF_MODE_TRGMII);

	return 0;
}

static int
mt7623_pad_clk_setup(struct dsa_switch *ds)
{
	struct mt7628_priv *priv = ds->priv;
	int i;

	for (i = 0 ; i < NUM_TRGMII_CTRL; i++)
		mt7623_trgmii_write(priv, GSW_TRGMII_TD_ODT(i),
				    TD_DM_DRVP(8) | TD_DM_DRVN(8));

	mt7623_trgmii_set(priv, GSW_TRGMII_RCK_CTRL, RX_RST | RXC_DQSISEL);
	mt7623_trgmii_clear(priv, GSW_TRGMII_RCK_CTRL, RX_RST);

	return 0;
}

static void
mt7628_mib_reset(struct dsa_switch *ds)
{
	struct mt7628_priv *priv = ds->priv;

	mt7628_write(priv, MT7628_MIB_CCR, CCR_MIB_FLUSH);
	mt7628_write(priv, MT7628_MIB_CCR, CCR_MIB_ACTIVATE);
}

static void
mt7628_port_set_status(struct mt7628_priv *priv, int port, int enable)
{
	u32 mask = PMCR_TX_EN | PMCR_RX_EN;

	if (enable)
		mt7628_set(priv, MT7628_PMCR_P(port), mask);
	else
		mt7628_clear(priv, MT7628_PMCR_P(port), mask);
}

static int mt7628_phy_read(struct dsa_switch *ds, int port, int regnum)
{
	struct mt7628_priv *priv = ds->priv;

	return mdiobus_read_nested(priv->bus, port, regnum);
}

static int mt7628_phy_write(struct dsa_switch *ds, int port, int regnum,
			    u16 val)
{
	struct mt7628_priv *priv = ds->priv;

	return mdiobus_write_nested(priv->bus, port, regnum, val);
}

static void
mt7628_get_strings(struct dsa_switch *ds, int port, u32 stringset,
		   uint8_t *data)
{
	int i;

	if (stringset != ETH_SS_STATS)
		return;

	for (i = 0; i < ARRAY_SIZE(mt7628_mib); i++)
		strncpy(data + i * ETH_GSTRING_LEN, mt7628_mib[i].name,
			ETH_GSTRING_LEN);
}

static void
mt7628_get_ethtool_stats(struct dsa_switch *ds, int port,
			 uint64_t *data)
{
	struct mt7628_priv *priv = ds->priv;
	const struct mt7628_mib_desc *mib;
	u32 reg, i;
	u64 hi;

	for (i = 0; i < ARRAY_SIZE(mt7628_mib); i++) {
		mib = &mt7628_mib[i];
		reg = MT7628_PORT_MIB_COUNTER(port) + mib->offset;

		data[i] = mt7628_read(priv, reg);
		if (mib->size == 2) {
			hi = mt7628_read(priv, reg + 4);
			data[i] |= hi << 32;
		}
	}
}

static int
mt7628_get_sset_count(struct dsa_switch *ds, int port, int sset)
{
	if (sset != ETH_SS_STATS)
		return 0;

	return ARRAY_SIZE(mt7628_mib);
}

static void mt7628_adjust_link(struct dsa_switch *ds, int port,
			       struct phy_device *phydev)
{
	struct mt7628_priv *priv = ds->priv;

	if (phy_is_pseudo_fixed_link(phydev)) {
		if (priv->id == ID_MT7628) {
			dev_dbg(priv->dev, "phy-mode for master device = %x\n",
				phydev->interface);

			/* Setup TX circuit incluing relevant PAD and driving */
			mt7628_pad_clk_setup(ds, phydev->interface);

			/* Setup RX circuit, relevant PAD and driving on the
			 * host which must be placed after the setup on the
			 * device side is all finished.
			 */
			mt7623_pad_clk_setup(ds);
		}
	} else {
		u16 lcl_adv = 0, rmt_adv = 0;
		u8 flowctrl;
		u32 mcr = PMCR_USERP_LINK | PMCR_FORCE_MODE;

		switch (phydev->speed) {
		case SPEED_1000:
			mcr |= PMCR_FORCE_SPEED_1000;
			break;
		case SPEED_100:
			mcr |= PMCR_FORCE_SPEED_100;
			break;
		}

		if (phydev->link)
			mcr |= PMCR_FORCE_LNK;

		if (phydev->duplex) {
			mcr |= PMCR_FORCE_FDX;

			if (phydev->pause)
				rmt_adv = LPA_PAUSE_CAP;
			if (phydev->asym_pause)
				rmt_adv |= LPA_PAUSE_ASYM;

			lcl_adv = linkmode_adv_to_lcl_adv_t(
				phydev->advertising);
			flowctrl = mii_resolve_flowctrl_fdx(lcl_adv, rmt_adv);

			if (flowctrl & FLOW_CTRL_TX)
				mcr |= PMCR_TX_FC_EN;
			if (flowctrl & FLOW_CTRL_RX)
				mcr |= PMCR_RX_FC_EN;
		}
		mt7628_write(priv, MT7628_PMCR_P(port), mcr);
	}
}

static int
mt7628_cpu_port_enable(struct mt7628_priv *priv,
		       int port)
{
	/* Enable Mediatek header mode on the cpu port */
	mt7628_write(priv, MT7628_PVC_P(port),
		     PORT_SPEC_TAG);

	/* Setup the MAC by default for the cpu port */
	mt7628_write(priv, MT7628_PMCR_P(port), PMCR_CPUP_LINK);

	/* Disable auto learning on the cpu port */
	mt7628_set(priv, MT7628_PSC_P(port), SA_DIS);

	/* Unknown unicast frame fordwarding to the cpu port */
	mt7628_set(priv, MT7628_MFC, UNU_FFP(BIT(port)));

	/* Set CPU port number */
	if (priv->id == ID_MT7621)
		mt7628_rmw(priv, MT7628_MFC, CPU_MASK, CPU_EN | CPU_PORT(port));

	/* CPU port gets connected to all user ports of
	 * the switch
	 */
	mt7628_write(priv, MT7628_PCR_P(port),
		     PCR_MATRIX(dsa_user_ports(priv->ds)));

	return 0;
}

static int
mt7628_port_enable(struct dsa_switch *ds, int port,
		   struct phy_device *phy)
{
	struct mt7628_priv *priv = ds->priv;

	mutex_lock(&priv->reg_mutex);

	/* Setup the MAC for the user port */
	mt7628_write(priv, MT7628_PMCR_P(port), PMCR_USERP_LINK);

	/* Allow the user port gets connected to the cpu port and also
	 * restore the port matrix if the port is the member of a certain
	 * bridge.
	 */
	priv->ports[port].pm |= PCR_MATRIX(BIT(MT7628_CPU_PORT));
	priv->ports[port].enable = true;
	mt7628_rmw(priv, MT7628_PCR_P(port), PCR_MATRIX_MASK,
		   priv->ports[port].pm);
	mt7628_port_set_status(priv, port, 1);

	mutex_unlock(&priv->reg_mutex);

	return 0;
}

static void
mt7628_port_disable(struct dsa_switch *ds, int port)
{
	struct mt7628_priv *priv = ds->priv;

	mutex_lock(&priv->reg_mutex);

	/* Clear up all port matrix which could be restored in the next
	 * enablement for the port.
	 */
	priv->ports[port].enable = false;
	mt7628_rmw(priv, MT7628_PCR_P(port), PCR_MATRIX_MASK,
		   PCR_MATRIX_CLR);
	mt7628_port_set_status(priv, port, 0);

	mutex_unlock(&priv->reg_mutex);
}

static void
mt7628_stp_state_set(struct dsa_switch *ds, int port, u8 state)
{
	struct mt7628_priv *priv = ds->priv;
	u32 stp_state;

	switch (state) {
	case BR_STATE_DISABLED:
		stp_state = MT7628_STP_DISABLED;
		break;
	case BR_STATE_BLOCKING:
		stp_state = MT7628_STP_BLOCKING;
		break;
	case BR_STATE_LISTENING:
		stp_state = MT7628_STP_LISTENING;
		break;
	case BR_STATE_LEARNING:
		stp_state = MT7628_STP_LEARNING;
		break;
	case BR_STATE_FORWARDING:
	default:
		stp_state = MT7628_STP_FORWARDING;
		break;
	}

	mt7628_rmw(priv, MT7628_SSP_P(port), FID_PST_MASK, stp_state);
}

static int
mt7628_port_bridge_join(struct dsa_switch *ds, int port,
			struct net_device *bridge)
{
	struct mt7628_priv *priv = ds->priv;
	u32 port_bitmap = BIT(MT7628_CPU_PORT);
	int i;

	mutex_lock(&priv->reg_mutex);

	for (i = 0; i < MT7628_NUM_PORTS; i++) {
		/* Add this port to the port matrix of the other ports in the
		 * same bridge. If the port is disabled, port matrix is kept
		 * and not being setup until the port becomes enabled.
		 */
		if (dsa_is_user_port(ds, i) && i != port) {
			if (dsa_to_port(ds, i)->bridge_dev != bridge)
				continue;
			if (priv->ports[i].enable)
				mt7628_set(priv, MT7628_PCR_P(i),
					   PCR_MATRIX(BIT(port)));
			priv->ports[i].pm |= PCR_MATRIX(BIT(port));

			port_bitmap |= BIT(i);
		}
	}

	/* Add the all other ports to this port matrix. */
	if (priv->ports[port].enable)
		mt7628_rmw(priv, MT7628_PCR_P(port),
			   PCR_MATRIX_MASK, PCR_MATRIX(port_bitmap));
	priv->ports[port].pm |= PCR_MATRIX(port_bitmap);

	mutex_unlock(&priv->reg_mutex);

	return 0;
}

static void
mt7628_port_set_vlan_unaware(struct dsa_switch *ds, int port)
{
	struct mt7628_priv *priv = ds->priv;
	bool all_user_ports_removed = true;
	int i;

	/* When a port is removed from the bridge, the port would be set up
	 * back to the default as is at initial boot which is a VLAN-unaware
	 * port.
	 */
	mt7628_rmw(priv, MT7628_PCR_P(port), PCR_PORT_VLAN_MASK,
		   MT7628_PORT_MATRIX_MODE);
	mt7628_rmw(priv, MT7628_PVC_P(port), VLAN_ATTR_MASK,
		   VLAN_ATTR(MT7628_VLAN_TRANSPARENT));

	for (i = 0; i < MT7628_NUM_PORTS; i++) {
		if (dsa_is_user_port(ds, i) &&
		    dsa_port_is_vlan_filtering(&ds->ports[i])) {
			all_user_ports_removed = false;
			break;
		}
	}

	/* CPU port also does the same thing until all user ports belonging to
	 * the CPU port get out of VLAN filtering mode.
	 */
	if (all_user_ports_removed) {
		mt7628_write(priv, MT7628_PCR_P(MT7628_CPU_PORT),
			     PCR_MATRIX(dsa_user_ports(priv->ds)));
		mt7628_write(priv, MT7628_PVC_P(MT7628_CPU_PORT),
			     PORT_SPEC_TAG);
	}
}

static void
mt7628_port_set_vlan_aware(struct dsa_switch *ds, int port)
{
	struct mt7628_priv *priv = ds->priv;

	/* The real fabric path would be decided on the membership in the
	 * entry of VLAN table. PCR_MATRIX set up here with ALL_MEMBERS
	 * means potential VLAN can be consisting of certain subset of all
	 * ports.
	 */
	mt7628_rmw(priv, MT7628_PCR_P(port),
		   PCR_MATRIX_MASK, PCR_MATRIX(MT7628_ALL_MEMBERS));

	/* Trapped into security mode allows packet forwarding through VLAN
	 * table lookup.
	 */
	mt7628_rmw(priv, MT7628_PCR_P(port), PCR_PORT_VLAN_MASK,
		   MT7628_PORT_SECURITY_MODE);

	/* Set the port as a user port which is to be able to recognize VID
	 * from incoming packets before fetching entry within the VLAN table.
	 */
	mt7628_rmw(priv, MT7628_PVC_P(port), VLAN_ATTR_MASK,
		   VLAN_ATTR(MT7628_VLAN_USER));
}

static void
mt7628_port_bridge_leave(struct dsa_switch *ds, int port,
			 struct net_device *bridge)
{
	struct mt7628_priv *priv = ds->priv;
	int i;

	mutex_lock(&priv->reg_mutex);

	for (i = 0; i < MT7628_NUM_PORTS; i++) {
		/* Remove this port from the port matrix of the other ports
		 * in the same bridge. If the port is disabled, port matrix
		 * is kept and not being setup until the port becomes enabled.
		 * And the other port's port matrix cannot be broken when the
		 * other port is still a VLAN-aware port.
		 */
		if (dsa_is_user_port(ds, i) && i != port &&
		   !dsa_port_is_vlan_filtering(&ds->ports[i])) {
			if (dsa_to_port(ds, i)->bridge_dev != bridge)
				continue;
			if (priv->ports[i].enable)
				mt7628_clear(priv, MT7628_PCR_P(i),
					     PCR_MATRIX(BIT(port)));
			priv->ports[i].pm &= ~PCR_MATRIX(BIT(port));
		}
	}

	/* Set the cpu port to be the only one in the port matrix of
	 * this port.
	 */
	if (priv->ports[port].enable)
		mt7628_rmw(priv, MT7628_PCR_P(port), PCR_MATRIX_MASK,
			   PCR_MATRIX(BIT(MT7628_CPU_PORT)));
	priv->ports[port].pm = PCR_MATRIX(BIT(MT7628_CPU_PORT));

	mutex_unlock(&priv->reg_mutex);
}

static int
mt7628_port_fdb_add(struct dsa_switch *ds, int port,
		    const unsigned char *addr, u16 vid)
{
	struct mt7628_priv *priv = ds->priv;
	int ret;
	u8 port_mask = BIT(port);

	mutex_lock(&priv->reg_mutex);
	mt7628_fdb_write(priv, vid, port_mask, addr, -1, STATIC_ENT);
	ret = mt7628_fdb_cmd(priv, MT7628_FDB_WRITE, NULL);
	mutex_unlock(&priv->reg_mutex);

	return ret;
}

static int
mt7628_port_fdb_del(struct dsa_switch *ds, int port,
		    const unsigned char *addr, u16 vid)
{
	struct mt7628_priv *priv = ds->priv;
	int ret;
	u8 port_mask = BIT(port);

	mutex_lock(&priv->reg_mutex);
	mt7628_fdb_write(priv, vid, port_mask, addr, -1, STATIC_EMP);
	ret = mt7628_fdb_cmd(priv, MT7628_FDB_WRITE, NULL);
	mutex_unlock(&priv->reg_mutex);

	return ret;
}

static int
mt7628_port_fdb_dump(struct dsa_switch *ds, int port,
		     dsa_fdb_dump_cb_t *cb, void *data)
{
	struct mt7628_priv *priv = ds->priv;
	struct mt7628_fdb _fdb = { 0 };
	int cnt = MT7628_NUM_FDB_RECORDS;
	int ret = 0;
	u32 rsp = 0;

	mutex_lock(&priv->reg_mutex);

	ret = mt7628_fdb_cmd(priv, MT7628_FDB_START, &rsp);
	if (ret < 0)
		goto err;

	do {
		if (rsp & ATC_SRCH_HIT) {
			mt7628_fdb_read(priv, &_fdb);
			if (_fdb.port_mask & BIT(port)) {
				ret = cb(_fdb.mac, _fdb.vid, _fdb.noarp,
					 data);
				if (ret < 0)
					break;
			}
		}
	} while (--cnt &&
		 !(rsp & ATC_SRCH_END) &&
		 !mt7628_fdb_cmd(priv, MT7628_FDB_NEXT, &rsp));
err:
	mutex_unlock(&priv->reg_mutex);

	return 0;
}

static int
mt7628_vlan_cmd(struct mt7628_priv *priv, enum mt7628_vlan_cmd cmd, u16 vid)
{
	struct mt7628_dummy_poll p;
	u32 val;
	int ret;

	val = VTCR_BUSY | VTCR_FUNC(cmd) | vid;
	mt7628_write(priv, MT7628_VTCR, val);

	INIT_MT7628_DUMMY_POLL(&p, priv, MT7628_VTCR);
	ret = readx_poll_timeout(_mt7628_read, &p, val,
				 !(val & VTCR_BUSY), 20, 20000);
	if (ret < 0) {
		dev_err(priv->dev, "poll timeout\n");
		return ret;
	}

	val = mt7628_read(priv, MT7628_VTCR);
	if (val & VTCR_INVALID) {
		dev_err(priv->dev, "read VTCR invalid\n");
		return -EINVAL;
	}

	return 0;
}

static int
mt7628_port_vlan_filtering(struct dsa_switch *ds, int port,
			   bool vlan_filtering)
{
	if (vlan_filtering) {
		/* The port is being kept as VLAN-unaware port when bridge is
		 * set up with vlan_filtering not being set, Otherwise, the
		 * port and the corresponding CPU port is required the setup
		 * for becoming a VLAN-aware port.
		 */
		mt7628_port_set_vlan_aware(ds, port);
		mt7628_port_set_vlan_aware(ds, MT7628_CPU_PORT);
	} else {
		mt7628_port_set_vlan_unaware(ds, port);
	}

	return 0;
}

static int
mt7628_port_vlan_prepare(struct dsa_switch *ds, int port,
			 const struct switchdev_obj_port_vlan *vlan)
{
	/* nothing needed */

	return 0;
}

static void
mt7628_hw_vlan_add(struct mt7628_priv *priv,
		   struct mt7628_hw_vlan_entry *entry)
{
	u8 new_members;
	u32 val;

	new_members = entry->old_members | BIT(entry->port) |
		      BIT(MT7628_CPU_PORT);

	/* Validate the entry with independent learning, create egress tag per
	 * VLAN and joining the port as one of the port members.
	 */
	val = IVL_MAC | VTAG_EN | PORT_MEM(new_members) | VLAN_VALID;
	mt7628_write(priv, MT7628_VAWD1, val);

	/* Decide whether adding tag or not for those outgoing packets from the
	 * port inside the VLAN.
	 */
	val = entry->untagged ? MT7628_VLAN_EGRESS_UNTAG :
				MT7628_VLAN_EGRESS_TAG;
	mt7628_rmw(priv, MT7628_VAWD2,
		   ETAG_CTRL_P_MASK(entry->port),
		   ETAG_CTRL_P(entry->port, val));

	/* CPU port is always taken as a tagged port for serving more than one
	 * VLANs across and also being applied with egress type stack mode for
	 * that VLAN tags would be appended after hardware special tag used as
	 * DSA tag.
	 */
	mt7628_rmw(priv, MT7628_VAWD2,
		   ETAG_CTRL_P_MASK(MT7628_CPU_PORT),
		   ETAG_CTRL_P(MT7628_CPU_PORT,
			       MT7628_VLAN_EGRESS_STACK));
}

static void
mt7628_hw_vlan_del(struct mt7628_priv *priv,
		   struct mt7628_hw_vlan_entry *entry)
{
	u8 new_members;
	u32 val;

	new_members = entry->old_members & ~BIT(entry->port);

	val = mt7628_read(priv, MT7628_VAWD1);
	if (!(val & VLAN_VALID)) {
		dev_err(priv->dev,
			"Cannot be deleted due to invalid entry\n");
		return;
	}

	/* If certain member apart from CPU port is still alive in the VLAN,
	 * the entry would be kept valid. Otherwise, the entry is got to be
	 * disabled.
	 */
	if (new_members && new_members != BIT(MT7628_CPU_PORT)) {
		val = IVL_MAC | VTAG_EN | PORT_MEM(new_members) |
		      VLAN_VALID;
		mt7628_write(priv, MT7628_VAWD1, val);
	} else {
		mt7628_write(priv, MT7628_VAWD1, 0);
		mt7628_write(priv, MT7628_VAWD2, 0);
	}
}

static void
mt7628_hw_vlan_update(struct mt7628_priv *priv, u16 vid,
		      struct mt7628_hw_vlan_entry *entry,
		      mt7628_vlan_op vlan_op)
{
	u32 val;

	/* Fetch entry */
	mt7628_vlan_cmd(priv, MT7628_VTCR_RD_VID, vid);

	val = mt7628_read(priv, MT7628_VAWD1);

	entry->old_members = (val >> PORT_MEM_SHFT) & PORT_MEM_MASK;

	/* Manipulate entry */
	vlan_op(priv, entry);

	/* Flush result to hardware */
	mt7628_vlan_cmd(priv, MT7628_VTCR_WR_VID, vid);
}

static void
mt7628_port_vlan_add(struct dsa_switch *ds, int port,
		     const struct switchdev_obj_port_vlan *vlan)
{
	bool untagged = vlan->flags & BRIDGE_VLAN_INFO_UNTAGGED;
	bool pvid = vlan->flags & BRIDGE_VLAN_INFO_PVID;
	struct mt7628_hw_vlan_entry new_entry;
	struct mt7628_priv *priv = ds->priv;
	u16 vid;

	/* The port is kept as VLAN-unaware if bridge with vlan_filtering not
	 * being set.
	 */
	if (!dsa_port_is_vlan_filtering(&ds->ports[port]))
		return;

	mutex_lock(&priv->reg_mutex);

	for (vid = vlan->vid_begin; vid <= vlan->vid_end; ++vid) {
		mt7628_hw_vlan_entry_init(&new_entry, port, untagged);
		mt7628_hw_vlan_update(priv, vid, &new_entry,
				      mt7628_hw_vlan_add);
	}

	if (pvid) {
		mt7628_rmw(priv, MT7628_PPBV1_P(port), G0_PORT_VID_MASK,
			   G0_PORT_VID(vlan->vid_end));
		priv->ports[port].pvid = vlan->vid_end;
	}

	mutex_unlock(&priv->reg_mutex);
}

static int
mt7628_port_vlan_del(struct dsa_switch *ds, int port,
		     const struct switchdev_obj_port_vlan *vlan)
{
	struct mt7628_hw_vlan_entry target_entry;
	struct mt7628_priv *priv = ds->priv;
	u16 vid, pvid;

	/* The port is kept as VLAN-unaware if bridge with vlan_filtering not
	 * being set.
	 */
	if (!dsa_port_is_vlan_filtering(&ds->ports[port]))
		return 0;

	mutex_lock(&priv->reg_mutex);

	pvid = priv->ports[port].pvid;
	for (vid = vlan->vid_begin; vid <= vlan->vid_end; ++vid) {
		mt7628_hw_vlan_entry_init(&target_entry, port, 0);
		mt7628_hw_vlan_update(priv, vid, &target_entry,
				      mt7628_hw_vlan_del);

		/* PVID is being restored to the default whenever the PVID port
		 * is being removed from the VLAN.
		 */
		if (pvid == vid)
			pvid = G0_PORT_VID_DEF;
	}

	mt7628_rmw(priv, MT7628_PPBV1_P(port), G0_PORT_VID_MASK, pvid);
	priv->ports[port].pvid = pvid;

	mutex_unlock(&priv->reg_mutex);

	return 0;
}

static enum dsa_tag_protocol
mtk_get_tag_protocol(struct dsa_switch *ds, int port)
{
	struct mt7628_priv *priv = ds->priv;

	if (port != MT7628_CPU_PORT) {
		dev_warn(priv->dev,
			 "port not matched with tagging CPU port\n");
		return DSA_TAG_PROTO_NONE;
	} else {
		return DSA_TAG_PROTO_MTK;
	}
}
#endif

static int mt7628_setup(struct dsa_switch *ds)
{
	printk("%s (%d)!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", __func__, __LINE__); // test-only
#if 0
	struct mt7628_priv *priv = ds->priv;
	int ret, i;
	u32 id, val;
	struct device_node *dn;
	struct mt7628_dummy_poll p;

	/* The parent node of master netdev which holds the common system
	 * controller also is the container for two GMACs nodes representing
	 * as two netdev instances.
	 */
	dn = ds->ports[MT7628_CPU_PORT].master->dev.of_node->parent;

	if (priv->id == ID_MT7628) {
		priv->ethernet = syscon_node_to_regmap(dn);
		if (IS_ERR(priv->ethernet))
			return PTR_ERR(priv->ethernet);

		regulator_set_voltage(priv->core_pwr, 1000000, 1000000);
		ret = regulator_enable(priv->core_pwr);
		if (ret < 0) {
			dev_err(priv->dev,
				"Failed to enable core power: %d\n", ret);
			return ret;
		}

		regulator_set_voltage(priv->io_pwr, 3300000, 3300000);
		ret = regulator_enable(priv->io_pwr);
		if (ret < 0) {
			dev_err(priv->dev, "Failed to enable io pwr: %d\n",
				ret);
			return ret;
		}
	}

	/* Reset whole chip through gpio pin or memory-mapped registers for
	 * different type of hardware
	 */
	if (priv->mcm) {
		reset_control_assert(priv->rstc);
		usleep_range(1000, 1100);
		reset_control_deassert(priv->rstc);
	} else {
		gpiod_set_value_cansleep(priv->reset, 0);
		usleep_range(1000, 1100);
		gpiod_set_value_cansleep(priv->reset, 1);
	}

	/* Waiting for MT7628 got to stable */
	INIT_MT7628_DUMMY_POLL(&p, priv, MT7628_HWTRAP);
	ret = readx_poll_timeout(_mt7628_read, &p, val, val != 0,
				 20, 1000000);
	if (ret < 0) {
		dev_err(priv->dev, "reset timeout\n");
		return ret;
	}

	id = mt7628_read(priv, MT7628_CREV);
	id >>= CHIP_NAME_SHIFT;
	if (id != MT7628_ID) {
		dev_err(priv->dev, "chip %x can't be supported\n", id);
		return -ENODEV;
	}

	/* Reset the switch through internal reset */
	mt7628_write(priv, MT7628_SYS_CTRL,
		     SYS_CTRL_PHY_RST | SYS_CTRL_SW_RST |
		     SYS_CTRL_REG_RST);

	/* Enable Port 6 only; P5 as GMAC5 which currently is not supported */
	val = mt7628_read(priv, MT7628_MHWTRAP);
	val &= ~MHWTRAP_P6_DIS & ~MHWTRAP_PHY_ACCESS;
	val |= MHWTRAP_MANUAL;
	mt7628_write(priv, MT7628_MHWTRAP, val);

	/* Enable and reset MIB counters */
	mt7628_mib_reset(ds);

	mt7628_clear(priv, MT7628_MFC, UNU_FFP_MASK);

	for (i = 0; i < MT7628_NUM_PORTS; i++) {
		/* Disable forwarding by default on all ports */
		mt7628_rmw(priv, MT7628_PCR_P(i), PCR_MATRIX_MASK,
			   PCR_MATRIX_CLR);

		if (dsa_is_cpu_port(ds, i))
			mt7628_cpu_port_enable(priv, i);
		else
			mt7628_port_disable(ds, i);
	}

	/* Flush the FDB table */
	ret = mt7628_fdb_cmd(priv, MT7628_FDB_FLUSH, NULL);
	if (ret < 0)
		return ret;
#endif

	return 0;
}

#include <linux/bitfield.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

// test-only: for sysc_read/write()
#include <asm/mach-ralink/ralink_regs.h>
#include <asm/mach-ralink/mt7620.h>

/* System controller register */
#define MT7628_RSTCTRL_REG	0x34
#define RSTCTRL_EPHY_RST	BIT(24)

#define MT7628_AGPIO_CFG_REG	0x3c
#define MT7628_EPHY_GPIO_AIO_EN	GENMASK(20, 17)
#define MT7628_EPHY_P0_DIS	BIT(16)

#define MT7628_GPIO2_MODE_REG	0x64

/* Ethernet switch register */
#define MT7628_SWITCH_FCT0	0x0008
#define MT7628_SWITCH_PFC1	0x0014
#define MT7628_SWITCH_FPA	0x0084
#define MT7628_SWITCH_SOCPC	0x008c
#define MT7628_SWITCH_POC0	0x0090
#define MT7628_SWITCH_POC2	0x0098
#define MT7628_SWITCH_SGC	0x009c
#define MT7628_SWITCH_PCR0	0x00c0
#define PCR0_PHY_ADDR		GENMASK(4, 0)
#define PCR0_PHY_REG		GENMASK(12, 8)
#define PCR0_WT_PHY_CMD		BIT(13)
#define PCR0_RD_PHY_CMD		BIT(14)
#define PCR0_WT_DATA		GENMASK(31, 16)

#define MT7628_SWITCH_PCR1	0x00c4
#define PCR1_WT_DONE		BIT(0)
#define PCR1_RD_RDY		BIT(1)
#define PCR1_RD_DATA		GENMASK(31, 16)

#define MT7628_SWITCH_FPA1	0x00c8
#define MT7628_SWITCH_FCT2	0x00cc
#define MT7628_SWITCH_SGC2	0x00e4
#define MT7628_SWITCH_BMU_CTRL	0x0110

#define CONFIG_MDIO_TIMEOUT	100
#define CONFIG_DMA_STOP_TIMEOUT	100
#define CONFIG_TX_DMA_TIMEOUT	100

#define LINK_DELAY_TIME		500		/* 500 ms */
#define LINK_TIMEOUT		10000		/* 10 seconds */

// test-only: rename
struct mt7628_esw_priv {
	void __iomem *base;		/* switch base address */

	struct dsa_switch	*ds;

	struct mii_dev *bus;
};

static int mdio_wait_read(struct mt7628_esw_priv *priv, u32 mask, bool mask_set)
{
	void __iomem *base = priv->base;
	int ret;

#if 0
	ret = wait_for_bit_le32(base + MT7628_SWITCH_PCR1, mask, mask_set,
				CONFIG_MDIO_TIMEOUT, false);
	if (ret) {
		printf("MDIO operation timeout!\n");
		return -ETIMEDOUT;
	}
#else
	mdelay(10);
#endif

	return 0;
}

static int mii_mgr_read(struct mt7628_esw_priv *priv,
			u32 phy_addr, u32 phy_register, u32 *read_data)
{
	void __iomem *base = priv->base;
	u32 status = 0;
	u32 ret;

	*read_data = 0xffff;
	/* Make sure previous read operation is complete */
	ret = mdio_wait_read(priv, PCR1_RD_RDY, false);
	if (ret)
		return ret;

	writel(PCR0_RD_PHY_CMD |
	       FIELD_PREP(PCR0_PHY_REG, phy_register) |
	       FIELD_PREP(PCR0_PHY_ADDR, phy_addr),
	       base + MT7628_SWITCH_PCR0);

	/* Make sure previous read operation is complete */
	ret = mdio_wait_read(priv, PCR1_RD_RDY, true);
	if (ret)
		return ret;

	status = readl(base + MT7628_SWITCH_PCR1);
	*read_data = FIELD_GET(PCR1_RD_DATA, status);

	return 0;
}

static int mii_mgr_write(struct mt7628_esw_priv *priv,
			 u32 phy_addr, u32 phy_register, u32 write_data)
{
	void __iomem *base = priv->base;
	u32 data;
	int ret;

	/* Make sure previous write operation is complete */
	ret = mdio_wait_read(priv, PCR1_WT_DONE, false);
	if (ret)
		return ret;

	data = FIELD_PREP(PCR0_WT_DATA, write_data) |
		FIELD_PREP(PCR0_PHY_REG, phy_register) |
		FIELD_PREP(PCR0_PHY_ADDR, phy_addr) |
		PCR0_WT_PHY_CMD;
	writel(data, base + MT7628_SWITCH_PCR0);

	return mdio_wait_read(priv, PCR1_WT_DONE, true);
}

static void mt7628_ephy_init(struct mt7628_esw_priv *priv)
{
	int i;

	printk("%s (%d)\n", __func__, __LINE__); // test-only
	mii_mgr_write(priv, 0, 31, 0x2000);	/* change G2 page */
	mii_mgr_write(priv, 0, 26, 0x0000);

	for (i = 0; i < 5; i++) {
		mii_mgr_write(priv, i, 31, 0x8000);	/* change L0 page */
		mii_mgr_write(priv, i,  0, 0x3100);

		/* EEE disable */
		mii_mgr_write(priv, i, 30, 0xa000);
		mii_mgr_write(priv, i, 31, 0xa000);	/* change L2 page */
		mii_mgr_write(priv, i, 16, 0x0606);
		mii_mgr_write(priv, i, 23, 0x0f0e);
		mii_mgr_write(priv, i, 24, 0x1610);
		mii_mgr_write(priv, i, 30, 0x1f15);
		mii_mgr_write(priv, i, 28, 0x6111);
	}

	/* 100Base AOI setting */
	mii_mgr_write(priv, 0, 31, 0x5000);	/* change G5 page */
	mii_mgr_write(priv, 0, 19, 0x004a);
	mii_mgr_write(priv, 0, 20, 0x015a);
	mii_mgr_write(priv, 0, 21, 0x00ee);
	mii_mgr_write(priv, 0, 22, 0x0033);
	mii_mgr_write(priv, 0, 23, 0x020a);
	mii_mgr_write(priv, 0, 24, 0x0000);
	mii_mgr_write(priv, 0, 25, 0x024a);
	mii_mgr_write(priv, 0, 26, 0x035a);
	mii_mgr_write(priv, 0, 27, 0x02ee);
	mii_mgr_write(priv, 0, 28, 0x0233);
	mii_mgr_write(priv, 0, 29, 0x000a);
	mii_mgr_write(priv, 0, 30, 0x0000);

	/* Fix EPHY idle state abnormal behavior */
	mii_mgr_write(priv, 0, 31, 0x4000);	/* change G4 page */
	mii_mgr_write(priv, 0, 29, 0x000d);
	mii_mgr_write(priv, 0, 30, 0x0500);
}

static void rt305x_esw_init(struct mt7628_esw_priv *priv)
{
	void __iomem *base = priv->base;

	printk("%s (%d)\n", __func__, __LINE__); // test-only
	/*
	 * FC_RLS_TH=200, FC_SET_TH=160
	 * DROP_RLS=120, DROP_SET_TH=80
	 */
	writel(0xc8a07850, base + MT7628_SWITCH_FCT0);
	writel(0x00000000, base + MT7628_SWITCH_SGC2);
	writel(0x00405555, base + MT7628_SWITCH_PFC1);
	writel(0x00007f7f, base + MT7628_SWITCH_POC0);
	writel(0x00007f7f, base + MT7628_SWITCH_POC2);	/* disable VLAN */
	writel(0x0002500c, base + MT7628_SWITCH_FCT2);
	/* hashing algorithm=XOR48, aging interval=300sec */
	writel(0x0008a301, base + MT7628_SWITCH_SGC);
	writel(0x02404040, base + MT7628_SWITCH_SOCPC);

	/* Ext PHY Addr=0x1f */
	writel(0x3f502b28, base + MT7628_SWITCH_FPA1);
	writel(0x00000000, base + MT7628_SWITCH_FPA);
	/* 1us cycle number=125 (FE's clock=125Mhz) */
	writel(0x7d000000, base + MT7628_SWITCH_BMU_CTRL);

	/* Configure analog GPIO setup */
	rt_sysc_m32(MT7628_EPHY_P0_DIS, MT7628_EPHY_GPIO_AIO_EN,
		    MT7628_AGPIO_CFG_REG);

	/* Reset PHY */
	rt_sysc_m32(0, RSTCTRL_EPHY_RST, MT7628_RSTCTRL_REG);
	rt_sysc_m32(RSTCTRL_EPHY_RST, 0, MT7628_RSTCTRL_REG);
	mdelay(10);

	/* Set P0 EPHY LED mode */
	rt_sysc_m32(0x0ffc0ffc, 0x05540554, MT7628_GPIO2_MODE_REG);
	mdelay(10);

	mt7628_ephy_init(priv);
}

static int mt7628_phy_read(struct dsa_switch *ds, int port, int regnum)
{
	struct mt7628_esw_priv *priv = ds->priv;
	u32 val;
	int ret;

	ret = mii_mgr_read(priv, port, regnum, &val);
	// test-only: ret error handling

	return val;
}

static int mt7628_phy_write(struct dsa_switch *ds, int port, int regnum,
			    u16 val)
{
	struct mt7628_esw_priv *priv = ds->priv;

	return mii_mgr_write(priv, port, regnum, val);
}

static const struct dsa_switch_ops mt7628_switch_ops = {
//	.get_tag_protocol	= mtk_get_tag_protocol,
	.setup			= mt7628_setup,
	.phy_read		= mt7628_phy_read,
	.phy_write		= mt7628_phy_write,
#if 0
	.get_strings		= mt7628_get_strings,
//	.phy_read		= mt7628_phy_read,
//	.phy_write		= mt7628_phy_write,
	.get_ethtool_stats	= mt7628_get_ethtool_stats,
	.get_sset_count		= mt7628_get_sset_count,
	.adjust_link		= mt7628_adjust_link,
	.port_enable		= mt7628_port_enable,
	.port_disable		= mt7628_port_disable,
	.port_stp_state_set	= mt7628_stp_state_set,
	.port_bridge_join	= mt7628_port_bridge_join,
	.port_bridge_leave	= mt7628_port_bridge_leave,
	.port_fdb_add		= mt7628_port_fdb_add,
	.port_fdb_del		= mt7628_port_fdb_del,
	.port_fdb_dump		= mt7628_port_fdb_dump,
	.port_vlan_filtering	= mt7628_port_vlan_filtering,
	.port_vlan_prepare	= mt7628_port_vlan_prepare,
	.port_vlan_add		= mt7628_port_vlan_add,
	.port_vlan_del		= mt7628_port_vlan_del,
#endif
};

#if 1
//static int mt7628_probe(struct mdio_device *mdiodev)
static int mt7628_probe(struct platform_device *pdev)
{
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
//	struct device_node *np = pdev->dev.of_node;
	struct mt7628_esw_priv *priv;
	int ret;

	printk("%s (%d)!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", __func__, __LINE__); // test-only

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->base = devm_ioremap_resource(&pdev->dev, res);
	printk("%s (%d): base=%pF\n", __func__, __LINE__, priv->base); // test-only
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	ret = device_reset(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "switch reset failed!\n");
		return ret;
	}

	priv->ds = dsa_switch_alloc(&pdev->dev, DSA_MAX_PORTS);
	if (!priv->ds)
		return -ENOMEM;

	// test-only: name and return code
	rt305x_esw_init(priv);

	priv->ds->priv = priv;
	priv->ds->dev = &pdev->dev;
	priv->ds->ops = &mt7628_switch_ops;
	dev_set_drvdata(&pdev->dev, priv->ds);

	return dsa_register_switch(priv->ds);
#if 0
	struct mt7628_priv *priv;
	struct device_node *dn;

	dn = mdiodev->dev.of_node;

	priv = devm_kzalloc(&mdiodev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->ds = dsa_switch_alloc(&mdiodev->dev, DSA_MAX_PORTS);
	if (!priv->ds)
		return -ENOMEM;

	/* Use medatek,mcm property to distinguish hardware type that would
	 * casues a little bit differences on power-on sequence.
	 */
	priv->mcm = of_property_read_bool(dn, "mediatek,mcm");
	if (priv->mcm) {
		dev_info(&mdiodev->dev, "MT7628 adapts as multi-chip module\n");

		priv->rstc = devm_reset_control_get(&mdiodev->dev, "mcm");
		if (IS_ERR(priv->rstc)) {
			dev_err(&mdiodev->dev, "Couldn't get our reset line\n");
			return PTR_ERR(priv->rstc);
		}
	}

	/* Get the hardware identifier from the devicetree node.
	 * We will need it for some of the clock and regulator setup.
	 */
	priv->id = (unsigned int)(unsigned long)
		of_device_get_match_data(&mdiodev->dev);

	if (priv->id == ID_MT7628) {
		priv->core_pwr = devm_regulator_get(&mdiodev->dev, "core");
		if (IS_ERR(priv->core_pwr))
			return PTR_ERR(priv->core_pwr);

		priv->io_pwr = devm_regulator_get(&mdiodev->dev, "io");
		if (IS_ERR(priv->io_pwr))
			return PTR_ERR(priv->io_pwr);
	}

	/* Not MCM that indicates switch works as the remote standalone
	 * integrated circuit so the GPIO pin would be used to complete
	 * the reset, otherwise memory-mapped register accessing used
	 * through syscon provides in the case of MCM.
	 */
	if (!priv->mcm) {
		priv->reset = devm_gpiod_get_optional(&mdiodev->dev, "reset",
						      GPIOD_OUT_LOW);
		if (IS_ERR(priv->reset)) {
			dev_err(&mdiodev->dev, "Couldn't get our reset line\n");
			return PTR_ERR(priv->reset);
		}
	}

	priv->bus = mdiodev->bus;
	priv->dev = &mdiodev->dev;
	priv->ds->priv = priv;
	priv->ds->ops = &mt7628_switch_ops;
	mutex_init(&priv->reg_mutex);
	dev_set_drvdata(&mdiodev->dev, priv);

	return dsa_register_switch(priv->ds);
#endif
}

//static void mt7628_remove(struct mdio_device *mdiodev)
static int mt7628_remove(struct platform_device *pdev)
{
	struct mt7628_esw_priv *priv = dev_get_drvdata(&pdev->dev);

	printk("%s (%d)\n", __func__, __LINE__); // test-only
#if 0
	struct mt7628_priv *priv = dev_get_drvdata(&mdiodev->dev);
	int ret = 0;

	ret = regulator_disable(priv->core_pwr);
	if (ret < 0)
		dev_err(priv->dev,
			"Failed to disable core power: %d\n", ret);

	ret = regulator_disable(priv->io_pwr);
	if (ret < 0)
		dev_err(priv->dev, "Failed to disable io pwr: %d\n",
			ret);

	dsa_unregister_switch(priv->ds);
	mutex_destroy(&priv->reg_mutex);
#else
	dsa_unregister_switch(priv->ds);

	return 0;
#endif
}

static const struct of_device_id mt7628_of_match[] = {
	{ .compatible = "mediatek,mt7628-esw" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, mt7628_of_match);

//static struct mdio_driver mt7628_mdio_driver = {
static struct platform_driver mt7628_mdio_driver = {
	.probe  = mt7628_probe,
	.remove = mt7628_remove,
//	.mdiodrv.driver = {
	.driver = {
		.name = "mt7628-esw",
		.of_match_table = mt7628_of_match,
	},
};

//mdio_module_driver(mt7628_mdio_driver);
module_platform_driver(mt7628_mdio_driver);
#endif

MODULE_AUTHOR("Stefan Roese <sr@denx.de>");
MODULE_DESCRIPTION("Driver for Mediatek MT7628 Switch (ESW)");
MODULE_LICENSE("GPL");
