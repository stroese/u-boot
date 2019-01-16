// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe driver for Marvell MVEBU SoCs
 *
 * Based on Barebox drivers/pci/pci-mvebu.c
 *
 * Ported to U-Boot by:
 * Anton Schubert <anton.schubert@gmx.de>
 * Stefan Roese <sr@denx.de>
 */

#include <common.h>
#include <dm.h>
#include <dm/device-internal.h>
#include <dm/lists.h>
#include <pci.h>
#include <asm/io.h>
#include <asm/arch/cpu.h>
#include <asm/arch/soc.h>
#include <linux/errno.h>
#include <linux/mbus.h>

DECLARE_GLOBAL_DATA_PTR;

/* PCIe unit register offsets */
#define SELECT(x, n)			((x >> n) & 1UL)

#define PCIE_DEV_ID_OFF			0x0000
#define PCIE_CMD_OFF			0x0004
#define PCIE_DEV_REV_OFF		0x0008
#define  PCIE_BAR_LO_OFF(n)		(0x0010 + ((n) << 3))
#define  PCIE_BAR_HI_OFF(n)		(0x0014 + ((n) << 3))
#define PCIE_CAPAB_OFF			0x0060
#define PCIE_CTRL_STAT_OFF		0x0068
#define PCIE_HEADER_LOG_4_OFF		0x0128
#define  PCIE_BAR_CTRL_OFF(n)		(0x1804 + (((n) - 1) * 4))
#define  PCIE_WIN04_CTRL_OFF(n)		(0x1820 + ((n) << 4))
#define  PCIE_WIN04_BASE_OFF(n)		(0x1824 + ((n) << 4))
#define  PCIE_WIN04_REMAP_OFF(n)	(0x182c + ((n) << 4))
#define PCIE_WIN5_CTRL_OFF		0x1880
#define PCIE_WIN5_BASE_OFF		0x1884
#define PCIE_WIN5_REMAP_OFF		0x188c
#define PCIE_CONF_ADDR_OFF		0x18f8
#define  PCIE_CONF_ADDR_EN		BIT(31)
#define  PCIE_CONF_REG(r)		((((r) & 0xf00) << 16) | ((r) & 0xfc))
#define  PCIE_CONF_BUS(b)		(((b) & 0xff) << 16)
#define  PCIE_CONF_DEV(d)		(((d) & 0x1f) << 11)
#define  PCIE_CONF_FUNC(f)		(((f) & 0x7) << 8)
#define  PCIE_CONF_ADDR(dev, reg) \
	(PCIE_CONF_BUS(PCI_BUS(dev)) | PCIE_CONF_DEV(PCI_DEV(dev))    | \
	 PCIE_CONF_FUNC(PCI_FUNC(dev)) | PCIE_CONF_REG(reg) | \
	 PCIE_CONF_ADDR_EN)
#define PCIE_CONF_DATA_OFF		0x18fc
#define PCIE_MASK_OFF			0x1910
#define  PCIE_MASK_ENABLE_INTS          (0xf << 24)
#define PCIE_CTRL_OFF			0x1a00
#define  PCIE_CTRL_X1_MODE		BIT(0)
#define PCIE_STAT_OFF			0x1a04
#define  PCIE_STAT_BUS                  (0xff << 8)
#define  PCIE_STAT_DEV                  (0x1f << 16)
#define  PCIE_STAT_LINK_DOWN		BIT(0)
#define PCIE_DEBUG_CTRL			0x1a60
#define  PCIE_DEBUG_SOFT_RESET		BIT(20)

struct resource {
	u32 start;
	u32 end;
};

struct mvebu_pcie {
	struct pci_controller hose;
	void __iomem *base;
	void __iomem *membase;
	struct resource mem;
	void __iomem *iobase;
	u32 port;
	u32 lane;
	u32 lane_mask;
	pci_dev_t dev;
	char name[16];
	int mem_target;
	int mem_attr;
};

/*
 * MVEBU PCIe controller needs MEMORY and I/O BARs to be mapped
 * into SoCs address space. Each controller will map 128M of MEM
 * and 64K of I/O space when registered.
 */
static void __iomem *mvebu_pcie_membase = (void __iomem *)MBUS_PCI_MEM_BASE;
#define PCIE_MEM_SIZE	(128 << 20)

#if defined(CONFIG_ARMADA_38X)
/*
 * On A38x MV6820 these PEX ports are supported:
 *  0 - Port 0.0
 *  1 - Port 1.0
 *  2 - Port 2.0
 *  3 - Port 3.0
 */
#define MAX_PEX 4

static void mvebu_get_port_lane(struct mvebu_pcie *pcie, int pex_idx,
				int *mem_target, int *mem_attr)
{
	u8 port[] = { 0, 1, 2, 3 };
	u8 lane[] = { 0, 0, 0, 0 };
	u8 target[] = { 8, 4, 4, 4 };
	u8 attr[] = { 0xe8, 0xe8, 0xd8, 0xb8 };

	pcie->port = port[pex_idx];
	pcie->lane = lane[pex_idx];
	*mem_target = target[pex_idx];
	*mem_attr = attr[pex_idx];
}
#else
/*
 * On AXP MV78460 these PEX ports are supported:
 *  0 - Port 0.0
 *  1 - Port 0.1
 *  2 - Port 0.2
 *  3 - Port 0.3
 *  4 - Port 1.0
 *  5 - Port 1.1
 *  6 - Port 1.2
 *  7 - Port 1.3
 *  8 - Port 2.0
 *  9 - Port 3.0
 */
#define MAX_PEX 10

static void mvebu_get_port_lane(struct mvebu_pcie *pcie, int pex_idx,
				int *mem_target, int *mem_attr)
{
	u8 port[] = { 0, 0, 0, 0, 1, 1, 1, 1, 2, 3 };
	u8 lane[] = { 0, 1, 2, 3, 0, 1, 2, 3, 0, 0 };
	u8 target[] = { 4, 4, 4, 4, 8, 8, 8, 8, 4, 8 };
	u8 attr[] = { 0xe8, 0xd8, 0xb8, 0x78,
		      0xe8, 0xd8, 0xb8, 0x78,
		      0xf8, 0xf8 };

	pcie->port = port[pex_idx];
	pcie->lane = lane[pex_idx];
	*mem_target = target[pex_idx];
	*mem_attr = attr[pex_idx];
}
#endif

struct mvebu_pcie_base {
	struct mvebu_pcie pcie[MAX_PEX];
};

static int mvebu_pex_unit_is_x4(int pex_idx)
{
	int pex_unit = pex_idx < 9 ? pex_idx >> 2 : 3;
	u32 mask = (0x0f << (pex_unit * 8));

	return (readl(COMPHY_REFCLK_ALIGNMENT) & mask) == mask;
}

static inline bool mvebu_pcie_link_up(struct mvebu_pcie *pcie)
{
	u32 val;
	val = readl(pcie->base + PCIE_STAT_OFF);
	return !(val & PCIE_STAT_LINK_DOWN);
}

static void mvebu_pcie_set_local_bus_nr(struct mvebu_pcie *pcie, int busno)
{
	u32 stat;

	stat = readl(pcie->base + PCIE_STAT_OFF);
	stat &= ~PCIE_STAT_BUS;
	stat |= busno << 8;
	writel(stat, pcie->base + PCIE_STAT_OFF);
}

static void mvebu_pcie_set_local_dev_nr(struct mvebu_pcie *pcie, int devno)
{
	u32 stat;

	stat = readl(pcie->base + PCIE_STAT_OFF);
	stat &= ~PCIE_STAT_DEV;
	stat |= devno << 16;
	writel(stat, pcie->base + PCIE_STAT_OFF);
}

static int mvebu_pcie_get_local_bus_nr(struct mvebu_pcie *pcie)
{
	u32 stat;

	stat = readl(pcie->base + PCIE_STAT_OFF);
	return (stat & PCIE_STAT_BUS) >> 8;
}

static int mvebu_pcie_get_local_dev_nr(struct mvebu_pcie *pcie)
{
	u32 stat;

	stat = readl(pcie->base + PCIE_STAT_OFF);
	return (stat & PCIE_STAT_DEV) >> 16;
}

static inline struct mvebu_pcie *hose_to_pcie(struct pci_controller *hose)
{
	return container_of(hose, struct mvebu_pcie, hose);
}

static int mvebu_pcie_read_config(struct udevice *bus, pci_dev_t bdf,
				  uint offset, ulong *valuep,
				  enum pci_size_t size)
{
	struct mvebu_pcie *pcie = dev_get_platdata(bus);
	int local_bus = PCI_BUS(pcie->dev);
	int local_dev = PCI_DEV(pcie->dev);
	u32 reg;
	u32 data;

	debug("PCIE CFG read:  (b,d,f)=(%2d,%2d,%2d) ",
	      PCI_BUS(bdf), PCI_DEV(bdf), PCI_FUNC(bdf));

	/* Only allow one other device besides the local one on the local bus */
	if (PCI_BUS(bdf) == local_bus && PCI_DEV(bdf) != local_dev) {
		if (local_dev == 0 && PCI_DEV(bdf) != 1) {
			debug("- out of range\n");
			/*
			 * If local dev is 0, the first other dev can
			 * only be 1
			 */
			*valuep = pci_get_ff(size);
			return 0;
		} else if (local_dev != 0 && PCI_DEV(bdf) != 0) {
			debug("- out of range\n");
			/*
			 * If local dev is not 0, the first other dev can
			 * only be 0
			 */
			*valuep = pci_get_ff(size);
			return 0;
		}
	}

	/* write address */
	reg = PCIE_CONF_ADDR(bdf, offset);
	writel(reg, pcie->base + PCIE_CONF_ADDR_OFF);
	data = readl(pcie->base + PCIE_CONF_DATA_OFF);
	debug("(addr,val)=(0x%04x, 0x%08x)\n", offset, data);
	*valuep = pci_conv_32_to_size(data, offset, size);

	return 0;
}

static int mvebu_pcie_write_config(struct udevice *bus, pci_dev_t bdf,
				   uint offset, ulong value,
				   enum pci_size_t size)
{
	struct mvebu_pcie *pcie = dev_get_platdata(bus);
	int local_bus = PCI_BUS(pcie->dev);
	int local_dev = PCI_DEV(pcie->dev);
	u32 data;

	debug("PCIE CFG write: (b,d,f)=(%2d,%2d,%2d) ",
	      PCI_BUS(bdf), PCI_DEV(bdf), PCI_FUNC(bdf));
	debug("(addr,val)=(0x%04x, 0x%08lx)\n", offset, value);

	/* Only allow one other device besides the local one on the local bus */
	if (PCI_BUS(bdf) == local_bus && PCI_DEV(bdf) != local_dev) {
		if (local_dev == 0 && PCI_DEV(bdf) != 1) {
			/*
			 * If local dev is 0, the first other dev can
			 * only be 1
			 */
			return 0;
		} else if (local_dev != 0 && PCI_DEV(bdf) != 0) {
			/*
			 * If local dev is not 0, the first other dev can
			 * only be 0
			 */
			return 0;
		}
	}

	writel(PCIE_CONF_ADDR(bdf, offset), pcie->base + PCIE_CONF_ADDR_OFF);
	data = pci_conv_size_to_32(0, value, offset, size);
	writel(data, pcie->base + PCIE_CONF_DATA_OFF);

	return 0;
}

/*
 * Setup PCIE BARs and Address Decode Wins:
 * BAR[0,2] -> disabled, BAR[1] -> covers all DRAM banks
 * WIN[0-3] -> DRAM bank[0-3]
 */
static void mvebu_pcie_setup_wins(struct mvebu_pcie *pcie)
{
	const struct mbus_dram_target_info *dram = mvebu_mbus_dram_info();
	u32 size;
	int i;

	/* First, disable and clear BARs and windows. */
	for (i = 1; i < 3; i++) {
		writel(0, pcie->base + PCIE_BAR_CTRL_OFF(i));
		writel(0, pcie->base + PCIE_BAR_LO_OFF(i));
		writel(0, pcie->base + PCIE_BAR_HI_OFF(i));
	}

	for (i = 0; i < 5; i++) {
		writel(0, pcie->base + PCIE_WIN04_CTRL_OFF(i));
		writel(0, pcie->base + PCIE_WIN04_BASE_OFF(i));
		writel(0, pcie->base + PCIE_WIN04_REMAP_OFF(i));
	}

	writel(0, pcie->base + PCIE_WIN5_CTRL_OFF);
	writel(0, pcie->base + PCIE_WIN5_BASE_OFF);
	writel(0, pcie->base + PCIE_WIN5_REMAP_OFF);

	/* Setup windows for DDR banks. Count total DDR size on the fly. */
	size = 0;
	for (i = 0; i < dram->num_cs; i++) {
		const struct mbus_dram_window *cs = dram->cs + i;

		writel(cs->base & 0xffff0000,
		       pcie->base + PCIE_WIN04_BASE_OFF(i));
		writel(0, pcie->base + PCIE_WIN04_REMAP_OFF(i));
		writel(((cs->size - 1) & 0xffff0000) |
		       (cs->mbus_attr << 8) |
		       (dram->mbus_dram_target_id << 4) | 1,
		       pcie->base + PCIE_WIN04_CTRL_OFF(i));

		size += cs->size;
	}

	/* Round up 'size' to the nearest power of two. */
	if ((size & (size - 1)) != 0)
		size = 1 << fls(size);

	/* Setup BAR[1] to all DRAM banks. */
	writel(dram->cs[0].base | 0xc, pcie->base + PCIE_BAR_LO_OFF(1));
	writel(0, pcie->base + PCIE_BAR_HI_OFF(1));
	writel(((size - 1) & 0xffff0000) | 0x1,
	       pcie->base + PCIE_BAR_CTRL_OFF(1));
}

/**
 * pcie_dw_mvebu_probe() - Probe the PCIe bus for active link
 *
 * @dev: A pointer to the device being operated on
 *
 * Probe for an active link on the PCIe bus and configure the controller
 * to enable this port.
 *
 * Return: 0 on success, else -ENODEV
 */
static int mvebu_pcie_probe(struct udevice *dev)
{
	struct mvebu_pcie *pcie = dev_get_platdata(dev);
	struct udevice *ctlr = pci_get_controller(dev);
	struct pci_controller *hose = dev_get_uclass_priv(ctlr);
	static int bus;
	u32 reg;

	debug("%s: PCIe %d.%d - up, base %08x\n", __func__,
	      pcie->port, pcie->lane, (u32)pcie->base);

	/* Read Id info and local bus/dev */
	debug("direct conf read %08x, local bus %d, local dev %d\n",
	      readl(pcie->base), mvebu_pcie_get_local_bus_nr(pcie),
	      mvebu_pcie_get_local_dev_nr(pcie));

	mvebu_pcie_set_local_bus_nr(pcie, bus);
	mvebu_pcie_set_local_dev_nr(pcie, 0);
	pcie->dev = PCI_BDF(bus, 0, 0);

	pcie->mem.start = (u32)mvebu_pcie_membase;
	pcie->mem.end = pcie->mem.start + PCIE_MEM_SIZE - 1;
	mvebu_pcie_membase += PCIE_MEM_SIZE;

	if (mvebu_mbus_add_window_by_id(pcie->mem_target, pcie->mem_attr,
					(phys_addr_t)pcie->mem.start,
					PCIE_MEM_SIZE)) {
		printf("PCIe unable to add mbus window for mem at %08x+%08x\n",
		       (u32)pcie->mem.start, PCIE_MEM_SIZE);
	}

	/* Setup windows and configure host bridge */
	mvebu_pcie_setup_wins(pcie);

	/* Master + slave enable. */
	reg = readl(pcie->base + PCIE_CMD_OFF);
	reg |= PCI_COMMAND_MEMORY;
	reg |= PCI_COMMAND_MASTER;
	reg |= BIT(10);		/* disable interrupts */
	writel(reg, pcie->base + PCIE_CMD_OFF);

	/* Set BAR0 to internal registers */
	writel(SOC_REGS_PHY_BASE, pcie->base + PCIE_BAR_LO_OFF(0));
	writel(0, pcie->base + PCIE_BAR_HI_OFF(0));

	/* PCI memory space */
	pci_set_region(hose->regions + 0, pcie->mem.start,
		       pcie->mem.start, PCIE_MEM_SIZE, PCI_REGION_MEM);
	pci_set_region(hose->regions + 1,
		       0, 0,
		       gd->ram_size,
		       PCI_REGION_MEM | PCI_REGION_SYS_MEMORY);
	hose->region_count = 2;

	bus++;

	return 0;
}

static const struct dm_pci_ops mvebu_pcie_ops = {
	.read_config	= mvebu_pcie_read_config,
	.write_config	= mvebu_pcie_write_config,
};

static struct driver pcie_mvebu_drv = {
	.name			= "pcie_mvebu",
	.id			= UCLASS_PCI,
	.ops			= &mvebu_pcie_ops,
	.probe			= mvebu_pcie_probe,
	.platdata_auto_alloc_size = sizeof(struct mvebu_pcie),
};

static int mvebu_pcie_port_parse_dt(ofnode node, struct mvebu_pcie *pcie)
{
	const u32 *addr;
	int len;

	addr = ofnode_get_property(node, "assigned-addresses", &len);
	if (!addr) {
		pr_err("property \"assigned-addresses\" not found");
		return -FDT_ERR_NOTFOUND;
	}

	pcie->base = (void *)(fdt32_to_cpu(addr[2]) + SOC_REGS_PHY_BASE);

	return 0;
}

/*
 * Use a MISC device to bind the n instances (child nodes) of the
 * PCIe base controller in UCLASS_PCI.
 */
static int mvebu_pcie_bind(struct udevice *parent)
{
	struct mvebu_pcie *pcie;
	struct uclass_driver *drv;
	struct udevice *dev;
	ofnode subnode;
	u32 soc_ctrl;
	int port = 0;
	int i = 0;
	int ret;

	soc_ctrl = readl(MVEBU_SYSTEM_REG_BASE + 0x4);

	/* Lookup eth driver */
	drv = lists_uclass_lookup(UCLASS_PCI);
	if (!drv) {
		puts("Cannot find PCI driver\n");
		return -ENOENT;
	}

	ofnode_for_each_subnode(subnode, dev_ofnode(parent)) {
		pcie = calloc(1, sizeof(*pcie));
		if (!pcie)
			return -ENOMEM;

		/* Get port number, lane number and memory target / attr */
		mvebu_get_port_lane(pcie, i,
				    &pcie->mem_target, &pcie->mem_attr);

		/* Don't read at all from pci registers if port power is down */
		if (SELECT(soc_ctrl, pcie->port) == 0) {
			if (pcie->lane == 0)
				debug("%s: skipping port %d\n", __func__,
				      pcie->port);
			i++;
			free(pcie);
			continue;
		}

		/* Parse PCIe controller register base from DT */
		ret = mvebu_pcie_port_parse_dt(subnode, pcie);
		if (ret < 0) {
			i++;
			free(pcie);
			continue;
		}

		/* Check link and skip ports that have no link */
		if (!mvebu_pcie_link_up(pcie)) {
			debug("%s: PCIe %d.%d - down\n", __func__,
			      pcie->port, pcie->lane);

			i++;
			free(pcie);
			continue;
		}

		sprintf(pcie->name, "pcie%d.%d", pcie->port, pcie->lane);

		/* Create child device UCLASS_PCI and bind it */
		device_bind_ofnode(parent, &pcie_mvebu_drv, pcie->name, pcie,
				   subnode, &dev);

		/* need to skip more for X4 links, otherwise scan will hang */
		if (mvebu_soc_family() == MVEBU_SOC_AXP) {
			if (mvebu_pex_unit_is_x4(i)) {
				subnode = ofnode_next_subnode(subnode);
				subnode = ofnode_next_subnode(subnode);
				subnode = ofnode_next_subnode(subnode);
				i += 3;
			}
		}

		i++;
		port++;
	}

	return 0;
}

static const struct udevice_id mvebu_pcie_ids[] = {
	{ .compatible = "marvell,armada-xp-pcie" },
	{ .compatible = "marvell,armada-370-pcie" },
	{ }
};

U_BOOT_DRIVER(pcie_mvebu_base) = {
	.name			= "pcie_mvebu_base",
	.id			= UCLASS_MISC,
	.of_match		= mvebu_pcie_ids,
	.bind			= mvebu_pcie_bind,
};
