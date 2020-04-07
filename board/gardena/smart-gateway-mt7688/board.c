// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018 Stefan Roese <sr@denx.de>
 */

#define DEBUG
#include <common.h>
#include <env.h>
#include <env_internal.h>
#include <init.h>
#include <led.h>
#include <malloc.h>
#include <net.h>
#include <spi.h>
#include <spi_flash.h>
#include <u-boot/crc.h>
#include <uuid.h>
#include <linux/ctype.h>
#include <linux/io.h>

#define MT76XX_AGPIO_CFG	0x1000003c

#define FACTORY_DATA_OFFS	0xc0000
#define FACTORY_DATA_SECT_SIZE	0x10000
#if ((CONFIG_ENV_OFFSET_REDUND + CONFIG_ENV_SIZE) > FACTORY_DATA_OFFS)
#error "U-Boot image with environment too big (overlapping with factory-data)!"
#endif
#define FACTORY_DATA_USER_OFFS	0x140
#define FACTORY_DATA_SIZE	0x1f0
#define FACTORY_DATA_CRC_LEN	(FACTORY_DATA_SIZE -			\
				 FACTORY_DATA_USER_OFFS - sizeof(u32))

#define FACTORY_DATA_MAGIC	0xCAFEBABE

struct factory_data_values {
	u8 pad_1[4];
	u8 wifi_mac[6];		/* offs: 0x004: binary value */
	u8 pad_2[30];
	u8 eth_mac[6];		/* offs: 0x028: binary value */
	u8 pad_3[FACTORY_DATA_USER_OFFS - 4 - 6 - 30 - 6];
	/* User values start here at offset 0x140 */
	u32 crc;
	u32 magic;
	u32 version;
	char ipr_id[UUID_STR_LEN];	/* UUID as string w/o ending \0 */
	char hqv_id[UUID_STR_LEN];	/* UUID as string w/o ending \0 */
	char unielec_id[UUID_STR_LEN];	/* UUID as string w/o ending \0 */
};

int board_early_init_f(void)
{
	void __iomem *gpio_mode;

	/* Configure digital vs analog GPIOs */
	gpio_mode = ioremap_nocache(MT76XX_AGPIO_CFG, 0x100);
	iowrite32(0x00fe01ff, gpio_mode);

	return 0;
}

static bool prepare_uuid_var(const char *fd_ptr, const char *env_var_name,
			     char errorchar)
{
	char str[UUID_STR_LEN + 1] = { 0 };	/* Enough for UUID stuff */
	bool env_updated = false;
	char *env;
	int i;

	memcpy(str, fd_ptr, UUID_STR_LEN);

	/* Convert non-ascii character to 'X' */
	for (i = 0; i < UUID_STR_LEN; i++) {
		if (!(isascii(str[i]) && isprint(str[i])))
			str[i] = errorchar;
	}

	env = env_get(env_var_name);
	if (strcmp(env, str)) {
		env_set(env_var_name, str);
		env_updated = true;
	}

	return env_updated;
}

static void factory_data_env_config(void)
{
	struct factory_data_values *fd;
	struct spi_flash *sf;
	int env_updated = 0;
	char str[UUID_STR_LEN + 1];	/* Enough for UUID stuff */
	char *env;
	u8 *buf;
	u32 crc;
	int ret;
	u8 *ptr;

	buf = malloc(FACTORY_DATA_SIZE);
	if (!buf) {
		printf("F-Data:Unable to allocate buffer\n");
		return;
	}

	/*
	 * Get values from factory-data area in SPI NOR
	 */
	sf = spi_flash_probe(CONFIG_SF_DEFAULT_BUS,
			     CONFIG_SF_DEFAULT_CS,
			     CONFIG_SF_DEFAULT_SPEED,
			     CONFIG_SF_DEFAULT_MODE);
	if (!sf) {
		printf("F-Data:Unable to access SPI NOR flash\n");
		goto err_free;
	}

	ret = spi_flash_read(sf, FACTORY_DATA_OFFS, FACTORY_DATA_SIZE,
			     (void *)buf);
	if (ret) {
		printf("F-Data:Unable to read factory-data from SPI NOR\n");
		goto err_spi_flash;
	}

	fd = (struct factory_data_values *)buf;

	if (fd->magic != FACTORY_DATA_MAGIC)
		printf("F-Data:Magic value not correct\n");

	crc = crc32(0, (u8 *)&fd->magic, FACTORY_DATA_CRC_LEN);
	if (crc != fd->crc)
		printf("F-Data:CRC not correct\n");
	else
		printf("F-Data:factory-data version %x detected\n",
		       fd->version);

	/* Handle wifi_mac env variable */
	ptr = fd->wifi_mac;
	sprintf(str, "%pM", ptr);
	if (!is_valid_ethaddr(ptr))
		printf("F-Data:Invalid MAC addr: wifi_mac %s\n", str);

	env = env_get("wifiaddr");
	if (strcmp(env, str)) {
		env_set("wifiaddr", str);
		env_updated = 1;
	}

	/* Handle eth_mac env variable */
	ptr = fd->eth_mac;
	sprintf(str, "%pM", ptr);
	if (!is_valid_ethaddr(ptr))
		printf("F-Data:Invalid MAC addr: eth_mac %s\n", str);

	env = env_get("ethaddr");
	if (strcmp(env, str)) {
		env_set("ethaddr", str);
		env_updated = 1;
	}

	/* Handle UUID env variables */
	env_updated |= prepare_uuid_var(fd->ipr_id, "linuxmoduleid", 'X');
	env_updated |= prepare_uuid_var(fd->hqv_id, "linuxmodulehqvid", '\0');
	env_updated |= prepare_uuid_var(fd->unielec_id,
					"linuxmoduleunielecid", '\0');

	/* Check if the environment was updated and needs to get stored */
	if (env_updated != 0) {
		printf("F-Data:Values don't match env values -> saving\n");
		env_save();
	} else {
		debug("F-Data:Values match current env values\n");
	}

err_spi_flash:
	spi_flash_free(sf);

err_free:
	free(buf);
}

int board_late_init(void)
{
	if (IS_ENABLED(CONFIG_LED))
		led_default_state();

	factory_data_env_config();

	return 0;
}

static void copy_or_generate_uuid(char *fd_ptr, const char *env_var_name)
{
	char str[UUID_STR_LEN + 1] = { 0 };	/* Enough for UUID stuff */
	char *env;

	/* Don't use the UUID dest place, as the \0 char won't fit */
	env = env_get(env_var_name);
	if (env)
		strncpy(str, env, UUID_STR_LEN);
	else
		gen_rand_uuid_str(str, UUID_STR_FORMAT_STD);

	memcpy(fd_ptr, str, UUID_STR_LEN);
}

/*
 * Helper function to provide some sane factory-data values for testing
 * purpose, when these values are not programmed correctly
 */
int do_fd_write(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	struct factory_data_values *fd;
	struct spi_flash *sf;
	u8 *buf;
	int ret = CMD_RET_FAILURE;

	buf = malloc(FACTORY_DATA_SECT_SIZE);
	if (!buf) {
		printf("F-Data:Unable to allocate buffer\n");
		return CMD_RET_FAILURE;
	}

	sf = spi_flash_probe(CONFIG_SF_DEFAULT_BUS,
			     CONFIG_SF_DEFAULT_CS,
			     CONFIG_SF_DEFAULT_SPEED,
			     CONFIG_SF_DEFAULT_MODE);
	if (!sf) {
		printf("F-Data:Unable to access SPI NOR flash\n");
		goto err_free;
	}

	/* Generate the factory-data struct */

	/* Fist read complete sector into buffer */
	ret = spi_flash_read(sf, FACTORY_DATA_OFFS, FACTORY_DATA_SECT_SIZE,
			     (void *)buf);
	if (ret) {
		printf("F-Data:spi_flash_read failed (%d)\n", ret);
		goto err_spi_flash;
	}

	fd = (struct factory_data_values *)buf;
	fd->magic = FACTORY_DATA_MAGIC;
	fd->version = 0x1;

	/* Use existing MAC and UUID values or generate some random ones */
	if (!eth_env_get_enetaddr("wifiaddr", fd->wifi_mac)) {
		net_random_ethaddr(fd->wifi_mac);
		/* to get a different seed value for the MAC address */
		mdelay(10);
	}

	if (!eth_env_get_enetaddr("ethaddr", fd->eth_mac))
		net_random_ethaddr(fd->eth_mac);

	copy_or_generate_uuid(fd->ipr_id, "linuxmoduleid");
	copy_or_generate_uuid(fd->hqv_id, "linuxmodulehqvid");
	copy_or_generate_uuid(fd->unielec_id, "linuxmoduleunielecid");

	printf("New factory-data values:\n");
	printf("wifiaddr=%pM\n", fd->wifi_mac);
	printf("ethaddr=%pM\n", fd->eth_mac);

	/*
	 * We don't have the \0 char at the end, so we need to specify the
	 * length in the printf format instead
	 */
	printf("linuxmoduleid=%." __stringify(UUID_STR_LEN) "s\n", fd->ipr_id);
	printf("linuxmodulehqvid=%." __stringify(UUID_STR_LEN) "s\n",
	       fd->hqv_id);
	printf("linuxmoduleunielecid=%." __stringify(UUID_STR_LEN) "s\n",
	       fd->unielec_id);

	fd->crc = crc32(0, (u8 *)&fd->magic, FACTORY_DATA_CRC_LEN);

	ret = spi_flash_erase(sf, FACTORY_DATA_OFFS, FACTORY_DATA_SECT_SIZE);
	if (ret) {
		printf("F-Data:spi_flash_erase failed (%d)\n", ret);
		goto err_spi_flash;
	}

	ret = spi_flash_write(sf, FACTORY_DATA_OFFS, FACTORY_DATA_SECT_SIZE,
			      buf);
	if (ret) {
		printf("F-Data:spi_flash_write failed (%d)\n", ret);
		goto err_spi_flash;
	}

	printf("F-Data:factory-data values written to SPI NOR flash\n");

err_spi_flash:
	spi_flash_free(sf);

err_free:
	free(buf);

	return ret;
}

#ifndef CONFIG_SPL_BUILD
U_BOOT_CMD(
	fd_write,	1,	0,	do_fd_write,
	"Write test factory-data values to SPI NOR",
	"\n"
);
#endif





#ifndef CONFIG_SPL_BUILD



#include <cpu_func.h>
#include <spl.h>
#if IS_ENABLED(CONFIG_SPL_LZMA)
#include <lzma/LzmaTypes.h>
#include <lzma/LzmaDec.h>
#include <lzma/LzmaTools.h>
#endif

#ifndef CONFIG_SYS_BOOTM_LEN
#define CONFIG_SYS_BOOTM_LEN	(8 << 20)
#endif

#if 0
#ifndef CONFIG_SYS_UBOOT_START
#define CONFIG_SYS_UBOOT_START	CONFIG_SYS_TEXT_BASE
#endif
#ifndef CONFIG_SYS_MONITOR_LEN
/* Unknown U-Boot size, let's assume it will not be more than 200 KB */
#define CONFIG_SYS_MONITOR_LEN	(200 * 1024)
#endif

void spl_set_header_raw_uboot(struct spl_image_info *spl_image)
{
	ulong u_boot_pos = binman_sym(ulong, u_boot_any, image_pos);

	spl_image->size = CONFIG_SYS_MONITOR_LEN;

	/*
	 * Binman error cases: address of the end of the previous region or the
	 * start of the image's entry area (usually 0) if there is no previous
	 * region.
	 */
	if (u_boot_pos && u_boot_pos != BINMAN_SYM_MISSING) {
		/* Binman does not support separated entry addresses */
		spl_image->entry_point = u_boot_pos;
		spl_image->load_addr = u_boot_pos;
	} else {
		spl_image->entry_point = CONFIG_SYS_UBOOT_START;
		spl_image->load_addr = CONFIG_SYS_TEXT_BASE;
	}
	spl_image->os = IH_OS_U_BOOT;
	spl_image->name = "U-Boot";
}
#endif

int spl_parse_image_header(struct spl_image_info *spl_image,
			   const struct image_header *header)
{
#ifdef CONFIG_SPL_LOAD_FIT_FULL
	int ret = spl_load_fit_image(spl_image, header);

	if (!ret)
		return ret;
#endif
	printk("%s (%d): header=%p\n", __func__, __LINE__, header); // test-only
	if (image_get_magic(header) == IH_MAGIC) {
		printk("%s (%d)\n", __func__, __LINE__); // test-only
#ifdef CONFIG_SPL_LEGACY_IMAGE_SUPPORT
		u32 header_size = sizeof(struct image_header);

#ifdef CONFIG_SPL_LEGACY_IMAGE_CRC_CHECK
		/* check uImage header CRC */
		if (!image_check_hcrc(header)) {
			puts("SPL: Image header CRC check failed!\n");
			return -EINVAL;
		}
#endif

		if (spl_image->flags & SPL_COPY_PAYLOAD_ONLY) {
			/*
			 * On some system (e.g. powerpc), the load-address and
			 * entry-point is located at address 0. We can't load
			 * to 0-0x40. So skip header in this case.
			 */
			spl_image->load_addr = image_get_load(header);
			spl_image->entry_point = image_get_ep(header);
			spl_image->size = image_get_data_size(header);
		} else {
			spl_image->entry_point = image_get_ep(header);
			/* Load including the header */
			spl_image->load_addr = image_get_load(header) -
				header_size;
			spl_image->size = image_get_data_size(header) +
				header_size;
		}
#ifdef CONFIG_SPL_LEGACY_IMAGE_CRC_CHECK
		/* store uImage data length and CRC to check later */
		spl_image->dcrc_data = image_get_load(header);
		spl_image->dcrc_length = image_get_data_size(header);
		spl_image->dcrc = image_get_dcrc(header);
#endif

		spl_image->os = image_get_os(header);
		spl_image->name = image_get_name(header);
		debug(SPL_TPL_PROMPT
		      "payload image: %32s load addr: 0x%lx size: %d\n",
		      spl_image->name, spl_image->load_addr, spl_image->size);
#else
		/* LEGACY image not supported */
		debug("Legacy boot image support not enabled, proceeding to other boot methods\n");
		return -EINVAL;
#endif
	} else {
#ifdef CONFIG_SPL_PANIC_ON_RAW_IMAGE
		/*
		 * CONFIG_SPL_PANIC_ON_RAW_IMAGE is defined when the
		 * code which loads images in SPL cannot guarantee that
		 * absolutely all read errors will be reported.
		 * An example is the LPC32XX MLC NAND driver, which
		 * will consider that a completely unreadable NAND block
		 * is bad, and thus should be skipped silently.
		 */
		panic("** no mkimage signature but raw image not supported");
#endif

#ifdef CONFIG_SPL_OS_BOOT
		ulong start, end;

		if (!bootz_setup((ulong)header, &start, &end)) {
			spl_image->name = "Linux";
			spl_image->os = IH_OS_LINUX;
			spl_image->load_addr = CONFIG_SYS_LOAD_ADDR;
			spl_image->entry_point = CONFIG_SYS_LOAD_ADDR;
			spl_image->size = end - start;
			debug(SPL_TPL_PROMPT
			      "payload zImage, load addr: 0x%lx size: %d\n",
			      spl_image->load_addr, spl_image->size);
			return 0;
		}
#endif

#if 0
//#ifdef CONFIG_SPL_RAW_IMAGE_SUPPORT
		/* Signature not found - assume u-boot.bin */
		debug("mkimage signature not found - ih_magic = %x\n",
			header->ih_magic);
		spl_set_header_raw_uboot(spl_image);
//#else
#endif
		/* RAW image not supported, proceed to other boot methods. */
		debug("Raw boot image support not enabled, proceeding to other boot methods\n");
		return -EINVAL;
//#endif
	}

	return 0;
}



static ulong spl_nor_load_read(struct spl_load_info *load, ulong sector,
			       ulong count, void *buf)
{
	debug("%s: sector %lx, count %lx, buf %p\n",
	      __func__, sector, count, buf);
	memcpy(buf, (void *)sector, count);

	return count;
}

#if 0
unsigned long __weak spl_nor_get_uboot_base(void)
{
	printk("%s (%d): CONFIG_SYS_UBOOT_BASE=%08x\n", __func__, __LINE__, CONFIG_SYS_UBOOT_BASE); // test-only
	return CONFIG_SYS_UBOOT_BASE;
}
#endif

unsigned long spl_nor_get_uboot_base(void)
{
	void *uboot_base = (void *)0x9c009864;

	printk("%s (%d): uboot_base=%p\n", __func__, __LINE__, uboot_base); // test-only
	if (fdt_magic(uboot_base) == FDT_MAGIC)
		return (unsigned long)uboot_base + fdt_totalsize(uboot_base);

	return (unsigned long)uboot_base;
}





#define LZMA_LEN	(1 << 20)

int spl_parse_legacy_header(struct spl_image_info *spl_image,
			    const struct image_header *header)
{
	u32 header_size = sizeof(struct image_header);

	/* check uImage header CRC */
	if (IS_ENABLED(CONFIG_SPL_LEGACY_IMAGE_CRC_CHECK) &&
	    !image_check_hcrc(header)) {
		puts("SPL: Image header CRC check failed!\n");
		return -EINVAL;
	}

	if (spl_image->flags & SPL_COPY_PAYLOAD_ONLY) {
		/*
		 * On some system (e.g. powerpc), the load-address and
		 * entry-point is located at address 0. We can't load
		 * to 0-0x40. So skip header in this case.
		 */
		spl_image->load_addr = image_get_load(header);
		spl_image->entry_point = image_get_ep(header);
		spl_image->size = image_get_data_size(header);
	} else {
		spl_image->entry_point = image_get_ep(header);
		/* Load including the header */
		spl_image->load_addr = image_get_load(header) -
			header_size;
		spl_image->size = image_get_data_size(header) +
			header_size;
	}

#ifdef CONFIG_SPL_LEGACY_IMAGE_CRC_CHECK
	/* store uImage data length and CRC to check later */
	spl_image->dcrc_data = image_get_load(header);
	spl_image->dcrc_length = image_get_data_size(header);
	spl_image->dcrc = image_get_dcrc(header);
#endif

	spl_image->os = image_get_os(header);
	spl_image->name = image_get_name(header);
	debug(SPL_TPL_PROMPT
	      "payload image: %32s load addr: 0x%lx size: %d\n",
	      spl_image->name, spl_image->load_addr, spl_image->size);

	return 0;
}




int spl_load_legacy_img(struct spl_image_info *spl_image,
			const struct image_header *header,
			uintptr_t dataptr, struct spl_load_info *info)
{
	__maybe_unused SizeT lzma_len;
	__maybe_unused void *src;
	int ret;

	printk("%s (%d)\n", __func__, __LINE__); // test-only
	ret = spl_parse_image_header(spl_image, header);
	if (ret)
		return ret;

	printk("%s (%d)\n", __func__, __LINE__); // test-only
	switch (image_get_comp(header)) {
	case IH_COMP_NONE:
		info->read(info, dataptr, spl_image->size,
			   (void *)(unsigned long)spl_image->load_addr);
		break;

#if IS_ENABLED(CONFIG_SPL_LZMA)
	case IH_COMP_LZMA:
		lzma_len = LZMA_LEN;

		debug("LZMA: Decompressing %08lx to %08lx\n",
		      dataptr, spl_image->load_addr);
		src = malloc(spl_image->size);
		if (!src) {
			printf("Unable to allocate %d bytes for LZMA\n",
			       spl_image->size);
			return -ENOMEM;
		}

		info->read(info, dataptr, spl_image->size, src);
		ret = lzmaBuffToBuffDecompress((void *)spl_image->load_addr,
					       &lzma_len, src, spl_image->size);
		if (ret) {
			printf("LZMA decompression error: %d\n", ret);
			return ret;
		}

		spl_image->size = lzma_len;
		break;
#endif

	default:
		debug("Compression method %s is not supported\n",
		      genimg_get_comp_short_name(image_get_comp(header)));
		return -EINVAL;
	}

	/* Flush cache of loaded U-Boot image */
	flush_cache((unsigned long)spl_image->load_addr, spl_image->size);

	return 0;
}


static int spl_nor_load_image(struct spl_image_info *spl_image,
			      struct spl_boot_device *bootdev)
{
	__maybe_unused const struct image_header *header;
	__maybe_unused struct spl_load_info load;
	int ret;

	/*
	 * Loading of the payload to SDRAM is done with skipping of
	 * the mkimage header in this SPL NOR driver
	 */
	spl_image->flags |= SPL_COPY_PAYLOAD_ONLY;

#ifdef CONFIG_SPL_OS_BOOT
	if (!spl_start_uboot()) {
		/*
		 * Load Linux from its location in NOR flash to its defined
		 * location in SDRAM
		 */
		header = (const struct image_header *)CONFIG_SYS_OS_BASE;
#ifdef CONFIG_SPL_LOAD_FIT
		if (image_get_magic(header) == FDT_MAGIC) {
			debug("Found FIT\n");
			load.bl_len = 1;
			load.read = spl_nor_load_read;

			ret = spl_load_simple_fit(spl_image, &load,
						  CONFIG_SYS_OS_BASE,
						  (void *)header);

#if defined CONFIG_SYS_SPL_ARGS_ADDR && defined CONFIG_CMD_SPL_NOR_OFS
			memcpy((void *)CONFIG_SYS_SPL_ARGS_ADDR,
			       (void *)CONFIG_CMD_SPL_NOR_OFS,
			       CONFIG_CMD_SPL_WRITE_SIZE);
#endif
			return ret;
		}
#endif
		if (image_get_os(header) == IH_OS_LINUX) {
			/* happy - was a Linux */

			ret = spl_parse_image_header(spl_image, header);
			if (ret)
				return ret;

			memcpy((void *)spl_image->load_addr,
			       (void *)(CONFIG_SYS_OS_BASE +
					sizeof(struct image_header)),
			       spl_image->size);
#ifdef CONFIG_SYS_FDT_BASE
			spl_image->arg = (void *)CONFIG_SYS_FDT_BASE;
#endif

			return 0;
		} else {
			puts("The Expected Linux image was not found.\n"
			     "Please check your NOR configuration.\n"
			     "Trying to start u-boot now...\n");
		}
	}
#endif

	/*
	 * Load real U-Boot from its location in NOR flash to its
	 * defined location in SDRAM
	 */
#ifdef CONFIG_SPL_LOAD_FIT
	header = (const struct image_header *)spl_nor_get_uboot_base();
	if (image_get_magic(header) == FDT_MAGIC) {
		debug("Found FIT format U-Boot\n");
		load.bl_len = 1;
		load.read = spl_nor_load_read;
		ret = spl_load_simple_fit(spl_image, &load,
					  spl_nor_get_uboot_base(),
					  (void *)header);

		return ret;
	}
#endif
	if (IS_ENABLED(CONFIG_SPL_LOAD_IMX_CONTAINER)) {
		load.bl_len = 1;
		load.read = spl_nor_load_read;
		return spl_load_imx_container(spl_image, &load,
					      spl_nor_get_uboot_base());
	}

	printk("%s (%d)\n", __func__, __LINE__); // test-only
	if (IS_ENABLED(CONFIG_SPL_LEGACY_IMAGE_SUPPORT)) {
		struct image_header hdr;
		uintptr_t dataptr;

		printk("%s (%d)\n", __func__, __LINE__); // test-only
		/* Payload image may not be aligned, so copy it for safety */
		memcpy(&hdr, (void *)spl_nor_get_uboot_base(), sizeof(hdr));
		dataptr = spl_nor_get_uboot_base() + sizeof(hdr);

		/* Legacy image handling */
		load.bl_len = 1;
		load.read = spl_nor_load_read;
		return spl_load_legacy_img(spl_image, &hdr, dataptr, &load);
	}

	return 0;
}
SPL_LOAD_IMAGE_METHOD("NOR", 0, BOOT_DEVICE_NOR, spl_nor_load_image);

static int spl_load_image(struct spl_image_info *spl_image,
			  struct spl_image_loader *loader)
{
	int ret;
	struct spl_boot_device bootdev;

	bootdev.boot_device = loader->boot_device;
	bootdev.boot_device_name = NULL;

	loader->load_image = spl_nor_load_image; // test-only

	printk("%s (%d)\n", __func__, __LINE__); // test-only
	ret = loader->load_image(spl_image, &bootdev);
	printk("%s (%d)\n", __func__, __LINE__); // test-only
#ifdef CONFIG_SPL_LEGACY_IMAGE_CRC_CHECK
	if (!ret && spl_image->dcrc_length) {
		/* check data crc */
		ulong dcrc = crc32_wd(0, (unsigned char *)spl_image->dcrc_data,
				      spl_image->dcrc_length, CHUNKSZ_CRC32);
		if (dcrc != spl_image->dcrc) {
			puts("SPL: Image data CRC check failed!\n");
			ret = -EINVAL;
		}
	}
#endif
	return ret;
}

static struct spl_image_loader *spl_ll_find_loader(uint boot_device)
{
	struct spl_image_loader *drv =
		ll_entry_start(struct spl_image_loader, spl_image_loader);
	const int n_ents =
		ll_entry_count(struct spl_image_loader, spl_image_loader);
	struct spl_image_loader *entry;

	for (entry = drv; entry != drv + n_ents; entry++) {
		if (boot_device == entry->boot_device)
			return entry;
	}

	/* Not found */
	return NULL;
}

int do_spl_test(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	struct spl_image_info spl_image;
	struct spl_image_loader *loader;

	printk("%s (%d)\n", __func__, __LINE__); // test-only
	loader = spl_ll_find_loader(BOOT_DEVICE_NOR);
	spl_load_image(&spl_image, loader);

	return 0;
}

U_BOOT_CMD(
	spl_test,	1,	0,	do_spl_test,
	"SPL test",
	"\n"
);
#endif
