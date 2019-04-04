// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2017 Google, Inc
 */

#include <common.h>
#include <dm.h>
#include <errno.h>
#include <wdt.h>
#include <dm/device-internal.h>
#include <dm/lists.h>

#ifndef WDT_DEFAULT_TIMEOUT
#define WDT_DEFAULT_TIMEOUT	60
#endif

DECLARE_GLOBAL_DATA_PTR;

int wdt_start(struct udevice *dev, u64 timeout_ms, ulong flags)
{
	const struct wdt_ops *ops = device_get_ops(dev);

	if (!ops->start)
		return -ENOSYS;

	return ops->start(dev, timeout_ms, flags);
}

int wdt_stop(struct udevice *dev)
{
	const struct wdt_ops *ops = device_get_ops(dev);

	if (!ops->stop)
		return -ENOSYS;

	return ops->stop(dev);
}

int wdt_reset(struct udevice *dev)
{
	const struct wdt_ops *ops = device_get_ops(dev);

	if (!ops->reset)
		return -ENOSYS;

	return ops->reset(dev);
}

int wdt_expire_now(struct udevice *dev, ulong flags)
{
	int ret = 0;
	const struct wdt_ops *ops;

	debug("WDT Resetting: %lu\n", flags);
	ops = device_get_ops(dev);
	if (ops->expire_now) {
		return ops->expire_now(dev, flags);
	} else {
		if (!ops->start)
			return -ENOSYS;

		ret = ops->start(dev, 1, flags);
		if (ret < 0)
			return ret;

		hang();
	}

	return ret;
}

#ifdef CONFIG_WATCHDOG
/*
 * Called by macro WATCHDOG_RESET. This function be called *very* early,
 * so we need to make sure, that the watchdog driver is ready before using
 * it in this function.
 */
void watchdog_reset(void)
{
	static ulong next_reset;
	ulong now;

	/* Exit if GD is not ready or watchdog is not initialized yet */
	if (!gd || !(gd->flags & GD_FLG_WDT_READY))
		return;

	/* Do not reset the watchdog too often */
	now = get_timer(0);
	if (now > next_reset) {
		next_reset = now + 1000;	/* reset every 1000ms */
		wdt_reset(gd->watchdog_dev);
	}
}
#endif

static int wdt_post_bind(struct udevice *dev)
{
	u32 __maybe_unused timeout = WDT_DEFAULT_TIMEOUT;

#if defined(CONFIG_NEEDS_MANUAL_RELOC)
	struct wdt_ops *ops = (struct wdt_ops *)device_get_ops(dev);
	static int reloc_done;

	if (!reloc_done) {
		if (ops->start)
			ops->start += gd->reloc_off;
		if (ops->stop)
			ops->stop += gd->reloc_off;
		if (ops->reset)
			ops->reset += gd->reloc_off;
		if (ops->expire_now)
			ops->expire_now += gd->reloc_off;

		reloc_done++;
	}
#endif

#ifdef CONFIG_WATCHDOG
	/*
	 * Use only the first watchdog device in U-Boot to trigger the
	 * watchdog reset
	 */
	if (gd->watchdog_dev) {
		debug("Only one watchdog device used!\n");
		return 0;
	}

	/*
	 * Init watchdog: This will call the probe function of the
	 * watchdog driver, enabling the use of the device
	 */
	if (uclass_get_device(UCLASS_WDT, 0,
			      (struct udevice **)&gd->watchdog_dev)) {
		debug("Watchdog: Not found!\n");
		return 0;
	}

#if CONFIG_IS_ENABLED(OF_CONTROL)
	timeout = dev_read_u32_default(gd->watchdog_dev, "timeout-sec",
				       WDT_DEFAULT_TIMEOUT);
#endif

	wdt_start(gd->watchdog_dev, timeout * 1000, 0);
	gd->flags |= GD_FLG_WDT_READY;
	printf("WDT:   Started (%ds timeout)\n", timeout);
#endif

	return 0;
}

UCLASS_DRIVER(wdt) = {
	.id		= UCLASS_WDT,
	.name		= "watchdog",
	.flags		= DM_UC_FLAG_SEQ_ALIAS,
	.post_bind	= wdt_post_bind,
};
