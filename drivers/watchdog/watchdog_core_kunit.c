// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit test for the watchdog core's keepalive worker at reboot.
 *
 * A fake watchdog that is running at registration and that nobody opens is
 * pinged by the core's worker. With stop_on_reboot=0 the reboot notifier
 * must end those pings and leave the hardware running.
 */

#include <kunit/test.h>
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/watchdog.h>

struct wdt_fake {
	struct watchdog_device wdd;
	atomic_t pings;
	atomic_t stops;
};

static int wdt_fake_start(struct watchdog_device *wdd)
{
	return 0;
}

static int wdt_fake_stop(struct watchdog_device *wdd)
{
	struct wdt_fake *fake = watchdog_get_drvdata(wdd);

	atomic_inc(&fake->stops);
	return 0;
}

static int wdt_fake_ping(struct watchdog_device *wdd)
{
	struct wdt_fake *fake = watchdog_get_drvdata(wdd);

	atomic_inc(&fake->pings);
	return 0;
}

static const struct watchdog_info wdt_fake_info = {
	.options = WDIOF_KEEPALIVEPING | WDIOF_SETTIMEOUT | WDIOF_MAGICCLOSE,
	.identity = "kunit fake watchdog",
};

static const struct watchdog_ops wdt_fake_ops = {
	.start = wdt_fake_start,
	.stop = wdt_fake_stop,
	.ping = wdt_fake_ping,
};

static void wdt_fake_unregister(void *data)
{
	struct wdt_fake *fake = data;

	clear_bit(WDOG_HW_RUNNING, &fake->wdd.status);
	watchdog_unregister_device(&fake->wdd);
}

/*
 * A 1 s timeout: the worker pings every 500 ms while it runs. Registered
 * stopped and then marked running, as a driver that finds its watchdog
 * running after registration does, so the core takes no module or device
 * reference for it.
 */
static struct wdt_fake *wdt_fake_register(struct kunit *test)
{
	struct wdt_fake *fake;

	fake = kunit_kzalloc(test, sizeof(*fake), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, fake);

	fake->wdd.info = &wdt_fake_info;
	fake->wdd.ops = &wdt_fake_ops;
	fake->wdd.min_timeout = 1;
	fake->wdd.max_timeout = 10;
	fake->wdd.timeout = 1;
	watchdog_set_drvdata(&fake->wdd, fake);

	KUNIT_ASSERT_EQ(test, watchdog_register_device(&fake->wdd), 0);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, wdt_fake_unregister,
							fake), 0);
	set_bit(WDOG_HW_RUNNING, &fake->wdd.status);
	KUNIT_ASSERT_EQ(test, watchdog_set_last_hw_keepalive(&fake->wdd, 0), 0);
	return fake;
}

static void watchdog_reboot_keep_stops_pings(struct kunit *test)
{
	struct wdt_fake *fake = wdt_fake_register(test);
	struct notifier_block *nb = &fake->wdd.reboot_nb;
	int before;

	if (test_bit(WDOG_STOP_ON_REBOOT, &fake->wdd.status) ||
	    !nb->notifier_call)
		kunit_skip(test, "needs watchdog.stop_on_reboot=0");

	msleep(1600);
	KUNIT_EXPECT_GE(test, atomic_read(&fake->pings), 3);

	KUNIT_EXPECT_EQ(test, nb->notifier_call(nb, SYS_RESTART, NULL),
			NOTIFY_DONE);
	/* a ping already queued may still land */
	msleep(100);
	before = atomic_read(&fake->pings);
	msleep(2100);
	KUNIT_EXPECT_EQ(test, atomic_read(&fake->pings), before);
	KUNIT_EXPECT_EQ(test, atomic_read(&fake->stops), 0);
	KUNIT_EXPECT_TRUE(test, watchdog_hw_running(&fake->wdd));
}

static void watchdog_reboot_keep_ignores_other_events(struct kunit *test)
{
	struct wdt_fake *fake = wdt_fake_register(test);
	struct notifier_block *nb = &fake->wdd.reboot_nb;
	int before;

	if (test_bit(WDOG_STOP_ON_REBOOT, &fake->wdd.status) ||
	    !nb->notifier_call)
		kunit_skip(test, "needs watchdog.stop_on_reboot=0");

	/* 0 is no reboot, halt or power-off: the worker goes on */
	nb->notifier_call(nb, 0, NULL);
	msleep(100);
	before = atomic_read(&fake->pings);
	msleep(1600);
	KUNIT_EXPECT_GE(test, atomic_read(&fake->pings), before + 2);
}

static struct kunit_case watchdog_core_test_cases[] = {
	KUNIT_CASE_SLOW(watchdog_reboot_keep_stops_pings),
	KUNIT_CASE_SLOW(watchdog_reboot_keep_ignores_other_events),
	{}
};

static struct kunit_suite watchdog_core_test_suite = {
	.name = "watchdog_core",
	.test_cases = watchdog_core_test_cases,
};

kunit_test_suite(watchdog_core_test_suite);

MODULE_DESCRIPTION("KUnit test for the watchdog core's keepalive at reboot");
MODULE_LICENSE("GPL");
