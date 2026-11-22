// SPDX-License-Identifier: GPL-2.0-only
/*
 * ASUS Zenbook A14 UX3407QA Embedded Controller driver
 *
 * The EC exposes its mailbox at I2C address 0x5b. The mailbox uses raw
 * register and compound operations for fan and temperature readings.
 */
// allow: SIZE_OK - I2C lifecycle and hwmon callbacks share EC state.

#include <linux/atomic.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/hwmon.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_profile.h>
#include <linux/pm.h>
#include <linux/pm_qos.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include <linux/timekeeping.h>
#include <linux/workqueue.h>

#define DRV_NAME		"asus-zenbook-a14-ec"
#define HWMON_NAME		"asus_zenbook_a14_ec"

static bool read_c6_once;
module_param(read_c6_once, bool, 0);
MODULE_PARM_DESC(read_c6_once,
		 "Read EC register C6:00 once at probe for diagnostics");

/* Hidden: delayed C6:00 read, exposed via the hwmon c6_read_once attribute. */
static bool read_c6_delayed;
module_param(read_c6_delayed, bool, 0);

/* Uptime after which the delayed C6:00 read is allowed. */
#define C6_DELAY_NS	(900ULL * NSEC_PER_SEC)

static bool tj_feed;
module_param(tj_feed, bool, 0);
MODULE_PARM_DESC(tj_feed,
		 "Feed the hottest SoC thermal zone temperature to EC mailbox C6:00 once per second (experimental, UX3407QA only)");

/* Host SoC temperature mailbox, written by the Windows driver. */
#define EC_TJ_MAJ		0xc6
#define EC_TJ_MIN		0x00
#define EC_TJ_SAFE		0x4b	/* EC boot default (75 C) */
#define EC_TJ_MIN_C		20
#define EC_TJ_MAX_C		120
#define EC_TJ_INTERVAL_MS	1000
#define EC_TJ_MAX_FAIL		5

#define EC_I2C_ADDR		0x5b

/* EC protocol opcodes */
#define EC_OP_ADDR		0x10
#define EC_OP_DATA		0x11

/* Compound-op mailboxes at major 0xc4 */
#define EC_CC_BUSY		0x30
#define EC_CC_REGSEL		0x31
#define EC_CC_DATA		0x32

#define EC_SETTLE_INTERVAL_US	50000
#define EC_SETTLE_TIMEOUT_MS	2000

/* hwmon-exposed registers */
#define EC_REG_FAN_MODE_MAJ	0x01
#define EC_REG_FAN_MODE_RMIN	0x02	/* read:  0=auto, 2=manual */
#define EC_REG_FAN_MODE_WMIN	0x82	/* write: 0=auto, 2=manual */
#define EC_REG_FAN_TACH_MAJ	0x01
#define EC_REG_FAN_TACH_MIN	0x09	/* RPM = value × 88 */
#define EC_REG_PWM_MAJ		0x01
#define EC_REG_PWM_RMIN		0x0a	/* read:  0-255 */
#define EC_REG_PWM_WMIN		0x8a	/* write: 0-255 */
#define EC_REG_FAN_SEL_WMIN	0x8c	/* fan-id selector for PWM/tach */
#define EC_REG_TEMP_MAJ		0x05
#define EC_REG_TEMP_MIN		0x02	/* °C */

#define EC_NUM_FANS		2

#define FC_MAX_POINTS			8
#define FC_DEFAULT_INTERVAL_MS		2000
#define FC_DEFAULT_HYSTERESIS_C	2

/* Tach-to-RPM conversion: empirically RPM ≈ tach × 88. */
#define EC_TACH_RPM_MULT	88

/* PWM floor below which the fan does not spin. */
#define EC_PWM_SPIN_FLOOR	0x4b	/* 75 */
#define EC_TEMP_CRITICAL	80

/* Fan mode encoding observed in (0x01, 0x02). */
#define EC_FAN_MODE_AUTO	0
#define EC_FAN_MODE_MANUAL	2

/* CPU frequency capping for quiet profile (kHz). */
#define PP_QUIET_FREQ_KHZ	1440000
#define PP_MAX_FREQ_KHZ		3417600
#define PP_MAX_POLICIES		4

enum asus_ec_fan_owner {
	OWNER_NONE,
	OWNER_PLATFORM_PROFILE,
	OWNER_HWMON,
	OWNER_FAN_CURVE,
};

struct asus_ec_suspend_state {
	enum asus_ec_fan_owner owner;
	enum platform_profile_option pp;
	u8 pwm[EC_NUM_FANS];
	bool curve_enabled;
};

struct asus_ec {
	struct device		*dev;
	struct i2c_client	*client;
	struct mutex		lock;		/* serialises EC compound access */
	struct mutex		mode_lock;
	struct mutex		curve_lock;
	enum asus_ec_fan_owner manual_owner;

	struct device		*profile_dev;
	enum platform_profile_option pp_active;

	struct freq_qos_request	freq_req[PP_MAX_POLICIES];
	int			n_freq_req;
	bool			freq_qos_ok;

	struct device		*hwmon_dev;
	bool			c6_delayed;
	struct delayed_work	tj_work;
	unsigned int		tj_fail;
	bool			tj_started;
	struct delayed_work	curve_work;
	bool			curve_enabled;
	bool			curve_suspended;
	bool			curve_recovering;
	bool			suspend_prepared;
	unsigned int		curve_interval_ms;
	unsigned int		curve_hysteresis_c;
	int			curve_nr_points;
	int			curve_temps[FC_MAX_POINTS];
	int			curve_pwms[FC_MAX_POINTS];
	int			curve_last_temp;
	u8			curve_last_pwm;
	struct asus_ec_suspend_state suspend_state;

	/* DMA-safe scratch (kmalloc-backed via devm_kzalloc) */
	u8			tx[3];
	u8			rx[1];
};

/* ------------------------------------------------------------------ */
/* EC primitives — caller MUST hold ec->lock                          */
/* ------------------------------------------------------------------ */

static int __ec_rb(struct asus_ec *ec, u8 maj, u8 min, u8 *out)
{
	struct i2c_msg msgs[3];
	static const u8 op_data = EC_OP_DATA;
	int ret;

	ec->tx[0] = EC_OP_ADDR;
	ec->tx[1] = maj;
	ec->tx[2] = min;

	msgs[0].addr  = EC_I2C_ADDR;
	msgs[0].flags = 0;
	msgs[0].len   = 3;
	msgs[0].buf   = ec->tx;

	msgs[1].addr  = EC_I2C_ADDR;
	msgs[1].flags = 0;
	msgs[1].len   = 1;
	msgs[1].buf   = (u8 *)&op_data;

	msgs[2].addr  = EC_I2C_ADDR;
	msgs[2].flags = I2C_M_RD;
	msgs[2].len   = 1;
	msgs[2].buf   = ec->rx;

	ret = i2c_transfer(ec->client->adapter, msgs, 3);
	if (ret < 0) {
		dev_dbg(ec->dev, "ecrb(0x%02x,0x%02x) failed: %d\n",
			maj, min, ret);
		return ret;
	}
	if (ret != 3)
		return -EIO;

	*out = ec->rx[0];
	return 0;
}

static int __ec_wb(struct asus_ec *ec, u8 maj, u8 min, u8 val)
{
	struct i2c_msg msgs[2];
	u8 data[2];
	int ret;

	ec->tx[0] = EC_OP_ADDR;
	ec->tx[1] = maj;
	ec->tx[2] = min;

	data[0] = EC_OP_DATA;
	data[1] = val;

	msgs[0].addr  = EC_I2C_ADDR;
	msgs[0].flags = 0;
	msgs[0].len   = 3;
	msgs[0].buf   = ec->tx;

	msgs[1].addr  = EC_I2C_ADDR;
	msgs[1].flags = 0;
	msgs[1].len   = 2;
	msgs[1].buf   = data;

	ret = i2c_transfer(ec->client->adapter, msgs, 2);
	if (ret < 0) {
		dev_dbg(ec->dev, "ecwb(0x%02x,0x%02x,0x%02x) failed: %d\n",
			maj, min, val, ret);
		return ret;
	}
	if (ret != 2)
		return -EIO;
	return 0;
}

static int __ec_settle(struct asus_ec *ec)
{
	unsigned long deadline;
	u8 v;
	int ret;

	deadline = jiffies + msecs_to_jiffies(EC_SETTLE_TIMEOUT_MS);

	for (;;) {
		ret = __ec_rb(ec, 0xc4, EC_CC_BUSY, &v);
		if (ret)
			return ret;
		if (v == 0)
			return 0;
		if (time_after(jiffies, deadline)) {
			dev_warn(ec->dev,
				 "ec_settle timeout (last busy=0x%02x)\n", v);
			return -ETIMEDOUT;
		}
		usleep_range(EC_SETTLE_INTERVAL_US,
			     EC_SETTLE_INTERVAL_US + 10000);
	}
}

static int __ec_cr(struct asus_ec *ec, u8 a1, u8 a2, u8 *out)
{
	int ret;
	u8 v;

	if (a2 >= 0x80) {
		dev_err(ec->dev,
			"eccr refused: a2=0x%02x >= 0x80 destructive\n", a2);
		return -EINVAL;
	}

	ret = __ec_settle(ec);
	if (ret)
		return ret;

	ret = __ec_wb(ec, 0xc4, EC_CC_REGSEL, a2);
	if (ret)
		return ret;
	ret = __ec_wb(ec, 0xc4, EC_CC_BUSY, a1);
	if (ret)
		return ret;

	ret = __ec_settle(ec);
	if (ret)
		return ret;

	ret = __ec_rb(ec, 0xc4, EC_CC_DATA, &v);
	if (ret)
		return ret;

	ret = __ec_wb(ec, 0xc4, EC_CC_DATA, 0x00);
	if (ret)
		return ret;

	*out = v;
	return 0;
}

static int __ec_cw(struct asus_ec *ec, u8 a1, u8 a2, u8 val)
{
	int ret;

	ret = __ec_settle(ec);
	if (ret)
		return ret;

	ret = __ec_wb(ec, 0xc4, EC_CC_REGSEL, a2);
	if (ret)
		return ret;
	ret = __ec_wb(ec, 0xc4, EC_CC_DATA, val);
	if (ret)
		return ret;
	ret = __ec_wb(ec, 0xc4, EC_CC_BUSY, a1);
	if (ret)
		return ret;

	return __ec_settle(ec);
}

static int asus_ec_read_reg(struct asus_ec *ec, u8 maj, u8 min, u8 *out)
{
	int ret;

	mutex_lock(&ec->lock);
	ret = __ec_cr(ec, maj, min, out);
	mutex_unlock(&ec->lock);
	return ret;
}

/* ------------------------------------------------------------------ */
/* One-shot direct EC read of C6:00                                   */
/* ------------------------------------------------------------------ */

/* Global per-boot attempt latch: the read is never retried, not even
 * after a failed transfer.
 */
static atomic_t c6_attempted = ATOMIC_INIT(0);

/*
 * Perform the single direct EC transfer of register C6:00.
 * Returns 0 and stores the raw byte in @raw on success, -EALREADY if the
 * per-boot attempt was already used, or -EIO on transfer failure.
 */
static int asus_ec_read_c6_once(struct asus_ec *ec, u8 *raw)
{
	int ret;

	if (atomic_xchg(&c6_attempted, 1))
		return -EALREADY;

	mutex_lock(&ec->lock);
	ret = __ec_rb(ec, 0xc6, 0x00, raw);
	mutex_unlock(&ec->lock);
	if (ret) {
		dev_err(ec->dev, "C6:00 read failed: %d\n", ret);
		return -EIO;
	}

	dev_info(ec->dev, "C6:00 raw=0x%02x\n", *raw);
	return 0;
}

static int asus_ec_read_temp_input(struct asus_ec *ec, long *temp_mc)
{
	u8 temp;
	int ret;

	ret = asus_ec_read_reg(ec, EC_REG_TEMP_MAJ, EC_REG_TEMP_MIN, &temp);
	if (ret)
		return ret;

	*temp_mc = (long)temp * 1000;
	return 0;
}

static int asus_ec_set_fan_mode(struct asus_ec *ec, u8 mode)
{
	u8 actual;
	int ret;

	mutex_lock(&ec->lock);
	ret = __ec_cw(ec, EC_REG_FAN_MODE_MAJ, EC_REG_FAN_MODE_WMIN, mode);
	if (ret)
		goto out;
	/* Let the EC publish the mode change before verifying it. */
	usleep_range(EC_SETTLE_INTERVAL_US,
		     EC_SETTLE_INTERVAL_US + 10000);
	ret = __ec_cr(ec, EC_REG_FAN_MODE_MAJ, EC_REG_FAN_MODE_RMIN, &actual);
	if (ret)
		goto out;
	if (actual != mode) {
		dev_warn_ratelimited(ec->dev,
				     "fan mode write mismatch: requested %u, read %u\n",
				     mode, actual);
		ret = -EIO;
	}
out:
	mutex_unlock(&ec->lock);
	return ret;
}

static int asus_ec_set_pwm(struct asus_ec *ec, int fan_id, u8 speed)
{
	u8 actual;
	int ret;

	mutex_lock(&ec->lock);
	ret = __ec_cw(ec, EC_REG_PWM_MAJ, EC_REG_FAN_SEL_WMIN, fan_id);
	if (ret)
		goto out;
	/* Let the fan selector latch before issuing the PWM command. */
	usleep_range(EC_SETTLE_INTERVAL_US,
		     EC_SETTLE_INTERVAL_US + 10000);
	ret = __ec_cw(ec, EC_REG_PWM_MAJ, EC_REG_PWM_WMIN, speed);
	if (ret)
		goto out;
	/* Let the EC publish the command result before verifying it. */
	usleep_range(EC_SETTLE_INTERVAL_US,
		     EC_SETTLE_INTERVAL_US + 10000);
	ret = __ec_cr(ec, EC_REG_PWM_MAJ, EC_REG_PWM_RMIN, &actual);
	if (ret)
		goto out;
	if (actual != speed) {
		dev_warn_ratelimited(ec->dev,
				     "fan%d PWM write mismatch: requested %u, read %u\n",
				     fan_id, speed, actual);
		ret = -EIO;
	}
out:
	mutex_unlock(&ec->lock);
	return ret;
}

static int asus_ec_set_pwm_both(struct asus_ec *ec, u8 speed)
{
	int first_ret;
	int ret;

	first_ret = asus_ec_set_pwm(ec, 0, speed);
	ret = asus_ec_set_pwm(ec, 1, speed);

	return first_ret ? first_ret : ret;
}

/* Caller MUST hold ec->mode_lock. */
static int asus_ec_force_auto(struct asus_ec *ec)
{
	int ret;

	ret = asus_ec_set_fan_mode(ec, EC_FAN_MODE_AUTO);
	if (!ret)
		ec->manual_owner = OWNER_NONE;

	return ret;
}

/* Caller MUST hold ec->mode_lock. */
static int asus_ec_enter_manual(struct asus_ec *ec,
				enum asus_ec_fan_owner owner)
{
	int ret;

	if (ec->manual_owner == owner)
		return 0;
	if (ec->manual_owner != OWNER_NONE)
		return -EBUSY;

	ret = asus_ec_set_fan_mode(ec, EC_FAN_MODE_MANUAL);
	if (!ret)
		ec->manual_owner = owner;

	return ret;
}

/* Caller MUST hold ec->mode_lock. */
static int asus_ec_leave_manual(struct asus_ec *ec,
				enum asus_ec_fan_owner owner)
{
	if (ec->manual_owner != owner)
		return 0;

	return asus_ec_force_auto(ec);
}

static __maybe_unused u8 asus_ec_apply_pwm_floor(u8 pwm)
{
	return pwm == 0 ? 0 : max(pwm, EC_PWM_SPIN_FLOOR);
}

static int asus_ec_read_fan_reg(struct asus_ec *ec, int fan_id,
				 u8 reg_min, u8 *out)
{
	int ret;

	mutex_lock(&ec->lock);
	ret = __ec_cw(ec, EC_REG_PWM_MAJ, EC_REG_FAN_SEL_WMIN, fan_id);
	if (ret)
		goto out;
	ret = __ec_cr(ec, EC_REG_FAN_TACH_MAJ, reg_min, out);
out:
	mutex_unlock(&ec->lock);
	return ret;
}

/* Caller MUST hold ec->mode_lock. */
static int asus_ec_save_suspend_state(struct asus_ec *ec)
{
	struct asus_ec_suspend_state *state = &ec->suspend_state;
	int fan;
	int ret;

	state->owner = ec->manual_owner;
	state->pp = ec->pp_active;
	state->curve_enabled = ec->curve_enabled;

	for (fan = 0; fan < EC_NUM_FANS; fan++) {
		ret = asus_ec_read_fan_reg(ec, fan, EC_REG_PWM_RMIN,
					   &state->pwm[fan]);
		if (ret)
			return ret;
	}

	return 0;
}

/* ------------------------------------------------------------------ */
/* CPU frequency QoS                                                  */
/* ------------------------------------------------------------------ */

static void asus_ec_freq_qos_init(struct asus_ec *ec)
{
	struct cpufreq_policy *policy;
	int cpu, ret;

	ec->n_freq_req = 0;
	ec->freq_qos_ok = false;

	for_each_possible_cpu(cpu) {
		if (ec->n_freq_req >= PP_MAX_POLICIES)
			break;

		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			continue;

		if (cpu != cpumask_first(policy->related_cpus)) {
			cpufreq_cpu_put(policy);
			continue;
		}

		ret = freq_qos_add_request(&policy->constraints,
					   &ec->freq_req[ec->n_freq_req],
					   FREQ_QOS_MAX, PP_MAX_FREQ_KHZ);
		cpufreq_cpu_put(policy);

		if (ret < 0) {
			dev_warn(ec->dev,
				 "freq_qos: failed to add request for cpu%d: %d\n",
				 cpu, ret);
			continue;
		}

		ec->n_freq_req++;
	}

	ec->freq_qos_ok = ec->n_freq_req > 0;

	dev_info(ec->dev, "freq_qos: %d policies registered\n", ec->n_freq_req);
}

static void asus_ec_freq_qos_cleanup(struct asus_ec *ec)
{
	int i;

	for (i = 0; i < ec->n_freq_req; i++)
		freq_qos_remove_request(&ec->freq_req[i]);
	ec->n_freq_req = 0;
	ec->freq_qos_ok = false;
}

static int asus_ec_freq_qos_set(struct asus_ec *ec, s32 max_khz)
{
	int i, ret;

	for (i = 0; i < ec->n_freq_req; i++) {
		ret = freq_qos_update_request(&ec->freq_req[i], max_khz);
		if (ret < 0) {
			dev_warn(ec->dev,
				 "freq_qos: update %d to %d kHz failed: %d\n",
				 i, max_khz, ret);
			return ret;
		}
	}

	return 0;
}

/* ------------------------------------------------------------------ */
/* platform_profile integration                                       */
/* ------------------------------------------------------------------ */

#define PP_PERF_PWM	180

/* Caller MUST hold ec->mode_lock. */
static int asus_ec_apply_profile_state(struct asus_ec *ec,
				       enum platform_profile_option profile)
{
	int ret;

	switch (profile) {
	case PLATFORM_PROFILE_QUIET:
	case PLATFORM_PROFILE_BALANCED:
		return asus_ec_leave_manual(ec, OWNER_PLATFORM_PROFILE);
	case PLATFORM_PROFILE_PERFORMANCE:
		ret = asus_ec_enter_manual(ec, OWNER_PLATFORM_PROFILE);
		if (ret)
			return ret;
		return asus_ec_set_pwm_both(ec,
					    asus_ec_apply_pwm_floor(PP_PERF_PWM));
	default:
		return -EOPNOTSUPP;
	}
}

/* Caller MUST hold ec->mode_lock. */
static int asus_ec_restore_suspend_state(struct asus_ec *ec)
{
	struct asus_ec_suspend_state *state = &ec->suspend_state;
	int fan;
	int ret;

	switch (state->owner) {
	case OWNER_NONE:
		ret = asus_ec_force_auto(ec);
		break;
	case OWNER_PLATFORM_PROFILE:
		ret = asus_ec_apply_profile_state(ec, state->pp);
		break;
	case OWNER_HWMON:
	case OWNER_FAN_CURVE:
		ret = asus_ec_enter_manual(ec, state->owner);
		if (ret)
			break;
		for (fan = 0; fan < EC_NUM_FANS; fan++) {
			ret = asus_ec_set_pwm(ec, fan,
					      asus_ec_apply_pwm_floor(state->pwm[fan]));
			if (ret)
				break;
		}
		break;
	default:
		ret = -EOPNOTSUPP;
	}

	if (ret)
		return ret;

	ec->pp_active = state->pp;
	ec->curve_enabled = state->curve_enabled;
	return 0;
}

/* Caller MUST hold ec->mode_lock. */
static void asus_ec_suspend_fallback_auto(struct asus_ec *ec)
{
	int ret;

	ret = asus_ec_force_auto(ec);
	if (ret) {
		dev_warn(ec->dev, "suspend: restore auto failed: %d\n", ret);
		ec->curve_recovering = true;
		ret = asus_ec_set_pwm_both(ec, U8_MAX);
		if (ret)
			dev_warn(ec->dev,
				 "suspend: full-speed fallback failed: %d\n", ret);
		return;
	}

	ec->pp_active = PLATFORM_PROFILE_BALANCED;
	ec->curve_enabled = false;
	ec->curve_suspended = false;
	ec->curve_recovering = false;
}

static int asus_ec_pp_probe(void *drvdata, unsigned long *choices)
{
	set_bit(PLATFORM_PROFILE_QUIET, choices);
	set_bit(PLATFORM_PROFILE_BALANCED, choices);
	set_bit(PLATFORM_PROFILE_PERFORMANCE, choices);
	return 0;
}

static int asus_ec_pp_get(struct device *dev,
			  enum platform_profile_option *profile)
{
	struct asus_ec *ec = dev_get_drvdata(dev);

	*profile = ec->pp_active;
	return 0;
}

static int asus_ec_pp_set(struct device *dev,
			  enum platform_profile_option profile)
{
	struct asus_ec *ec = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&ec->mode_lock);

	if (profile == PLATFORM_PROFILE_QUIET) {
		if (!ec->freq_qos_ok) {
			ret = -EOPNOTSUPP;
			goto out;
		}

		ret = asus_ec_freq_qos_set(ec, PP_QUIET_FREQ_KHZ);
		if (ret)
			goto out;
	}

	if (ec->curve_enabled) {
		switch (profile) {
		case PLATFORM_PROFILE_QUIET:
			ret = 0;
			break;
		case PLATFORM_PROFILE_BALANCED:
		case PLATFORM_PROFILE_PERFORMANCE:
			ret = asus_ec_freq_qos_set(ec, PP_MAX_FREQ_KHZ);
			break;
		default:
			ret = -EOPNOTSUPP;
			break;
		}
		if (!ret)
			ec->pp_active = profile;
		goto out;
	}

	ret = asus_ec_apply_profile_state(ec, profile);
	if (ret)
		goto out;

	if (profile != PLATFORM_PROFILE_QUIET) {
		ret = asus_ec_freq_qos_set(ec, PP_MAX_FREQ_KHZ);
		if (ret)
			goto out;
	}

	ec->pp_active = profile;
	ret = 0;

out:
	mutex_unlock(&ec->mode_lock);
	return ret;
}

static const struct platform_profile_ops asus_ec_pp_ops = {
	.probe		= asus_ec_pp_probe,
	.profile_get	= asus_ec_pp_get,
	.profile_set	= asus_ec_pp_set,
};

/* Caller MUST hold ec->mode_lock. */
static u8 asus_ec_curve_pwm_for_temp(struct asus_ec *ec, int temp_c)
{
	int i;

	if (temp_c <= ec->curve_temps[0])
		return ec->curve_pwms[0];

	for (i = 1; i < ec->curve_nr_points; i++) {
		s64 temp_delta;
		s64 pwm_delta;

		if (temp_c > ec->curve_temps[i])
			continue;

		temp_delta = (s64)ec->curve_temps[i] - ec->curve_temps[i - 1];
		pwm_delta = ec->curve_pwms[i] - ec->curve_pwms[i - 1];
		return ec->curve_pwms[i - 1] +
			div_s64((s64)(temp_c - ec->curve_temps[i - 1]) * pwm_delta,
				temp_delta);
	}

	return ec->curve_pwms[ec->curve_nr_points - 1];
}

/* Caller MUST hold ec->mode_lock. */
static bool asus_ec_curve_crossed_point(struct asus_ec *ec, int temp_c)
{
	int i;

	for (i = 0; i < ec->curve_nr_points; i++) {
		int point = ec->curve_temps[i];

		if ((ec->curve_last_temp < temp_c &&
		     ec->curve_last_temp < point && temp_c >= point) ||
		    (ec->curve_last_temp > temp_c &&
		     ec->curve_last_temp > point && temp_c <= point))
			return true;
	}

	return false;
}

static void asus_ec_curve_apply(struct asus_ec *ec)
{
	long temp_mc;
	int temp_c;
	int pwm;
	int ret;
	bool update;

	mutex_lock(&ec->mode_lock);

	if (ec->curve_suspended)
		goto out;
	if (ec->curve_recovering)
		goto recover;
	if (!ec->curve_enabled)
		goto out;

	ret = asus_ec_read_temp_input(ec, &temp_mc);
	if (ret) {
		dev_warn(ec->dev, "fan curve: temperature read failed: %d; setting full speed\n",
			 ret);
		ec->curve_last_temp = -1;
		ret = asus_ec_set_pwm_both(ec, U8_MAX);
		if (ret) {
			dev_warn_ratelimited(ec->dev,
					     "fan curve: full-speed write failed: %d\n",
					     ret);
			ec->curve_recovering = true;
			goto recover;
		}
		ec->curve_last_pwm = U8_MAX;
		goto reschedule;
	}

	temp_c = temp_mc / 1000;
	if (temp_c >= EC_TEMP_CRITICAL) {
		ret = asus_ec_set_pwm_both(ec, U8_MAX);
		if (ret) {
			dev_warn_ratelimited(ec->dev,
					     "fan curve: critical full-speed write failed: %d\n",
					     ret);
			ec->curve_recovering = true;
			goto recover;
		}
		ec->curve_last_temp = temp_c;
		ec->curve_last_pwm = U8_MAX;
		goto reschedule;
	}

	pwm = asus_ec_apply_pwm_floor(asus_ec_curve_pwm_for_temp(ec, temp_c));
	update = ec->curve_last_temp < 0 ||
		 abs(temp_c - ec->curve_last_temp) >= ec->curve_hysteresis_c ||
		 asus_ec_curve_crossed_point(ec, temp_c);
	if (!update)
		goto reschedule;

	if (ec->curve_last_temp < 0 || pwm != ec->curve_last_pwm) {
		ret = asus_ec_set_pwm_both(ec, pwm);
		if (ret) {
			dev_warn_ratelimited(ec->dev,
					     "fan curve: pwm write failed: %d; restoring automatic control\n",
					     ret);
			ec->curve_recovering = true;
			goto recover;
		}
	}

	ec->curve_last_temp = temp_c;
	ec->curve_last_pwm = pwm;
	goto reschedule;

recover:
	ret = asus_ec_force_auto(ec);
	if (!ret) {
		ec->curve_enabled = false;
		ec->curve_suspended = false;
		ec->curve_recovering = false;
		goto out;
	}

	dev_warn_ratelimited(ec->dev,
			     "fan curve: automatic control restore failed: %d\n",
			     ret);
	ret = asus_ec_set_pwm_both(ec, U8_MAX);
	if (ret)
		dev_warn_ratelimited(ec->dev,
				     "fan curve: full-speed fallback failed: %d\n",
				     ret);

reschedule:
	mod_delayed_work(system_wq, &ec->curve_work,
			 msecs_to_jiffies(ec->curve_interval_ms));
out:
	mutex_unlock(&ec->mode_lock);
}

static void asus_ec_curve_work(struct work_struct *work)
{
	struct asus_ec *ec = container_of(to_delayed_work(work),
					  struct asus_ec, curve_work);

	asus_ec_curve_apply(ec);
}

static int asus_ec_curve_enable(struct asus_ec *ec)
{
	int ret = 0;

	mutex_lock(&ec->curve_lock);
	mutex_lock(&ec->mode_lock);

	if (ec->curve_recovering) {
		ret = -EBUSY;
		goto out_unlock_mode;
	}

	if (!ec->curve_enabled) {
		if (ec->manual_owner == OWNER_PLATFORM_PROFILE ||
		    ec->manual_owner == OWNER_HWMON) {
			ret = -EBUSY;
			goto out_unlock_mode;
		}

		ret = asus_ec_enter_manual(ec, OWNER_FAN_CURVE);
		if (ret)
			goto out_unlock_mode;

		ec->curve_enabled = true;
		ec->curve_suspended = false;
		ec->curve_recovering = false;
		ec->curve_last_temp = -1;
		ec->curve_last_pwm = 0;
	}

	if (!ec->curve_suspended)
		mod_delayed_work(system_wq, &ec->curve_work, 0);

out_unlock_mode:
	mutex_unlock(&ec->mode_lock);
	mutex_unlock(&ec->curve_lock);
	return ret;
}

static int asus_ec_curve_disable(struct asus_ec *ec)
{
	bool retry_recovery;
	int fallback_ret;
	int ret = 0;

	mutex_lock(&ec->curve_lock);
	mutex_lock(&ec->mode_lock);
	ec->curve_enabled = false;
	ec->curve_suspended = false;
	mutex_unlock(&ec->mode_lock);

	cancel_delayed_work_sync(&ec->curve_work);

	mutex_lock(&ec->mode_lock);
	if (ec->curve_recovering ||
	    ec->manual_owner == OWNER_FAN_CURVE) {
		ret = asus_ec_force_auto(ec);
		if (ret) {
			dev_warn_ratelimited(ec->dev,
					     "fan curve: automatic control disable failed: %d\n",
					     ret);
			ec->curve_recovering = true;
			fallback_ret = asus_ec_set_pwm_both(ec, U8_MAX);
			if (fallback_ret)
				dev_warn_ratelimited(ec->dev,
						     "fan curve: full-speed disable fallback failed: %d\n",
						     fallback_ret);
		} else {
			ec->curve_recovering = false;
		}
	}
	retry_recovery = ec->curve_recovering;
	mutex_unlock(&ec->mode_lock);

	if (retry_recovery)
		mod_delayed_work(system_wq, &ec->curve_work,
				 msecs_to_jiffies(ec->curve_interval_ms));
	mutex_unlock(&ec->curve_lock);

	return ret;
}

static int asus_ec_parse_curve_points(const char *buf, size_t count,
				      int *temps, int *pwms, int *nr_points)
{
	char *data;
	char *cursor;
	char *point;
	char *comma;
	int ret = 0;
	int n = 0;

	data = kstrndup(buf, count, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	cursor = strim(data);
	while ((point = strsep(&cursor, " \t\n\r"))) {
		if (!*point)
			continue;
		if (n == FC_MAX_POINTS) {
			ret = -E2BIG;
			goto out;
		}

		comma = strchr(point, ',');
		if (!comma || strchr(comma + 1, ',')) {
			ret = -EINVAL;
			goto out;
		}
		*comma++ = '\0';

		ret = kstrtoint(point, 0, &temps[n]);
		if (ret)
			goto out;
		ret = kstrtoint(comma, 0, &pwms[n]);
		if (ret)
			goto out;
		if (pwms[n] < 0 || pwms[n] > U8_MAX ||
		    (n && temps[n] <= temps[n - 1])) {
			ret = -EINVAL;
			goto out;
		}

		n++;
	}

	if (n < 2) {
		ret = -EINVAL;
		goto out;
	}

	*nr_points = n;
out:
	kfree(data);
	return ret;
}

static ssize_t fan_curve_enable_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct asus_ec *ec = dev_get_drvdata(dev);
	bool enabled;

	mutex_lock(&ec->mode_lock);
	enabled = ec->curve_enabled;
	mutex_unlock(&ec->mode_lock);

	return sysfs_emit(buf, "%u\n", enabled);
}

static ssize_t fan_curve_enable_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct asus_ec *ec = dev_get_drvdata(dev);
	unsigned int value;
	int ret;

	ret = kstrtouint(buf, 0, &value);
	if (ret)
		return ret;
	if (value > 1)
		return -EINVAL;

	ret = value ? asus_ec_curve_enable(ec) : asus_ec_curve_disable(ec);
	return ret ? ret : count;
}

static ssize_t fan_curve_interval_ms_show(struct device *dev,
					  struct device_attribute *attr, char *buf)
{
	struct asus_ec *ec = dev_get_drvdata(dev);
	unsigned int interval_ms;

	mutex_lock(&ec->mode_lock);
	interval_ms = ec->curve_interval_ms;
	mutex_unlock(&ec->mode_lock);

	return sysfs_emit(buf, "%u\n", interval_ms);
}

static ssize_t fan_curve_interval_ms_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct asus_ec *ec = dev_get_drvdata(dev);
	unsigned int interval_ms;
	int ret;

	ret = kstrtouint(buf, 0, &interval_ms);
	if (ret)
		return ret;

	interval_ms = clamp_val(interval_ms, 100, 30000);
	mutex_lock(&ec->mode_lock);
	ec->curve_interval_ms = interval_ms;
	if (ec->curve_enabled && !ec->curve_suspended)
		mod_delayed_work(system_wq, &ec->curve_work,
				 msecs_to_jiffies(interval_ms));
	mutex_unlock(&ec->mode_lock);

	return count;
}

static ssize_t fan_curve_points_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct asus_ec *ec = dev_get_drvdata(dev);
	int i;
	int len = 0;

	mutex_lock(&ec->mode_lock);
	for (i = 0; i < ec->curve_nr_points; i++)
		len += sysfs_emit_at(buf, len, "%s%d,%d", i ? " " : "",
				     ec->curve_temps[i], ec->curve_pwms[i]);
	mutex_unlock(&ec->mode_lock);

	len += sysfs_emit_at(buf, len, "\n");
	return len;
}

static ssize_t fan_curve_points_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct asus_ec *ec = dev_get_drvdata(dev);
	int temps[FC_MAX_POINTS];
	int pwms[FC_MAX_POINTS];
	int nr_points;
	int ret;

	ret = asus_ec_parse_curve_points(buf, count, temps, pwms, &nr_points);
	if (ret)
		return ret;

	mutex_lock(&ec->mode_lock);
	memcpy(ec->curve_temps, temps, nr_points * sizeof(temps[0]));
	memcpy(ec->curve_pwms, pwms, nr_points * sizeof(pwms[0]));
	ec->curve_nr_points = nr_points;
	ec->curve_last_temp = -1;
	if (ec->curve_enabled && !ec->curve_suspended)
		mod_delayed_work(system_wq, &ec->curve_work, 0);
	mutex_unlock(&ec->mode_lock);

	return count;
}

static ssize_t fan_curve_hysteresis_c_show(struct device *dev,
					    struct device_attribute *attr, char *buf)
{
	struct asus_ec *ec = dev_get_drvdata(dev);
	unsigned int hysteresis_c;

	mutex_lock(&ec->mode_lock);
	hysteresis_c = ec->curve_hysteresis_c;
	mutex_unlock(&ec->mode_lock);

	return sysfs_emit(buf, "%u\n", hysteresis_c);
}

static ssize_t fan_curve_hysteresis_c_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct asus_ec *ec = dev_get_drvdata(dev);
	unsigned int hysteresis_c;
	int ret;

	ret = kstrtouint(buf, 0, &hysteresis_c);
	if (ret)
		return ret;

	mutex_lock(&ec->mode_lock);
	ec->curve_hysteresis_c = clamp_val(hysteresis_c, 0, 10);
	mutex_unlock(&ec->mode_lock);

	return count;
}

static DEVICE_ATTR_RW(fan_curve_enable);
static DEVICE_ATTR_RW(fan_curve_interval_ms);
static DEVICE_ATTR_RW(fan_curve_points);
static DEVICE_ATTR_RW(fan_curve_hysteresis_c);

/*
 * Delayed C6:00 read. The transfer is performed on first read, but only
 * once the machine has been up for C6_DELAY_NS; the per-boot latch makes
 * sure the EC is touched at most once.
 */
static ssize_t c6_read_once_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct asus_ec *ec = dev_get_drvdata(dev);
	u8 raw;
	int ret;

	if (ktime_get_ns() < C6_DELAY_NS)
		return -EAGAIN;

	ret = asus_ec_read_c6_once(ec, &raw);
	if (ret)
		return ret;

	return sysfs_emit(buf, "0x%02x\n", raw);
}

static DEVICE_ATTR_ADMIN_RO(c6_read_once);

static struct attribute *asus_ec_curve_attrs[] = {
	&dev_attr_fan_curve_enable.attr,
	&dev_attr_fan_curve_interval_ms.attr,
	&dev_attr_fan_curve_points.attr,
	&dev_attr_fan_curve_hysteresis_c.attr,
	&dev_attr_c6_read_once.attr,
	NULL,
};

/* The C6:00 attribute is only exposed for the delayed native device. */
static umode_t asus_ec_curve_attr_is_visible(struct kobject *kobj,
					     struct attribute *attr, int n)
{
	const struct device *hwmon_dev = kobj_to_dev(kobj);
	struct asus_ec *ec = dev_get_drvdata(hwmon_dev);

	if (attr == &dev_attr_c6_read_once.attr)
		return ec && ec->c6_delayed ? 0400 : 0;

	return attr->mode;
}

static const struct attribute_group asus_ec_curve_attr_group = {
	.attrs = asus_ec_curve_attrs,
	.is_visible = asus_ec_curve_attr_is_visible,
};

/* ------------------------------------------------------------------ */
/* hwmon callbacks                                                    */
/* ------------------------------------------------------------------ */

static umode_t asus_ec_hwmon_is_visible(const void *drvdata,
					enum hwmon_sensor_types type,
					u32 attr, int channel)
{
	switch (type) {
	case hwmon_fan:
		if (channel >= EC_NUM_FANS)
			return 0;
		switch (attr) {
		case hwmon_fan_input:
		case hwmon_fan_label:
			return 0444;
		default:
			return 0;
		}
	case hwmon_pwm:
		if (channel >= EC_NUM_FANS)
			return 0;
		switch (attr) {
		case hwmon_pwm_input:
			return 0644;
		case hwmon_pwm_enable:
			return channel == 0 ? 0644 : 0;
		default:
			return 0;
		}
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
		case hwmon_temp_label:
			return 0444;
		default:
			return 0;
		}
	default:
		return 0;
	}
}

static int asus_ec_hwmon_read(struct device *dev,
			      enum hwmon_sensor_types type,
			      u32 attr, int channel, long *val)
{
	struct asus_ec *ec = dev_get_drvdata(dev);
	u8 v;
	int ret;

	switch (type) {
	case hwmon_fan:
		if (attr != hwmon_fan_input || channel >= EC_NUM_FANS)
			return -EOPNOTSUPP;
		ret = asus_ec_read_fan_reg(ec, channel, EC_REG_FAN_TACH_MIN, &v);
		if (ret)
			return ret;
		*val = (long)v * EC_TACH_RPM_MULT;
		return 0;

	case hwmon_pwm:
		switch (attr) {
		case hwmon_pwm_input:
			if (channel >= EC_NUM_FANS)
				return -EOPNOTSUPP;
			ret = asus_ec_read_fan_reg(ec, channel, EC_REG_PWM_RMIN, &v);
			if (ret)
				return ret;
			*val = v;
			return 0;
		case hwmon_pwm_enable:
			if (channel != 0)
				return -EOPNOTSUPP;
			ret = asus_ec_read_reg(ec, EC_REG_FAN_MODE_MAJ,
					       EC_REG_FAN_MODE_RMIN, &v);
			if (ret)
				return ret;
			if (v == EC_FAN_MODE_MANUAL)
				*val = 1;
			else if (v == EC_FAN_MODE_AUTO)
				*val = 2;
			else
				*val = 0;
			return 0;
		default:
			return -EOPNOTSUPP;
		}

	case hwmon_temp:
		if (attr != hwmon_temp_input)
			return -EOPNOTSUPP;
		return asus_ec_read_temp_input(ec, val);

	default:
		return -EOPNOTSUPP;
	}
}

static int asus_ec_hwmon_write(struct device *dev,
			       enum hwmon_sensor_types type,
			       u32 attr, int channel, long val)
{
	struct asus_ec *ec = dev_get_drvdata(dev);
	int ret;
	u8 speed;

	if (type != hwmon_pwm)
		return -EOPNOTSUPP;

	switch (attr) {
	case hwmon_pwm_enable:
		mutex_lock(&ec->mode_lock);
		switch (val) {
		case 1:
			if (ec->curve_enabled ||
			    ec->manual_owner == OWNER_PLATFORM_PROFILE ||
			    ec->manual_owner == OWNER_FAN_CURVE)
				ret = -EBUSY;
			else
				ret = asus_ec_enter_manual(ec, OWNER_HWMON);
			break;
		case 2:
			if (ec->manual_owner == OWNER_PLATFORM_PROFILE ||
			    ec->manual_owner == OWNER_FAN_CURVE)
				ret = -EBUSY;
			else
				ret = asus_ec_force_auto(ec);
			break;
		default:
			ret = -EINVAL;
		}
		mutex_unlock(&ec->mode_lock);
		return ret;

	case hwmon_pwm_input:
		if (val < 0 || val > 255)
			return -EINVAL;
		if (channel >= EC_NUM_FANS)
			return -EOPNOTSUPP;
		speed = asus_ec_apply_pwm_floor((u8)val);

		mutex_lock(&ec->mode_lock);
		if (ec->curve_enabled || ec->manual_owner != OWNER_HWMON) {
			mutex_unlock(&ec->mode_lock);
			return -EBUSY;
		}
		ret = asus_ec_set_pwm(ec, channel, speed);
		mutex_unlock(&ec->mode_lock);
		if (ret)
			return ret;

		return 0;

	default:
		return -EOPNOTSUPP;
	}
}

static const char * const fan_labels[] = { "left", "right" };

static int asus_ec_hwmon_read_string(struct device *dev,
				     enum hwmon_sensor_types type,
				     u32 attr, int channel,
				     const char **str)
{
	switch (type) {
	case hwmon_fan:
		if (attr == hwmon_fan_label && channel < EC_NUM_FANS) {
			*str = fan_labels[channel];
			return 0;
		}
		break;
	case hwmon_temp:
		if (attr == hwmon_temp_label) {
			*str = "ec";
			return 0;
		}
		break;
	default:
		break;
	}
	return -EOPNOTSUPP;
}

static const struct hwmon_ops asus_ec_hwmon_ops = {
	.is_visible	= asus_ec_hwmon_is_visible,
	.read		= asus_ec_hwmon_read,
	.write		= asus_ec_hwmon_write,
	.read_string	= asus_ec_hwmon_read_string,
};

static const struct hwmon_channel_info * const asus_ec_hwmon_info[] = {
	HWMON_CHANNEL_INFO(fan,
			   HWMON_F_INPUT | HWMON_F_LABEL,
			   HWMON_F_INPUT | HWMON_F_LABEL),
	HWMON_CHANNEL_INFO(pwm,
			   HWMON_PWM_INPUT | HWMON_PWM_ENABLE,
			   HWMON_PWM_INPUT),
	HWMON_CHANNEL_INFO(temp,
			   HWMON_T_INPUT | HWMON_T_LABEL),
	NULL
};

static const struct hwmon_chip_info asus_ec_hwmon_chip_info = {
	.ops	= &asus_ec_hwmon_ops,
	.info	= asus_ec_hwmon_info,
};

/* ------------------------------------------------------------------ */
/* SoC temperature feed                                               */
/* ------------------------------------------------------------------ */

static const char * const asus_ec_tj_zones[] = {
	"cpuss0-top-thermal", "cpuss0-btm-thermal",
	"cpuss1-top-thermal", "cpuss1-btm-thermal",
	"mem-thermal",
	"nsp0-thermal", "nsp1-thermal", "nsp2-thermal", "nsp3-thermal",
	"gpuss-0-thermal", "gpuss-1-thermal",
	"gpuss-2-thermal", "gpuss-3-thermal",
	"cpu0-0-top-thermal", "cpu0-0-btm-thermal",
	"cpu0-1-top-thermal", "cpu0-1-btm-thermal",
	"cpu0-2-top-thermal", "cpu0-2-btm-thermal",
	"cpu0-3-top-thermal", "cpu0-3-btm-thermal",
	"cpu1-0-top-thermal", "cpu1-0-btm-thermal",
	"cpu1-1-top-thermal", "cpu1-1-btm-thermal",
	"cpu1-2-top-thermal", "cpu1-2-btm-thermal",
	"cpu1-3-top-thermal", "cpu1-3-btm-thermal",
};

/* Hottest readable SoC zone in whole degrees C; -ENODATA if none reads. */
static int asus_ec_soc_tj(int *tj_c)
{
	int best = INT_MIN;
	int i;

	for (i = 0; i < ARRAY_SIZE(asus_ec_tj_zones); i++) {
		struct thermal_zone_device *tz;
		int temp;

		tz = thermal_zone_get_zone_by_name(asus_ec_tj_zones[i]);
		if (IS_ERR(tz) || thermal_zone_get_temp(tz, &temp))
			continue;
		best = max(best, temp);
	}
	if (best == INT_MIN)
		return -ENODATA;

	*tj_c = best / 1000;
	return 0;
}

static void asus_ec_tj_work(struct work_struct *work)
{
	struct asus_ec *ec = container_of(to_delayed_work(work),
					  struct asus_ec, tj_work);
	u8 val = EC_TJ_SAFE;
	int tj, ret;

	if (!asus_ec_soc_tj(&tj))
		val = clamp(tj, EC_TJ_MIN_C, EC_TJ_MAX_C);

	mutex_lock(&ec->lock);
	ret = __ec_wb(ec, EC_TJ_MAJ, EC_TJ_MIN, val);
	mutex_unlock(&ec->lock);

	if (!ret) {
		ec->tj_fail = 0;
	} else if (++ec->tj_fail >= EC_TJ_MAX_FAIL) {
		dev_warn(ec->dev, "Tj feed stopped after %u write failures: %d\n",
			 ec->tj_fail, ret);
		return;
	}

	queue_delayed_work(system_wq, &ec->tj_work,
			   msecs_to_jiffies(EC_TJ_INTERVAL_MS));
}

static void asus_ec_tj_start(struct asus_ec *ec)
{
	if (!tj_feed ||
	    !of_device_is_compatible(ec->dev->of_node,
				     "asus,zenbook-a14-ux3407qa-ec") ||
	    ec->client->addr != EC_I2C_ADDR)
		return;

	dev_info_once(ec->dev, "Tj feed enabled\n");
	ec->tj_fail = 0;
	ec->tj_started = true;
	queue_delayed_work(system_wq, &ec->tj_work, 0);
}

/* Stop the feed and put the EC default back; safe to call repeatedly. */
static void asus_ec_tj_stop(struct asus_ec *ec)
{
	if (!ec->tj_started)
		return;

	cancel_delayed_work_sync(&ec->tj_work);

	mutex_lock(&ec->lock);
	(void)__ec_wb(ec, EC_TJ_MAJ, EC_TJ_MIN, EC_TJ_SAFE);
	mutex_unlock(&ec->lock);
	ec->tj_started = false;
}

/* ------------------------------------------------------------------ */
/* probe / remove                                                     */
/* ------------------------------------------------------------------ */

static int asus_zenbook_a14_ec_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct asus_ec *ec;
	struct device *hwmon_dev;
	u8 mode;
	int ret;

	ec = devm_kzalloc(dev, sizeof(*ec), GFP_KERNEL);
	if (!ec)
		return -ENOMEM;

	ec->dev = dev;
	ec->client = client;
	mutex_init(&ec->lock);
	mutex_init(&ec->mode_lock);
	mutex_init(&ec->curve_lock);
	INIT_DELAYED_WORK(&ec->curve_work, asus_ec_curve_work);
	INIT_DELAYED_WORK(&ec->tj_work, asus_ec_tj_work);
	ec->curve_interval_ms = FC_DEFAULT_INTERVAL_MS;
	ec->curve_hysteresis_c = FC_DEFAULT_HYSTERESIS_C;
	ec->curve_nr_points = 5;
	ec->curve_temps[0] = 35;
	ec->curve_pwms[0] = 0;
	ec->curve_temps[1] = 45;
	ec->curve_pwms[1] = 60;
	ec->curve_temps[2] = 55;
	ec->curve_pwms[2] = 120;
	ec->curve_temps[3] = 65;
	ec->curve_pwms[3] = 180;
	ec->curve_temps[4] = 75;
	ec->curve_pwms[4] = 255;
	ec->curve_last_temp = -1;
	i2c_set_clientdata(client, ec);

	if (of_device_is_compatible(dev->of_node, "asus,zenbook-a14-ux3407qa-ec") &&
	    client->addr == EC_I2C_ADDR) {
		if (read_c6_delayed) {
			/* Delayed mode: the read is triggered from sysfs. */
			ec->c6_delayed = true;
		} else if (read_c6_once) {
			u8 raw;

			/* One direct read per boot only. */
			ret = asus_ec_read_c6_once(ec, &raw);
			if (ret == -EALREADY)
				return -ENODEV;
			if (ret)
				return -EIO;
		}
	}

	ret = asus_ec_read_reg(ec, EC_REG_FAN_MODE_MAJ, EC_REG_FAN_MODE_RMIN,
			       &mode);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read fan mode\n");

	if (mode == EC_FAN_MODE_MANUAL) {
		dev_warn(dev, "EC found in MANUAL mode at probe; forcing AUTO for safety\n");
		mutex_lock(&ec->mode_lock);
		ret = asus_ec_force_auto(ec);
		mutex_unlock(&ec->mode_lock);
		if (ret)
			return dev_err_probe(dev, ret, "failed to restore automatic fan mode\n");
	}

	hwmon_dev = devm_hwmon_device_register_with_info(dev, HWMON_NAME, ec,
						  &asus_ec_hwmon_chip_info, NULL);
	if (IS_ERR(hwmon_dev)) {
		ret = PTR_ERR(hwmon_dev);
		dev_err(dev, "hwmon registration failed: %d\n", ret);
		return ret;
	}

	ec->hwmon_dev = hwmon_dev;
	ret = sysfs_create_group(&hwmon_dev->kobj, &asus_ec_curve_attr_group);
	if (ret)
		return dev_err_probe(dev, ret, "fan curve sysfs registration failed\n");

	dev_info(dev, "hwmon registered\n");

	asus_ec_freq_qos_init(ec);
	ec->pp_active = PLATFORM_PROFILE_BALANCED;
	ec->profile_dev = devm_platform_profile_register(dev, DRV_NAME, ec,
					    &asus_ec_pp_ops);
	if (IS_ERR(ec->profile_dev)) {
		dev_warn(dev, "platform_profile registration failed: %ld (continuing)\n",
			 PTR_ERR(ec->profile_dev));
		ec->profile_dev = NULL;
	} else {
		dev_info(dev,
			 "platform_profile registered (quiet/balanced/performance)\n");
	}

	asus_ec_tj_start(ec);

	return 0;
}

static void asus_zenbook_a14_ec_remove(struct i2c_client *client)
{
	struct asus_ec *ec = i2c_get_clientdata(client);

	if (!ec)
		return;

	asus_ec_tj_stop(ec);
	sysfs_remove_group(&ec->hwmon_dev->kobj, &asus_ec_curve_attr_group);
	(void)asus_ec_curve_disable(ec);
	cancel_delayed_work_sync(&ec->curve_work);

	asus_ec_freq_qos_set(ec, PP_MAX_FREQ_KHZ);
	asus_ec_freq_qos_cleanup(ec);

	dev_info(&client->dev, "removed\n");
}

static void asus_zenbook_a14_ec_shutdown(struct i2c_client *client)
{
	struct asus_ec *ec = i2c_get_clientdata(client);

	if (!ec)
		return;

	asus_ec_tj_stop(ec);
	(void)asus_ec_curve_disable(ec);
	cancel_delayed_work_sync(&ec->curve_work);
}

/* ------------------------------------------------------------------ */
/* PM                                                                 */
/* ------------------------------------------------------------------ */

static int asus_ec_prepare(struct device *dev)
{
	struct asus_ec *ec = i2c_get_clientdata(to_i2c_client(dev));
	int ret;

	if (!ec)
		return 0;

	asus_ec_tj_stop(ec);

	mutex_lock(&ec->curve_lock);
	mutex_lock(&ec->mode_lock);
	ec->suspend_prepared = false;
	ret = asus_ec_save_suspend_state(ec);
	if (ret)
		goto out_unlock_mode;

	ec->curve_suspended = ec->curve_enabled;
	mutex_unlock(&ec->mode_lock);

	cancel_delayed_work_sync(&ec->curve_work);

	mutex_lock(&ec->mode_lock);
	ret = asus_ec_force_auto(ec);
	if (ret) {
		ec->curve_suspended = false;
		if (ec->curve_enabled)
			mod_delayed_work(system_wq, &ec->curve_work, 0);
		goto out_unlock_mode;
	}
	if (ec->curve_recovering) {
		ec->suspend_state.owner = OWNER_NONE;
		ec->suspend_state.curve_enabled = false;
		ec->curve_enabled = false;
		ec->curve_suspended = false;
		ec->curve_recovering = false;
	}

	ec->suspend_prepared = true;

out_unlock_mode:
	mutex_unlock(&ec->mode_lock);
	mutex_unlock(&ec->curve_lock);

	if (ret)
		asus_ec_tj_start(ec);

	return ret;
}

static int asus_ec_suspend(struct device *dev)
{
	return 0;
}

static int asus_ec_resume(struct device *dev)
{
	struct asus_ec *ec = i2c_get_clientdata(to_i2c_client(dev));
	int ret;

	if (!ec)
		return 0;

	mutex_lock(&ec->curve_lock);
	mutex_lock(&ec->mode_lock);
	if (!ec->suspend_prepared)
		goto out_unlock;

	ret = asus_ec_restore_suspend_state(ec);
	if (ret) {
		dev_warn(dev, "resume: restore state failed: %d\n", ret);
		asus_ec_suspend_fallback_auto(ec);
	}

	ret = asus_ec_freq_qos_set(ec,
			ec->pp_active == PLATFORM_PROFILE_QUIET ?
			PP_QUIET_FREQ_KHZ : PP_MAX_FREQ_KHZ);
	if (ret)
		dev_warn(dev, "resume: restore frequency cap failed: %d\n", ret);

	ec->suspend_prepared = false;

out_unlock:
	mutex_unlock(&ec->mode_lock);
	mutex_unlock(&ec->curve_lock);

	return 0;
}

static void asus_ec_complete(struct device *dev)
{
	struct asus_ec *ec = i2c_get_clientdata(to_i2c_client(dev));
	int ret;

	if (!ec)
		return;

	mutex_lock(&ec->curve_lock);
	mutex_lock(&ec->mode_lock);

	if (ec->suspend_prepared) {
		ret = asus_ec_restore_suspend_state(ec);
		if (ret) {
			dev_warn(dev, "complete: restore state failed: %d\n", ret);
			asus_ec_suspend_fallback_auto(ec);
		}
		ec->suspend_prepared = false;
	}

	if (ec->curve_recovering ||
	    (ec->curve_enabled && ec->curve_suspended)) {
		ec->curve_suspended = false;
		ec->curve_last_temp = -1;
		mod_delayed_work(system_wq, &ec->curve_work, 0);
	}

	mutex_unlock(&ec->mode_lock);
	mutex_unlock(&ec->curve_lock);

	asus_ec_tj_start(ec);
}

static const struct dev_pm_ops asus_zenbook_a14_ec_pm_ops = {
	.prepare	= asus_ec_prepare,
	.suspend	= asus_ec_suspend,
	.resume		= asus_ec_resume,
	.complete	= asus_ec_complete,
};

static const struct of_device_id asus_zenbook_a14_ec_of_match[] = {
	{ .compatible = "asus,zenbook-a14-ux3407qa-ec" },
	{ }
};
MODULE_DEVICE_TABLE(of, asus_zenbook_a14_ec_of_match);

static const struct i2c_device_id asus_zenbook_a14_ec_id[] = {
	{ "asus-zenbook-a14-ec", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, asus_zenbook_a14_ec_id);

static struct i2c_driver asus_zenbook_a14_ec_driver = {
	.driver = {
		.name = DRV_NAME,
		.of_match_table = asus_zenbook_a14_ec_of_match,
		.pm = &asus_zenbook_a14_ec_pm_ops,
	},
	.probe = asus_zenbook_a14_ec_probe,
	.remove = asus_zenbook_a14_ec_remove,
	.shutdown = asus_zenbook_a14_ec_shutdown,
	.id_table = asus_zenbook_a14_ec_id,
};
module_i2c_driver(asus_zenbook_a14_ec_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ASUS Zenbook A14 UX3407QA EC driver");
MODULE_AUTHOR("Sombre-Osmoze");
MODULE_ALIAS("i2c:asus-zenbook-a14-ec");
