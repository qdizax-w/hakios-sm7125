/*
 * The battery of miatoll (Redmi Note 9S and its siblings, SM7125), for iOS: the PM6150's fuel gauge ("QG",
 * SPMI slave 0, registers 0x4800; dt: qcom,pm6150@0 qpnp,qg qcom,qgauge@4800), the memory Android's gauge
 * driver keeps its state in (its SDAM, 0xb600: qcom,qg-sdam@b600) and the PM6150's charger (SMB5: 0x1000 the
 * charger, 0x1200 the battery interface, 0x1300 the USB input). Register layouts: MiCode curtana-q-oss
 * drivers/power/supply/qcom/qg-reg.h, qg-util.c, qg-defs.h, smb5-reg.h.
 *
 * Nothing is written: not to the gauge, not to its SDAM, not to the charger -- but for one thing, and only with
 * boot-arg charge=1: the charger's watchdog petted (see pet()), without which it stops charging a few minutes
 * into the run. Every other register below is only read, through the arbiter's observer channel, one transfer
 * at a time with the buttons' driver (hakios_spmi_lock()). How the charger charges is as the firmware set it.
 *
 * The QG is not a gauge that knows a percentage: it measures (the battery's voltage, QG_LAST_ADC_V; its
 * current, QG_LAST_ADC_I, positive while it discharges), and Android's driver makes the percentage out of
 * those and a model of the battery, and keeps it in the SDAM (QG_SDAM_SOC, valid with QG_SDAM_VALID) where
 * the next boot finds it. So here, with that model's own data, this cell's open-circuit voltage curve from the
 * device tree (ocv_mv below):
 *
 *   - the start: 4 s of voltage and current, the load's drop taken out, read off the curve (Android's SDAM
 *     percentage is said, not used: it is from before the phone sat in fastboot);
 *   - then, once a second: the charge counted (the current times the time) out of the capacity Android learnt
 *     (QG_SDAM_LEARNED_CAPACITY, mAh) or the profile's 5077 mAh, and pulled slowly towards the curve's charge
 *     for the last 30 s of voltage (see battery_thread for how slowly, and where not);
 *   - shown so that it does not wobble against the way the current flows; 100 when the charger has finished.
 *
 * Boot-arg qg=1: on; qg=2: and a line of what was read every 5 s, not every minute.
 */
#include <kern/clock.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.input.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern vm_offset_t pe_arm_get_soc_base_phys(void);
extern kern_return_t thread_terminate(thread_t thread);
extern void loader_log_putc(char c);

static int say(const char *format, ...) __printflike(1, 2);

/* every line to the loader's log as well, which keeps it when the console's is long (as hakios.torch.c does) */
static int
say(const char *format, ...)
{
	char line[300];
	va_list list;
	int length;

	va_start(list, format);
	length = vsnprintf(line, sizeof(line), format, list);
	va_end(list);
	for (const char *c = line; *c != 0; c++) {
		loader_log_putc(*c);
	}
	printf("%s", line);
	return length;
}

#define MIATOLL_SOC_BASE        0x00100000ul

/* ---- the PMIC arbiter (version 5), reads only ---------------------------------------------------- */

#define ARB_CORE_PHYS           0x0c440000UL
#define ARB_CORE_SIZE           0x1100UL
#define ARB_OBSERVER_PHYS       0x0e600000UL
#define ARB_OBSERVER_SIZE       0x10000UL
#define ARB_SYSTEM              0
#define ARB_VERSION             0x000
#define ARB_VERSION_5           0x50000000u
#define ARB_VERSION_7           0x70000000u
#define ARB_FEATURES            0x004
#define ARB_CHANNEL(n)          (0x900 + 4 * (n))
#define ARB_MAX_CHANNELS        512

#define CHANNEL_COMMAND         0x00
#define CHANNEL_STATUS          0x08
#define OBSERVER_DATA0          0x18
#define STATUS_DONE             0x1
#define STATUS_FAILURE          0x2
#define STATUS_DENIED           0x4
#define STATUS_DROPPED          0x8
#define COMMAND_READ            (1u << 27)
#define TRANSFER_TIMEOUT_US     2000

struct peripheral {
	const char *    name;
	uint16_t        id;             /* slave << 8 | peripheral */
	uint16_t        channel;
	bool            found;
};
static struct peripheral qg = { .name = "the PM6150's fuel gauge", .id = 0x048 };
static struct peripheral sdam = { .name = "the PM6150's SDAM of the gauge", .id = 0x0b6 };
static struct peripheral chgr = { .name = "the PM6150's charger", .id = 0x010 };
static struct peripheral batif = { .name = "the PM6150's battery interface", .id = 0x012 };
static struct peripheral usbin = { .name = "the PM6150's USB input", .id = 0x013 };
static struct peripheral misc = { .name = "the PM6150's charger misc block (its watchdog)", .id = 0x016 };

static vm_offset_t arb_core, arb_observer;
static uint32_t level;

static inline uint32_t
rd(vm_offset_t address)
{
	return *(volatile uint32_t *)address;
}

static inline void
wr(vm_offset_t address, uint32_t value)
{
	*(volatile uint32_t *)address = value;
}

static const char *
read_once(const struct peripheral *p, uint8_t reg, uint8_t *bytes, uint32_t count)
{
	vm_offset_t channel = arb_observer + 0x10000UL * ARB_SYSTEM + 0x80UL * p->channel;
	uint64_t start, interval;
	uint32_t status, data;

	if (!p->found) {
		return "no channel";
	}
	nanoseconds_to_absolutetime(TRANSFER_TIMEOUT_US * (uint64_t)NSEC_PER_USEC, &interval);
	hakios_spmi_lock();
	wr(channel + CHANNEL_COMMAND, COMMAND_READ | (uint32_t)reg << 4 | (count - 1));
	start = mach_absolute_time();
	do {
		status = rd(channel + CHANNEL_STATUS);
	} while (!(status & STATUS_DONE) && mach_absolute_time() - start < interval);
	data = rd(channel + OBSERVER_DATA0);
	hakios_spmi_unlock();
	if (!(status & STATUS_DONE)) {
		return "no answer from the arbiter";
	}
	if (status & (STATUS_DENIED | STATUS_FAILURE | STATUS_DROPPED)) {
		return (status & STATUS_DENIED) ? "the arbiter refuses" : (status & STATUS_FAILURE) ? "the transfer failed" :
		       "the transfer was dropped";
	}
	for (uint32_t i = 0; i < count; i++) {
		bytes[i] = (uint8_t)(data >> (8 * i));
	}
	return NULL;
}

/* `count' (1 to 4) registers from `reg' on, a dropped transfer asked for again twice; false if they cannot be read */
static bool
get(const struct peripheral *p, uint8_t reg, uint8_t *bytes, uint32_t count)
{
	const char *problem = NULL;

	for (int tries = 0; tries < 3; tries++) {
		if ((problem = read_once(p, reg, bytes, count)) == NULL) {
			return true;
		}
	}
	if (level >= 2) {
		say("qg: reading %#04x of %s: %s\n", reg, p->name, problem);
	}
	return false;
}

static void
find_channel(struct peripheral *p, uint32_t channels)
{
	for (uint32_t n = 0; n < channels; n++) {
		uint32_t map = rd(arb_core + ARB_CHANNEL(n));

		if (map != 0 && ((map >> 8) & 0xfff) == p->id) {
			p->found = true;
			p->channel = (uint16_t)n;
			return;
		}
	}
	say("qg: no channel of the arbiter is %s (slave %u, peripheral %#x)\n", p->name, p->id >> 8, p->id & 0xff);
}

/* ---- charge=1: the charger's watchdog petted, the one write of this file ---------------------------- */

/*
 * The charger (SMB5) has a watchdog that the firmware leaves on whenever something is plugged in (MISC 0x51
 * bit 1, WDOG_TIMER_EN_ON_PLUGIN) and whose bite stops charging (MISC 0x53 bit 7,
 * BITE_WDOG_DISABLE_CHARGING_CFG): it is there so that a charger nobody looks after stops. Android's driver
 * looks after it; under this kernel nobody does, and a few minutes after the boot the bite stops the charging
 * (seen: the charger going from "full on" to "disabled" with the phone plugged in, ~200 mA out of the battery).
 *
 * With boot-arg charge=1 (and only then) the watchdog is petted every 5 s while something is plugged in:
 * BARK_BITE_WDOG_PET (MISC 0x43) bit 0, written 1 -- smb5-reg.h; the register does nothing else. That is all
 * that is written. What the charger then does -- its voltage, its current, its temperature limits, when it
 * stops -- is all as the firmware and the hardware have it; nothing of that is set from here. The petting
 * stops, and the watchdog bites again as it would, if a write is refused or the charger reports a battery
 * temperature out of range or an error.
 */
#define ARB_CNFG_PHYS           0x0c40a000UL
#define ARB_CNFG_SIZE           0x1000UL
#define ARB_CHNLS_PHYS          0x0c600000UL
#define ARB_OWNER(n)            (0x700 + 4 * (n))
#define CHANNEL_WDATA0          0x10
#define COMMAND_WRITE           (0u << 27)
#define MISC_WDOG_PET           0x43

static uint32_t charge_level;
static vm_offset_t misc_write;
static uint16_t misc_write_channel;
static bool petting_stopped;

static void
pet_setup(uint32_t channels)
{
	vm_offset_t cnfg = ml_io_map(ARB_CNFG_PHYS, ARB_CNFG_SIZE);

	for (uint32_t n = 0; n < channels; n++) {
		uint32_t map = rd(arb_core + ARB_CHANNEL(n));

		if (map != 0 && ((map >> 8) & 0xfff) == 0x016 && (rd(cnfg + ARB_OWNER(n)) & 7) == ARB_SYSTEM) {
			misc_write_channel = (uint16_t)n;
			misc_write = ml_io_map(ARB_CHNLS_PHYS + 0x10000UL * n, 0x1000);
			say("qg: charge=1: the charger's watchdog will be petted (channel %u, MISC %#x)\n", n, MISC_WDOG_PET);
			return;
		}
	}
	say("qg: charge=1, but no channel of the charger's misc block is ours to write through: nothing petted\n");
}

static void
stop_petting(const char *why)
{
	if (!petting_stopped) {
		petting_stopped = true;
		say("qg: the charger's watchdog is no longer petted: %s\n", why);
	}
}

static void
pet(void)
{
	vm_offset_t channel = misc_write;
	uint64_t start, interval;
	uint32_t status;

	if (misc_write == 0 || petting_stopped) {
		return;
	}
	if (((rd(arb_core + ARB_CHANNEL(misc_write_channel)) >> 8) & 0xfff) != 0x016) {
		stop_petting("the arbiter's channel is no longer the charger's misc block");
		return;
	}
	nanoseconds_to_absolutetime(TRANSFER_TIMEOUT_US * (uint64_t)NSEC_PER_USEC, &interval);
	hakios_spmi_lock();
	wr(channel + CHANNEL_WDATA0, 0x01);
	wr(channel + CHANNEL_COMMAND, COMMAND_WRITE | (uint32_t)MISC_WDOG_PET << 4);
	start = mach_absolute_time();
	do {
		status = rd(channel + CHANNEL_STATUS);
	} while (!(status & STATUS_DONE) && mach_absolute_time() - start < interval);
	hakios_spmi_unlock();
	if (!(status & STATUS_DONE) || (status & (STATUS_DENIED | STATUS_FAILURE))) {
		stop_petting(!(status & STATUS_DONE) ? "no answer from the arbiter" : "the write was refused");
	}
}

/* ---- the gauge ------------------------------------------------------------------------------------ */

#define QG_TYPE_REG             0x04            /* 0x0d: a QG */
#define QG_SUBTYPE_REG          0x05            /* 0x03: the 5 A current range, 0x04 the 10 A one (qg_iraw_to_ua) */
#define QG_STATUS1_REG          0x08            /* [0] a battery is there */
#define QG_LAST_ADC_V_DATA0     0xc0            /* 2 bytes: 194.637 uV a step */
#define QG_LAST_ADC_I_DATA0     0xc2            /* 2 bytes, signed: 152.588 or 305.176 uA a step; > 0 discharging */

#define SDAM_VALID              0x46
#define SDAM_SOC                0x47            /* percent */
#define SDAM_TEMP               0x48            /* 2 bytes, tenths of a degree C, when Android last wrote it */
#define SDAM_LEARNED_CAPACITY   0x68            /* 2 bytes, mAh */

#define CHGR_STATUS1            0x06            /* [2:0]: 0 inhibit, 1 trickle, 2 pre, 3 full on, 4 taper, 5 done, 6 pause, 7 off */
#define BATIF_INT_RT_STS        0x10            /* [5] terminal missing, [4] therm or id missing, [0] temperature out of range */
#define USBIN_INT_RT_STS        0x10            /* [4] something plugged in */

#define DESIGN_UAH              5020000         /* dt: qcom,battery-type "..._5020mAH_..." */

static uint8_t qg_subtype;

/*
 * This battery's own open-circuit voltage against its charge: the profile in the phone's device tree (dtbo,
 * qcom,battery-data, qcom,SUNWODA_5020mAh qcom,pc-temp-v1-lut, the 25 C column, 0.1 mV units), which Android's
 * gauge works from. The phone's other cells' profiles (ATL, NVT, J6B's) are within 5 mV of it everywhere; which
 * cell is in this phone is told by a resistor (batt-id) that only the ADC reads, so this one stands for all.
 * Charge in tenths of a percent, the voltage in mV, from full down.
 */
static const uint16_t ocv_pm[] = {
	1000, 980, 960, 940, 920, 900, 880, 860, 840, 820, 800, 780, 760, 740, 720, 700, 680, 660, 640, 620, 600, 580, 560,
	540, 520, 500, 480, 460, 440, 420, 400, 380, 360, 340, 320, 300, 280, 260, 240, 220, 200, 180, 160, 140, 120, 100,
	90, 80, 70, 60, 50, 40, 30, 20, 10, 0,
};
static const uint16_t ocv_mv[] = {
	4435, 4413, 4392, 4371, 4350, 4329, 4305, 4280, 4254, 4230, 4205, 4181, 4157, 4134, 4111, 4090, 4069, 4048, 4029,
	4011, 3993, 3971, 3947, 3926, 3905, 3888, 3877, 3867, 3859, 3850, 3842, 3835, 3828, 3822, 3817, 3812, 3807, 3802,
	3795, 3786, 3776, 3764, 3751, 3734, 3719, 3710, 3707, 3705, 3703, 3698, 3673, 3629, 3570, 3489, 3361, 3000,
};
#define OCV_POINTS              (sizeof(ocv_mv) / sizeof(ocv_mv[0]))

/* the charge (tenths of a percent) the cell has at rest at `mv'; *slope: mV per point of charge there */
static int
ocv_to_pm(int mv, int *slope)
{
	*slope = 20;
	if (mv >= ocv_mv[0]) {
		return 1000;
	}
	for (unsigned i = 1; i < OCV_POINTS; i++) {
		if (mv >= ocv_mv[i]) {
			int dmv = ocv_mv[i - 1] - ocv_mv[i], dpm = ocv_pm[i - 1] - ocv_pm[i];

			*slope = dmv * 10 / dpm;
			return ocv_pm[i] + dpm * (mv - ocv_mv[i]) / dmv;
		}
	}
	return 0;
}

/* the profile's capacity at 25 C (qcom,fcc1-temp-lut), when Android has not learnt one */
#define PROFILE_UAH             5077000
/* the cell's and the path's resistance (ESR plus the board), the voltage's drop under load: rest = V - I x R */
#define RESISTANCE_MOHM         130

struct reading {
	bool    ok, present, plugged, temp_bad;
	int     uv, ua;                 /* ua: > 0 while charging, as struct hakios_battery_state has it */
	uint8_t charger;
};

/*
 * What the charger is set to and says, all of it read only: why it charges or not. CHGR 0x07 (errors) 0x0b (which
 * modes are enabled) 0x0d (the battery's temperature band) 0x42 (the charging enable command); MISC 0x10 (its
 * interrupts now: the watchdog's bark and bite among them) 0x51 (the watchdog on, on plug-in) 0x53 (whether a
 * bite stops charging). smb5-reg.h.
 */
static void
charger_report(const char *why)
{
	uint8_t s2 = 0, s5 = 0, s7 = 0, en = 0, misc_rt = 0, wd = 0, bite = 0;

	(void)get(&chgr, 0x07, &s2, 1);
	(void)get(&chgr, 0x0b, &s5, 1);
	(void)get(&chgr, 0x0d, &s7, 1);
	(void)get(&chgr, 0x42, &en, 1);
	(void)get(&misc, 0x10, &misc_rt, 1);
	(void)get(&misc, 0x51, &wd, 1);
	(void)get(&misc, 0x53, &bite, 1);
	say("qg: charger (%s): errors %#04x, modes %#04x, temperature band %#04x, enable command %#04x; misc interrupts %#04x, "
	    "watchdog config %#04x, bite config %#04x\n", why, s2, s5, s7, en, misc_rt, wd, bite);
}

static struct reading
measure(void)
{
	struct reading r = { 0 };
	uint8_t v[2], i[2], status1 = 0, b = 0, u = 0, c = 0;
	int raw_i;

	if (!get(&qg, QG_STATUS1_REG, &status1, 1) || !get(&qg, QG_LAST_ADC_V_DATA0, v, 2) || !get(&qg, QG_LAST_ADC_I_DATA0, i, 2)) {
		return r;
	}
	(void)get(&chgr, CHGR_STATUS1, &c, 1);
	(void)get(&batif, BATIF_INT_RT_STS, &b, 1);
	(void)get(&usbin, USBIN_INT_RT_STS, &u, 1);
	raw_i = (int16_t)(i[0] | i[1] << 8);
	r.ok = true;
	r.present = (status1 & 0x01) && !(b & 0x20);
	r.uv = (int)((194637LL * (v[0] | v[1] << 8)) / 1000);
	r.ua = -(int)(((qg_subtype == 0x03 ? 152588LL : 305176LL) * raw_i) / 1000);
	r.plugged = (u & 0x10) != 0;
	r.temp_bad = (b & 0x01) != 0;
	r.charger = c & 0x07;
	return r;
}

static void
battery_thread(__unused void *parameter, __unused wait_result_t result)
{
	uint32_t version, channels;
	uint8_t type[2] = { 0, 0 }, valid = 0, soc = 0, cap[2] = { 0, 0 }, temp[2] = { 0, 0 };
	int64_t charge_uah, full_uah = PROFILE_UAH;
	int64_t last_ns = 0, sum_uv = 0, sum_ua = 0;
	int android_dc, start_pm, slope, shown_pm, avg_uv, avg_ua;
	unsigned samples = 0;
	uint8_t last_state;
	struct reading r;
	/* the last 30 s of voltage and current, for the correction (a single conversion is one instant's) */
	static int window_uv[30], window_ua[30];

	arb_core = ml_io_map(ARB_CORE_PHYS, ARB_CORE_SIZE);
	arb_observer = ml_io_map(ARB_OBSERVER_PHYS, ARB_OBSERVER_SIZE);
	version = rd(arb_core + ARB_VERSION);
	channels = rd(arb_core + ARB_FEATURES) & 0x7ff;
	if (version < ARB_VERSION_5 || version >= ARB_VERSION_7) {
		say("qg: PMIC arbiter version %#x, not 5: nothing done\n", version);
		goto out;
	}
	if (channels == 0 || channels > ARB_MAX_CHANNELS) {
		channels = ARB_MAX_CHANNELS;
	}
	find_channel(&qg, channels);
	find_channel(&sdam, channels);
	find_channel(&chgr, channels);
	find_channel(&batif, channels);
	find_channel(&usbin, channels);
	find_channel(&misc, channels);
	if (charge_level != 0) {
		pet_setup(channels);
	}
	if (!get(&qg, QG_TYPE_REG, type, 2) || type[0] != 0x0d) {
		say("qg: the gauge says type %#x subtype %#x: not a QG, nothing done\n", type[0], type[1]);
		goto out;
	}
	qg_subtype = type[1];
	if (get(&sdam, SDAM_LEARNED_CAPACITY, cap, 2) && (cap[0] | cap[1] << 8) >= 2500 && (cap[0] | cap[1] << 8) <= 6000) {
		full_uah = (int64_t)(cap[0] | cap[1] << 8) * 1000;
	}
	(void)get(&sdam, SDAM_TEMP, temp, 2);
	(void)get(&sdam, SDAM_VALID, &valid, 1);
	(void)get(&sdam, SDAM_SOC, &soc, 1);
	/*
	 * The start: the voltage and current over 4 s, the load's drop taken out, read off the cell's own curve.
	 * (Android's SDAM percentage is only said: it is what Android had when it last wrote it, before the phone
	 * sat in fastboot and booted this, and it read 9 points off the voltage on the first run here.)
	 */
	for (int n = 0; n < 8; n++) {
		r = measure();
		if (r.ok) {
			sum_uv += r.uv;
			sum_ua += r.ua;
			samples++;
		}
		delay_for_interval(500, NSEC_PER_MSEC);
	}
	if (samples == 0) {
		say("qg: the gauge cannot be read: nothing done\n");
		goto out;
	}
	avg_uv = (int)(sum_uv / samples);
	avg_ua = (int)(sum_ua / samples);
	start_pm = ocv_to_pm(avg_uv / 1000 - avg_ua / 1000 * RESISTANCE_MOHM / 1000, &slope);
	charge_uah = full_uah * start_pm / 1000;
	shown_pm = start_pm;
	for (unsigned i = 0; i < 30; i++) {
		window_uv[i] = avg_uv;
		window_ua[i] = avg_ua;
	}
	android_dc = (int16_t)(temp[0] | temp[1] << 8);
	say("qg: the PM6150's QG (subtype %#x): %d mV, %d mA over 4 s; at rest %d mV: %d.%d%% on this cell's curve; "
	    "Android left %s%u%% at %d.%d C; capacity %lld mAh (%s)\n", qg_subtype, avg_uv / 1000, avg_ua / 1000,
	    avg_uv / 1000 - avg_ua / 1000 * RESISTANCE_MOHM / 1000, start_pm / 10, start_pm % 10, valid == 1 ? "" : "(not valid) ",
	    soc, android_dc / 10, android_dc < 0 ? -android_dc % 10 : android_dc % 10, full_uah / 1000,
	    full_uah == PROFILE_UAH ? "the profile's" : "learnt by Android");
	charger_report("at the start");
	last_state = r.charger;

	/*
	 * Once a second, as a gauge does it:
	 *   - the charge counted: the current measured times the time (the QG's last conversion is one instant, so
	 *     it is taken often);
	 *   - and pulled towards what the voltage says, slowly: the last 30 s of voltage and current averaged, the
	 *     load's drop taken out, read off the cell's curve, and the count moved 1/600 of the way there each
	 *     second (a time constant of 10 minutes) -- 1/3000 where the curve is flat (under 3 mV a point: 20-50 %
	 *     and 6-10 %), where a few mV of error are many points, and not at all above 1 A, where the drop is
	 *     larger than the resistance above can say;
	 *   - shown without wobbling: while discharging the percentage shown does not go up, while charging it does
	 *     not go down; it follows the count when that has moved more than a point the other way;
	 *   - the charger saying it has finished, with the cell near full (4.3 V at rest), takes it to 100 within a minute.
	 */
	for (unsigned tick = 0;; tick++) {
		struct hakios_battery_state s = { .size = sizeof(s) };
		uint64_t now_ns;
		int64_t pm, target_uah;
		int64_t wsum_uv = 0, wsum_ua = 0;
		int voltage_pm, rest_mv;

		absolutetime_to_nanoseconds(mach_absolute_time(), &now_ns);
		r = measure();
		if (r.ok) {
			if (r.charger != last_state) {
				char why[48];

				snprintf(why, sizeof(why), "its state went from %u to %u", last_state, r.charger);
				charger_report(why);
				last_state = r.charger;
			}
			/*
			 * The charge counted: the current since the last reading. The time between readings is not trusted as
			 * it is: this thread moves between processors, and a step that goes backwards (a processor's counter
			 * behind the last one's) or is far too long would be counted as that much current. A step backwards or
			 * longer than 5 s is counted as the loop's 1 s.
			 */
			if (last_ns != 0) {
				int64_t step = (int64_t)now_ns - last_ns;

				if (step <= 0 || step > 5 * (int64_t)NSEC_PER_SEC) {
					step = NSEC_PER_SEC;
				}
				/*
				 * (A signed divisor: NSEC_PER_SEC is an unsigned long long, and with it the division was done
				 * unsigned -- a discharge, negative, became an enormous charge, and the count read 100 % a minute
				 * after starting right at 8 %.)
				 */
				charge_uah += (int64_t)r.ua * step / (int64_t)(3600 * NSEC_PER_SEC);
			}
			last_ns = (int64_t)now_ns;
			/* ... pulled towards the voltage's charge */
			window_uv[tick % 30] = r.uv;
			window_ua[tick % 30] = r.ua;
			for (unsigned i = 0; i < 30; i++) {
				wsum_uv += window_uv[i];
				wsum_ua += window_ua[i];
			}
			rest_mv = (int)(wsum_uv / 30 / 1000 - wsum_ua / 30 / 1000 * RESISTANCE_MOHM / 1000);
			voltage_pm = ocv_to_pm(rest_mv, &slope);
			target_uah = full_uah * voltage_pm / 1000;
			if (wsum_ua / 30 > -1000000 && wsum_ua / 30 < 1000000) {
				charge_uah += (target_uah - charge_uah) / (slope < 3 ? 3000 : 600);
			}
			/*
			 * The charger has finished: full. Only near the cell's full voltage: the charger also says "terminated"
			 * for a moment on its way to "disabled" (seen at 26 %, when its watchdog stopped it), and taken alone
			 * that sent the count to 100 % within a minute.
			 */
			if (r.plugged && r.charger == 5 && rest_mv >= 4300) {
				charge_uah += (full_uah - charge_uah) / 20;
			}
			charge_uah = charge_uah < 0 ? 0 : charge_uah > full_uah ? full_uah : charge_uah;
			pm = charge_uah * 1000 / full_uah;
			if ((r.ua < 0 && pm < shown_pm) || (r.ua > 0 && pm > shown_pm) || pm > shown_pm + 10 || pm < shown_pm - 10 ||
			    (r.ua == 0 && pm != shown_pm)) {
				shown_pm = (int)pm;
			}

			s.percent_hundredths = (int32_t)(shown_pm * 10);
			s.voltage_uv = r.uv;
			s.current_ua = r.ua;
			/* the battery's temperature is not measured here (the ADC would have to be driven): left unknown */
			s.temperature_hundredths = r.temp_bad ? 4600 : -27300;
			s.present = r.present;
			s.external = r.plugged;
			s.status = !r.plugged ? 2 : r.charger == 5 ? 4 : (r.charger >= 1 && r.charger <= 4) ? 1 : 3;
			s.charge_uah = (int32_t)charge_uah;
			s.full_uah = (int32_t)full_uah;
			s.design_uah = DESIGN_UAH;
			s.usb_type = r.plugged ? 2 : 0;
			if (tick % 5 == 0) {
				hakios_battery_report(&s);
			}
			if (tick % 5 == 0 && misc_write != 0 && r.plugged) {
				uint8_t errors = 0;

				if (r.temp_bad) {
					stop_petting("the battery's temperature is out of range");
				} else if (!get(&chgr, 0x07, &errors, 1) || (errors & 0x02)) {
					stop_petting("the charger reports the battery over its voltage, or cannot be read");
				}
				pet();
			}
		}
		if ((level >= 2 && tick % 5 == 0) || tick % 60 == 0) {
			say("qg: %d.%d%%, %d mV, %d mA, %s, charger state %u%s%s\n", s.percent_hundredths / 100,
			    s.percent_hundredths / 10 % 10, r.uv / 1000, r.ua / 1000, r.plugged ? "plugged in" : "on battery", r.charger,
			    r.temp_bad ? ", BATTERY TEMPERATURE OUT OF RANGE" : "", r.ok ? "" : " (the gauge could not be read)");
		}
		delay_for_interval(1000, NSEC_PER_MSEC);
	}
out:
	thread_terminate(current_thread());
}

void
hakios_miatoll_battery_init(void)
{
	thread_t thread;

	if (!PE_parse_boot_argn("qg", &level, sizeof(level)) || level == 0) {
		return;
	}
	PE_parse_boot_argn("charge", &charge_level, sizeof(charge_level));
	if (pe_arm_get_soc_base_phys() != MIATOLL_SOC_BASE) {
		say("qg: not miatoll: nothing done\n");
		return;
	}
	if (kernel_thread_start(battery_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}
