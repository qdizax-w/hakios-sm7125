/*
 * The backlight of miatoll's LCD (the Huaxing panel's, dt: "ft8756 video mode dsi huaxing panel",
 * qcom,mdss-dsi-bl-pmic-control-type "bl_ctrl_wled", levels 1 to 4095): the PM6150L's WLED, of the fifth
 * generation (SPMI slave 5; registers 0xd800 its control, 0xd900 its current sinks; dt: qcom,pm6150l@5
 * qcom,wled@d800, "qcom,pm6150l-spmi-wled"; MiCode curtana-q-oss drivers/video/backlight/qcom-spmi-wled.c).
 *
 * The firmware has the WLED running when the kernel starts -- the panel is lit -- and set up as Android's
 * driver sets it up (strings, currents, the boost's protection). None of that is touched here. Brightness is
 * the modulator's level and nothing else: wled5_set_brightness() writes the active modulator's two
 * brightness registers (0x53/0x54 for modulator A, 0x63/0x64 for B: 12 bits) and then sets and clears its bit
 * in the sync register (0x65), which hands the new level to the modulator. Those three registers of the sink block are
 * the only ones this driver may write; the two level registers are read back after every write, and any
 * surprise ends all writing for the run.
 *
 * iOS's brightness (0 to 1000, written as a 32-bit number to /dev/hakios-backlight by libhakios.sb.dylib in
 * SpringBoard, which follows BackBoardServices' brightness) becomes a level on a perceptual curve, never below
 * LEVEL_MIN, so the panel never goes dark however low the slider is set.
 *
 * Boot-args:
 *   backlight=1   a look, reading only: the WLED's type, which modulator is on, its width, its level
 *   backlight=2   ... and the level iOS's, /dev/hakios-backlight, for the rest of the run
 */
#include <kern/clock.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <machine/machine_routines.h>
#include <miscfs/devfs/devfs.h>
#include <pexpert/pexpert.h>
#include <sys/conf.h>
#include <sys/hakios.input.h>
#include <sys/systm.h>
#include <sys/uio.h>

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

/* ---- the PMIC arbiter (version 5) ---------------------------------------------------------------- */

#define ARB_CORE_PHYS           0x0c440000UL
#define ARB_CORE_SIZE           0x1100UL
#define ARB_CNFG_PHYS           0x0c40a000UL
#define ARB_CNFG_SIZE           0x1000UL
#define ARB_OBSERVER_PHYS       0x0e600000UL
#define ARB_OBSERVER_SIZE       0x10000UL
#define ARB_CHNLS_PHYS          0x0c600000UL
#define ARB_SYSTEM              0
#define ARB_VERSION             0x000
#define ARB_VERSION_5           0x50000000u
#define ARB_VERSION_7           0x70000000u
#define ARB_FEATURES            0x004
#define ARB_CHANNEL(n)          (0x900 + 4 * (n))
#define ARB_OWNER(n)            (0x700 + 4 * (n))
#define ARB_MAX_CHANNELS        512

#define CHANNEL_COMMAND         0x00
#define CHANNEL_STATUS          0x08
#define CHANNEL_WDATA0          0x10
#define OBSERVER_DATA0          0x18
#define STATUS_DONE             0x1
#define STATUS_FAILURE          0x2
#define STATUS_DENIED           0x4
#define STATUS_DROPPED          0x8
#define COMMAND_WRITE           (0u << 27)
#define COMMAND_READ            (1u << 27)
#define TRANSFER_TIMEOUT_US     2000

struct peripheral {
	const char *    name;
	uint16_t        id;
	uint16_t        channel;
	bool            found, writable;
	vm_offset_t     write;
};
static struct peripheral ctrl = { .name = "the PM6150L's WLED control", .id = 0x5d8 };
static struct peripheral sink = { .name = "the PM6150L's WLED sinks", .id = 0x5d9 };

static vm_offset_t arb_core, arb_cnfg, arb_observer;
static uint32_t level;
static bool broken;

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
wait_done(vm_offset_t channel)
{
	uint64_t start = mach_absolute_time(), interval;
	uint32_t status;

	nanoseconds_to_absolutetime(TRANSFER_TIMEOUT_US * (uint64_t)NSEC_PER_USEC, &interval);
	do {
		status = rd(channel + CHANNEL_STATUS);
	} while (!(status & STATUS_DONE) && mach_absolute_time() - start < interval);
	return !(status & STATUS_DONE) ? "no answer from the arbiter" : (status & STATUS_DENIED) ? "the arbiter refuses" :
	       (status & STATUS_FAILURE) ? "the transfer failed" : (status & STATUS_DROPPED) ? "the transfer was dropped" : NULL;
}

static bool
get(const struct peripheral *p, uint8_t reg, uint8_t *bytes, uint32_t count)
{
	vm_offset_t channel = arb_observer + 0x10000UL * ARB_SYSTEM + 0x80UL * p->channel;
	const char *problem = "no channel";
	uint32_t data = 0;

	for (int tries = 0; p->found && tries < 3; tries++) {
		hakios_spmi_lock();
		wr(channel + CHANNEL_COMMAND, COMMAND_READ | (uint32_t)reg << 4 | (count - 1));
		problem = wait_done(channel);
		data = rd(channel + OBSERVER_DATA0);
		hakios_spmi_unlock();
		if (problem == NULL) {
			for (uint32_t i = 0; i < count; i++) {
				bytes[i] = (uint8_t)(data >> (8 * i));
			}
			return true;
		}
	}
	say("backlight: reading %#04x of %s: %s\n", reg, p->name, problem);
	return false;
}

static void
give_up(const char *why)
{
	if (!broken) {
		broken = true;
		say("backlight: %s: no more writing to the WLED for the rest of the run\n", why);
	}
}

/* the modulator in use, found at the start: its level registers and its bit in the sync register */
static uint8_t brt_lsb, brt_msb, sync_bit;

/* One register of the sink block, of the three only; the level registers read back. */
static bool
put(uint8_t reg, uint8_t value)
{
	const char *problem = NULL;
	uint8_t back = 0xff;

	if (broken || !sink.writable || sink.write == 0 || sink.id != 0x5d9 || brt_lsb == 0 ||
	    (reg != brt_lsb && reg != brt_msb && reg != 0x65)) {
		give_up("a write that may not be made");
		return false;
	}
	if (((rd(arb_core + ARB_CHANNEL(sink.channel)) >> 8) & 0xfff) != sink.id) {
		give_up("the arbiter's channel is no longer the WLED sinks'");
		return false;
	}
	for (int tries = 0; tries < 3; tries++) {
		hakios_spmi_lock();
		wr(sink.write + CHANNEL_WDATA0, value);
		wr(sink.write + CHANNEL_COMMAND, COMMAND_WRITE | (uint32_t)reg << 4);
		problem = wait_done(sink.write);
		hakios_spmi_unlock();
		if (problem == NULL || strcmp(problem, "the transfer was dropped") != 0) {
			break;
		}
	}
	if (problem != NULL) {
		say("backlight: %#04x <- %#04x: %s\n", reg, value, problem);
		give_up("a write that failed");
		return false;
	}
	if (reg != 0x65 && (!get(&sink, reg, &back, 1) || back != value)) {
		say("backlight: %#04x <- %#04x, but it reads %#04x\n", reg, value, back);
		give_up("a register that did not read back as written");
		return false;
	}
	return true;
}

#define LEVEL_MAX               4095            /* 12 bits; the panel's qcom,mdss-dsi-bl-max-level */
#define LEVEL_MIN               48              /* about 1 %: the lowest the slider makes, still readable */

static uint32_t shown_level;

static bool
set_level(uint32_t value)
{
	uint8_t sync = 0;

	value = value < LEVEL_MIN ? LEVEL_MIN : value > LEVEL_MAX ? LEVEL_MAX : value;
	/*
	 * The modulator takes the new level on the sync bit's rising edge: wled5_set_brightness() sets the bit and
	 * then clears it again, which is what lets the next level through. (Setting it alone, as the first version
	 * here did, latched one level and no other: iOS's levels all reached the registers and none the panel.)
	 * Cleared first as well, in case it was left set.
	 */
	if (!put(brt_lsb, (uint8_t)(value & 0xff)) || !put(brt_msb, (uint8_t)(value >> 8)) || !get(&sink, 0x65, &sync, 1) ||
	    !put(0x65, (uint8_t)(sync & ~0x03)) || !put(0x65, (uint8_t)((sync & ~0x03) | sync_bit)) ||
	    !put(0x65, (uint8_t)(sync & ~0x03))) {
		return false;
	}
	shown_level = value;
	return true;
}

/* iOS's 0..1000 -> a level: roughly perceptual (the square), LEVEL_MIN at 0 */
static uint32_t
level_for(uint32_t permille)
{
	uint64_t p = permille > 1000 ? 1000 : permille;

	return LEVEL_MIN + (uint32_t)((LEVEL_MAX - LEVEL_MIN) * p * p / 1000000);
}

/* ---- /dev/hakios-backlight ---------------------------------------------------------------------- */

static volatile uint32_t wanted = 0xffffffff;   /* nothing asked yet */

static int
bl_write(__unused dev_t dev, struct uio *uio, __unused int ioflag)
{
	uint32_t value;
	int error;

	while (uio_resid(uio) >= (user_ssize_t)sizeof(value)) {
		if ((error = uiomove((caddr_t)&value, sizeof(value), uio)) != 0) {
			return error;
		}
		wanted = value > 1000 ? 1000 : value;
	}
	return 0;
}

static int
bl_read(__unused dev_t dev, struct uio *uio, __unused int ioflag)
{
	uint32_t value = shown_level;

	return uiomove((caddr_t)&value, (int)MIN(sizeof(value), (size_t)uio_resid(uio)), uio);
}

static int
bl_open(__unused dev_t dev, __unused int flags, __unused int type, __unused struct proc *p)
{
	return 0;
}

static const struct cdevsw bl_cdevsw = {
	.d_open = bl_open, .d_close = bl_open, .d_read = bl_read, .d_write = bl_write,
	.d_ioctl = eno_ioctl, .d_stop = eno_stop, .d_reset = eno_reset, .d_select = eno_select,
	.d_mmap = eno_mmap, .d_strategy = eno_strat, .d_type = 0
};

/* ---- the start ------------------------------------------------------------------------------------ */

static void
find_channel(struct peripheral *p, uint32_t channels)
{
	for (uint32_t n = 0; n < channels; n++) {
		uint32_t map = rd(arb_core + ARB_CHANNEL(n)), owner;

		if (map == 0 || ((map >> 8) & 0xfff) != p->id) {
			continue;
		}
		owner = rd(arb_cnfg + ARB_OWNER(n)) & 7;
		if (!p->found || owner == ARB_SYSTEM) {
			p->found = true;
			p->channel = (uint16_t)n;
			p->writable = owner == ARB_SYSTEM;
		}
	}
	if (!p->found) {
		say("backlight: no channel of the arbiter is %s\n", p->name);
	}
}

static void
backlight_thread(__unused void *parameter, __unused wait_result_t result)
{
	uint32_t version, channels, applied = 0xffffffff, changes = 0;
	uint8_t ctype[2] = { 0 }, stype[2] = { 0 }, status = 0, en_a = 0, en_b = 0, wid_a = 0, wid_b = 0, a[2] = { 0 }, b[2] = { 0 };
	uint8_t src_a = 0, sinks = 0;
	int major;

	arb_core = ml_io_map(ARB_CORE_PHYS, ARB_CORE_SIZE);
	arb_cnfg = ml_io_map(ARB_CNFG_PHYS, ARB_CNFG_SIZE);
	arb_observer = ml_io_map(ARB_OBSERVER_PHYS, ARB_OBSERVER_SIZE);
	version = rd(arb_core + ARB_VERSION);
	channels = rd(arb_core + ARB_FEATURES) & 0x7ff;
	if (version < ARB_VERSION_5 || version >= ARB_VERSION_7) {
		say("backlight: PMIC arbiter version %#x, not 5: nothing done\n", version);
		goto out;
	}
	if (channels == 0 || channels > ARB_MAX_CHANNELS) {
		channels = ARB_MAX_CHANNELS;
	}
	find_channel(&ctrl, channels);
	find_channel(&sink, channels);
	if (!get(&ctrl, 0x04, ctype, 2) || !get(&sink, 0x04, stype, 2) || !get(&ctrl, 0x07, &status, 1) ||
	    !get(&sink, 0x50, &en_a, 1) || !get(&sink, 0x60, &en_b, 1) || !get(&sink, 0x52, &wid_a, 1) || !get(&sink, 0x62, &wid_b, 1) ||
	    !get(&sink, 0x53, a, 2) || !get(&sink, 0x63, b, 2) || !get(&sink, 0x51, &src_a, 1) || !get(&sink, 0x46, &sinks, 1)) {
		say("backlight: the WLED cannot be read: nothing done\n");
		goto out;
	}
	say("backlight: WLED control type %#x subtype %#x status %#x; sinks type %#x subtype %#x, strings on %#x; "
	    "modulator A %s (width %s, source %u) at %u, B %s (width %s) at %u\n", ctype[0], ctype[1], status, stype[0], stype[1],
	    sinks, (en_a & 0x80) ? "on" : "off", wid_a ? "15 bits" : "12 bits", src_a & 3, a[0] | (a[1] & 0x7f) << 8,
	    (en_b & 0x80) ? "on" : "off", wid_b ? "15 bits" : "12 bits", b[0] | (b[1] & 0x7f) << 8);
	if ((en_a & 0x80) && wid_a == 0) {
		brt_lsb = 0x53, brt_msb = 0x54, sync_bit = 0x01, shown_level = a[0] | (a[1] & 0x0f) << 8;
	} else if ((en_b & 0x80) && wid_b == 0) {
		brt_lsb = 0x63, brt_msb = 0x64, sync_bit = 0x02, shown_level = b[0] | (b[1] & 0x0f) << 8;
	} else {
		say("backlight: no modulator on at 12 bits, as Android's driver leaves it: nothing written\n");
		goto out;
	}
	if (level < 2) {
		goto out;
	}
	if (!sink.writable) {
		say("backlight: the sinks' channel is not ours to write through: nothing written\n");
		goto out;
	}
	sink.write = ml_io_map(ARB_CHNLS_PHYS + 0x10000UL * sink.channel, 0x1000);
	major = cdevsw_add(-1, &bl_cdevsw);
	if (major < 0 || devfs_make_node(makedev(major, 0), DEVFS_CHAR, UID_ROOT, GID_WHEEL, 0666, "hakios-backlight") == NULL) {
		say("backlight: /dev/hakios-backlight cannot be made\n");
		goto out;
	}
	say("backlight: the level is iOS's (/dev/hakios-backlight) from here on: %u now, %u to %u\n", shown_level, LEVEL_MIN,
	    LEVEL_MAX);
	/*
	 * A new level is not jumped to but eased into: every 8 ms a quarter of the way there (at least 4 steps of
	 * 4095), so a slider's levels, which come at whatever rate it is dragged and from more than one program,
	 * make a smooth change of the light instead of a flicker of jumps.
	 */
	while (!broken) {
		uint32_t want = wanted;

		if (want != 0xffffffff && want != applied) {
			if (changes++ < 5 || changes % 50 == 0) {
				say("backlight: iOS asks for %u of 1000: level %u\n", want, level_for(want));
			}
			applied = want;
		}
		if (applied != 0xffffffff && shown_level != level_for(applied)) {
			uint32_t target = level_for(applied), step;

			step = (target > shown_level ? target - shown_level : shown_level - target) / 4;
			step = step < 4 ? 4 : step;
			if (!set_level(target > shown_level ? MIN(target, shown_level + step) :
			    (shown_level > target + step ? shown_level - step : target))) {
				break;
			}
			delay_for_interval(8, NSEC_PER_MSEC);
			continue;
		}
		delay_for_interval(20, NSEC_PER_MSEC);
	}
out:
	thread_terminate(current_thread());
}

void
hakios_miatoll_backlight_init(void)
{
	thread_t thread;

	if (!PE_parse_boot_argn("backlight", &level, sizeof(level)) || level == 0) {
		return;
	}
	if (pe_arm_get_soc_base_phys() != MIATOLL_SOC_BASE) {
		say("backlight: not miatoll: nothing done\n");
		return;
	}
	if (kernel_thread_start(backlight_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}
