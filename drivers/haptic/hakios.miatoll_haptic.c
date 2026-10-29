/*
 * The vibration motor of the Redmi Note 9S (miatoll, curtana): a linear resonant actuator driven by an
 * Awinic AW8624 (chip ID 0x24), Android's "aw8624_haptic" (Xiaomi's aw8624.c). This is
 * drivers/haptic/hakios.haptic.c, the POCO's AW86927, with this phone's pins and the AW8624's registers and
 * sequences, which are Xiaomi's driver's:
 *
 *   I2C        the serial engine 4 (qupv3_se4_i2c at 0x890000, function qup04 on GPIO 115 data and 116 clock,
 *              the TLMM's west tile), address 0x5a (dtbo: aw8624_haptic@5A, the one of the two speaker
 *              amplifiers on the same bus is at 0x4c); done by hand, the pins taken for each transaction and
 *              given back as they were found
 *   reset      GPIO 52 (west tile), high = running (Xiaomi: low 5 ms, then high 5 ms, then the ID)
 *   interrupt  GPIO 26 (not used: polled)
 *   effects    Android's RAM waveforms, /vendor/firmware/aw8624_haptic.bin (fw-haptic/ here, not in the
 *              repository): eleven, 1..10 short (Android's effects 0..9), 11 the one Xiaomi loops for a
 *              vibration of some length ("effect_id_boundary + 1")
 *
 *   aw8624=1     a look: the pins, and whether the chip answers (its ID); nothing written
 *   aw8624=2     ... and the reset released as Xiaomi's driver releases it, the chip's registers listed
 *   aw8624=3     ... and Xiaomi's configuration written (standby, 24 kHz PWM, its brake), the waveforms loaded
 *              into the chip's RAM and read back; nothing moves
 *   aw8624=4     ... and one short effect played (aw8624_effect=N, default 1)
 *   aw8624=5     ... and the motor for iOS until the run ends, as the POCO's: what iOS plays on its "Actuator"
 *              (/dev/hakios-haptic) becomes taps (waveform 1) and vibrations (waveform 11, looped) of
 *              HAPTIC_MAX_MS at most
 *
 * Limits as the POCO's: Android's own waveforms and gain only; no effect longer than HAPTIC_MAX_MS; any fault
 * the chip reports (over-voltage, over-current, over-temperature; its under-voltage flag, which it latches at power-up
 * and in standby, is only noted, as Xiaomi's driver notes it), a write that does not read back or a byte nobody
 * acknowledges, and the chip goes into reset for the rest of the run, as it does at the run's end.
 */
#include <kern/clock.h>
#include <libkern/OSAtomic.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <machine/machine_routines.h>
#include <pexpert/pexpert.h>
#include <miscfs/devfs/devfs.h>
#include <sys/conf.h>
#include <sys/systm.h>
#include <sys/uio.h>
#include <sys/hakios.audio.h>
#include <sys/hakios.input.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern kern_return_t thread_terminate(thread_t thread);
extern vm_offset_t pe_arm_get_soc_base_phys(void);
extern void loader_log_putc(char c);

static vm_offset_t tlmm;
static uint32_t level;
static bool broken, released, leaving;

/* every line to the loader's log as well (and to the console when that may be written) */
static int say(const char *format, ...) __printflike(1, 2);

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
	if (!leaving && ml_get_interrupts_enabled()) {
		printf("%s", line);
	}
	return length;
}

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

static void
spin_us(uint32_t us)
{
	uint64_t until;

	nanoseconds_to_absolutetime((uint64_t)us * NSEC_PER_USEC, &until);
	until += mach_absolute_time();
	while (mach_absolute_time() < until) {
	}
}

static void
wait_ms(uint32_t ms)
{
	if (leaving) {
		spin_us(ms * 1000);
	} else {
		delay_for_interval(ms, NSEC_PER_MSEC);
	}
}

/*
 * I2C 4 (GPIO 115/116) is shared: the vibration motor's AW8624 (devices/miatoll/drivers/haptic) and, on some of these
 * phones, the speaker's TAS2562 (devices/miatoll/drivers/audio) are both on it, each driven by hand from a thread of
 * its own. A transaction holds the bus: this word, taken with a compare-and-swap (both files define it, weak: one).
 */
__attribute__((weak)) volatile UInt32 hakios_miatoll_i2c4_busy = 0;

static bool
i2c4_hold(void)
{
	for (unsigned waited = 0; waited < 4000; waited++) {           /* 200 ms at most */
		if (OSCompareAndSwap(0, 1, &hakios_miatoll_i2c4_busy)) {
			return true;
		}
		spin_us(50);
	}
	return false;
}

static void
i2c4_release(void)
{
	OSCompareAndSwap(1, 0, &hakios_miatoll_i2c4_busy);
}

/* ---- pins ------------------------------------------------------------------------------------------ */

#define TLMM_PHYS               0x03500000UL    /* the TLMM's west tile (pinctrl-atoll.c: WEST 0x00100000 above 0x03400000) */
#define TLMM_SIZE               0x80000UL
#define GPIO_CFG(n)             (tlmm + 0x1000UL * (n))         /* [1:0] pull, [5:2] function, [8:6] drive, [9] output */
#define GPIO_IO(n)              (tlmm + 0x1000UL * (n) + 4)     /* bit 0 in, bit 1 out */
#define CFG_PULL_UP             3
#define CFG_OUTPUT              (1u << 9)

#define PIN_SDA                 115     /* the serial engine's first pin is the data line, the second the clock */
#define PIN_SCL                 116
#define PIN_RESET               52

static void
show_pins(void)
{
	static const struct {
		unsigned pin;
		const char *what;
	} pins[] = { { PIN_SDA, "I2C data" }, { PIN_SCL, "I2C clock" }, { PIN_RESET, "reset" } };

	for (unsigned i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
		uint32_t cfg = rd(GPIO_CFG(pins[i].pin)), io = rd(GPIO_IO(pins[i].pin));

		say("haptic: gpio%u (%s): function %u, pull %u, %u mA, %s, reads %u\n", pins[i].pin, pins[i].what, (cfg >> 2) & 15,
		    cfg & 3, (((cfg >> 6) & 7) + 1) * 2, (cfg & CFG_OUTPUT) ? (io & 2 ? "drives high" : "drives low") : "input", io & 1);
	}
}

/* the reset pin as an output of the GPIO function, driving `high' (2 mA, no pull, as Android has it) */
static void
reset_pin(bool high)
{
	wr(GPIO_IO(PIN_RESET), high ? 2 : 0);
	wr(GPIO_CFG(PIN_RESET), CFG_OUTPUT);
	released = high;
}

/* ---- I2C by hand (as in hakios.speaker.c) ---------------------------------------------------------- */

static void
half_bit(void)
{
	spin_us(5);                     /* 100 kHz at most */
}

static void
line(unsigned pin, bool high)
{
	wr(GPIO_IO(pin), 0);
	wr(GPIO_CFG(pin), CFG_PULL_UP | (high ? 0 : CFG_OUTPUT));
}

static bool
level_of(unsigned pin)
{
	return rd(GPIO_IO(pin)) & 1;
}

static void
i2c_start(void)
{
	line(PIN_SDA, true);
	line(PIN_SCL, true);
	half_bit();
	line(PIN_SDA, false);
	half_bit();
	line(PIN_SCL, false);
	half_bit();
}

static void
i2c_stop(void)
{
	line(PIN_SDA, false);
	half_bit();
	line(PIN_SCL, true);
	half_bit();
	line(PIN_SDA, true);
	half_bit();
}

static bool
i2c_bit(bool out)
{
	bool in;

	line(PIN_SDA, out);
	half_bit();
	line(PIN_SCL, true);
	for (unsigned stretched = 0; !level_of(PIN_SCL) && stretched < 100; stretched++) {
		half_bit();
	}
	half_bit();
	in = level_of(PIN_SDA);
	line(PIN_SCL, false);
	return in;
}

static bool
i2c_write(uint8_t byte)
{
	for (int bit = 7; bit >= 0; bit--) {
		i2c_bit((byte >> bit) & 1);
	}
	return !i2c_bit(true);
}

static uint8_t
i2c_read(bool more)
{
	uint8_t byte = 0;

	for (int bit = 7; bit >= 0; bit--) {
		byte = (uint8_t)(byte << 1 | i2c_bit(true));
	}
	i2c_bit(!more);
	return byte;
}

static uint32_t taken_cfg[2], taken_io[2];

static bool
bus_take(void)
{
	if (!i2c4_hold()) {
		say("haptic: I2C 4 stays busy (the speaker's amplifier?): this transaction is not made\n");
		return false;
	}
	taken_cfg[0] = rd(GPIO_CFG(PIN_SDA));
	taken_cfg[1] = rd(GPIO_CFG(PIN_SCL));
	taken_io[0] = rd(GPIO_IO(PIN_SDA));
	taken_io[1] = rd(GPIO_IO(PIN_SCL));
	line(PIN_SDA, true);
	line(PIN_SCL, true);
	half_bit();
	if (!level_of(PIN_SDA) || !level_of(PIN_SCL)) {
		wr(GPIO_IO(PIN_SDA), taken_io[0] & 2);
		wr(GPIO_IO(PIN_SCL), taken_io[1] & 2);
		wr(GPIO_CFG(PIN_SDA), taken_cfg[0]);
		wr(GPIO_CFG(PIN_SCL), taken_cfg[1]);
		say("haptic: the I2C bus is not at rest (gpio%u reads %u, gpio%u reads %u): not touched\n", PIN_SDA,
		    level_of(PIN_SDA), PIN_SCL, level_of(PIN_SCL));
		i2c4_release();
		return false;
	}
	return true;
}

static void
bus_give_back(void)
{
	wr(GPIO_IO(PIN_SDA), taken_io[0] & 2);
	wr(GPIO_IO(PIN_SCL), taken_io[1] & 2);
	wr(GPIO_CFG(PIN_SDA), taken_cfg[0]);
	wr(GPIO_CFG(PIN_SCL), taken_cfg[1]);
	i2c4_release();
}

#define AW_ADDRESS              0x5a

/* `count' 8-bit registers from `reg' on (the chip's address counts up, but for its RAM data port) */
static bool
regs_read(uint8_t reg, uint8_t *values, unsigned count)
{
	bool answered;

	if (!bus_take()) {
		return false;
	}
	i2c_start();
	answered = i2c_write(AW_ADDRESS << 1) && i2c_write(reg);
	if (answered) {
		i2c_start();
		answered = i2c_write(AW_ADDRESS << 1 | 1);
	}
	for (unsigned i = 0; answered && i < count; i++) {
		values[i] = i2c_read(i + 1 < count);
	}
	i2c_stop();
	bus_give_back();
	return answered;
}

static bool
regs_write(uint8_t reg, const uint8_t *values, unsigned count)
{
	bool answered;

	if (!bus_take()) {
		return false;
	}
	i2c_start();
	answered = i2c_write(AW_ADDRESS << 1) && i2c_write(reg);
	for (unsigned i = 0; answered && i < count; i++) {
		answered = i2c_write(values[i]);
	}
	i2c_stop();
	bus_give_back();
	return answered;
}

/* ---- the chip -------------------------------------------------------------------------------------- */

#define AW_ID                   0x00            /* reads the ID; 0xaa written: a soft reset */
#define AW_CHIPID               0x24
#define AW_SYSST                0x01
#define AW_SYSINT               0x02            /* read: cleared */
#define AW_SYSINTM              0x03            /* [5] under-voltage interrupt masked */
#define AW_SYSCTRL              0x04            /* [5] RAM open for loading, [3:2] play mode (0 RAM), [0] standby */
#define AW_GO                   0x05            /* [0] go */
#define AW_WAVSEQ1              0x07
#define AW_WAVSEQ2              0x08
#define AW_WAVLOOP1             0x0f            /* [7:4] how often WAVSEQ1 repeats, 15: until stopped */
#define AW_BASE_ADDRH           0x21
#define AW_BASE_ADDRL           0x22
#define AW_FIFO_AEH             0x23
#define AW_FIFO_AEL             0x24
#define AW_FIFO_AFH             0x25
#define AW_FIFO_AFL             0x26
#define AW_PWMDBG               0x2e            /* [6:5] 2: 24 kHz */
#define AW_SW_BRAKE             0x39
#define AW_DATDBG               0x3b            /* the gain */
#define AW_RAMADDRH             0x40
#define AW_RAMADDRL             0x41
#define AW_RAMDATA              0x42
#define AW_GLB_STATE            0x47            /* [3:0] the state: 0 standby */
#define SYSCTRL_RAMINIT         0x20
#define SYSCTRL_PLAY_MODE       0x0c
#define SYSCTRL_STANDBY         0x01
#define SYSINT_FAULTS           0x46            /* over-voltage, over-current, over-temperature */
#define SYSINT_UVLO             0x20            /* under-voltage: latched at power-up and in standby; Xiaomi's driver only logs it */
#define GAIN                    0x80            /* Xiaomi's (aw8624.c: aw8624_haptic_effect_strength) */
#define SW_BRAKE                0x2c            /* the dtbo's vib_sw_brake */

#define HAPTIC_MAX_MS           1000

static void
give_up(const char *why)
{
	broken = true;
	say("haptic: %s: THE CHIP IS PUT INTO RESET (gpio%u low) and nothing more is done\n", why, PIN_RESET);
	if (tlmm != 0) {
		reset_pin(false);
	}
}

static bool
get(uint8_t reg, uint8_t *value)
{
	if (!regs_read(reg, value, 1)) {
		say("haptic: %#04x NOT READ\n", reg);
		return false;
	}
	return true;
}

/* One register written and read back; `quiet': not said (the RAM's data, the go bit, which do not read back so) */
static bool
put(uint8_t reg, uint8_t value)
{
	uint8_t after = 0;

	if (broken) {
		return false;
	}
	if (!regs_write(reg, &value, 1)) {
		say("haptic: %#04x <- %#04x NOT ACKNOWLEDGED\n", reg, value);
		give_up("a write was not acknowledged");
		return false;
	}
	if (!regs_read(reg, &after, 1)) {
		give_up("a register could not be read back");
		return false;
	}
	if (after != value) {
		say("haptic: %#04x <- %#04x, reads %#04x: NOT AS WRITTEN\n", reg, value, after);
		give_up("a register did not read back as written");
		return false;
	}
	return true;
}

/* bits of a register: (value & ~mask) | bits, written and read back */
static bool
put_bits(uint8_t reg, uint8_t mask, uint8_t bits)
{
	uint8_t value = 0;

	return get(reg, &value) && put(reg, (uint8_t)((value & ~mask) | bits));
}

/* a write that is not read back (the go bit, the RAM's data port): only acknowledged */
static bool
poke(uint8_t reg, uint8_t value)
{
	if (broken || !regs_write(reg, &value, 1)) {
		give_up("a write was not acknowledged");
		return false;
	}
	return true;
}

/* The chip's interrupt flags (reading clears them): false, and the chip given up on, if one is a fault. */
static bool
healthy(const char *when)
{
	uint8_t flags = 0;

	if (!get(AW_SYSINT, &flags)) {
		give_up("the flags could not be read");
		return false;
	}
	if (flags & SYSINT_FAULTS) {
		say("haptic: %s: the chip reports %#04x:%s%s%s%s\n", when, flags, flags & 0x40 ? " OVER-VOLTAGE" : "",
		    flags & 0x20 ? " UNDER-VOLTAGE" : "", flags & 0x04 ? " OVER-CURRENT" : "", flags & 0x02 ? " OVER-TEMPERATURE" : "");
		give_up("a fault");
		return false;
	}
	if (flags & SYSINT_UVLO) {
		static unsigned said;

		if (said++ < 3) {
			say("haptic: %s: the chip's under-voltage flag is set (%#04x): noted, as Xiaomi's driver notes it\n", when, flags);
		}
	}
	return true;
}

static void
show_registers(void)
{
	char line[200];
	int at = 0;

	for (unsigned reg = 0; reg <= 0x5f; reg++) {
		uint8_t value = 0;

		if (reg == AW_RAMDATA || reg == 0x06) {
			continue;               /* (data ports) */
		}
		if (!regs_read((uint8_t)reg, &value, 1)) {
			say("haptic: %#04x NOT READ\n", reg);
			return;
		}
		at += snprintf(line + at, sizeof(line) - at, " %02x=%02x", reg, value);
		if (at > 150) {
			say("haptic:%s\n", line);
			at = 0;
		}
	}
	if (at) {
		say("haptic:%s\n", line);
	}
}

static bool
chip_answers(void)
{
	uint8_t id = 0;

	if (!regs_read(AW_ID, &id, 1)) {
		say("haptic: nobody answers at %#x on the bus of gpio%u/%u\n", AW_ADDRESS, PIN_SDA, PIN_SCL);
		return false;
	}
	say("haptic: the chip at %#x is %#04x%s\n", AW_ADDRESS, id, id == AW_CHIPID ? ", an AW8624" : ": NOT THE AW8624 EXPECTED");
	return id == AW_CHIPID;
}

static uint32_t states_seen;           /* the GLB_STATE states seen by the last wait, a bit each */

/* until the chip says it is in standby (GLB_STATE 0), at most `ms' */
static bool
wait_standby(unsigned ms)
{
	states_seen = 0;
	for (unsigned waited = 0; waited <= ms; waited += 2) {
		uint8_t state = 0xff;

		if (!get(AW_GLB_STATE, &state)) {
			return false;
		}
		states_seen |= 1u << (state & 0x0f);
		if ((state & 0x0f) == 0) {
			return true;
		}
		wait_ms(2);
	}
	return false;
}

/* standby (Xiaomi's aw8624_haptic_play_mode(STANDBY)): the under-voltage interrupt masked, the chip asleep */
static bool
standby(void)
{
	return put_bits(AW_SYSINTM, 0x20, 0x20) && put_bits(AW_SYSCTRL, SYSCTRL_STANDBY, SYSCTRL_STANDBY);
}

/*
 * Xiaomi's configuration (aw8624_haptic_init): a soft reset, standby, PWM at 24 kHz, its brake. The motor
 * protection and the battery compensation are left as the chip has them after the reset.
 */
static bool
configure(void)
{
	say("haptic: Xiaomi's configuration is written\n");
	if (!poke(AW_ID, 0xaa)) {
		return false;
	}
	wait_ms(2);
	if (!chip_answers()) {
		give_up("the chip does not answer after its soft reset");
		return false;
	}
	return standby() && put_bits(AW_PWMDBG, 0x60, 0x40) && put(AW_SW_BRAKE, SW_BRAKE) && healthy("after the configuration");
}

/*
 * Android's waveforms into the chip's RAM (Xiaomi's aw8624_container_update): the file is a checksum (16 bits,
 * of all that follows), the address the RAM's contents begin at, and those contents. The RAM opened (SYSCTRL
 * bit 5), the base address and the FIFO's thresholds set from it, the address port set, the contents written a
 * byte at a time through the data port; then read back the same way and compared, and the RAM closed.
 */
static bool
load_waveforms(void)
{
	uint64_t size = 0;
	const uint8_t *file = hakios_image_file("fw", "aw8624_haptic.bin", &size, NULL);
	uint32_t sum = 0, base, length;

	if (file == NULL || size < 8) {
		say("haptic: no aw8624_haptic.bin in fw/ of the small image (fw-haptic/ when it is built)\n");
		return false;
	}
	for (uint64_t i = 2; i < size; i++) {
		sum += file[i];
	}
	if ((sum & 0xffff) != (uint32_t)(file[0] << 8 | file[1])) {
		say("haptic: aw8624_haptic.bin does not add up (%#x, it says %#x): not loaded\n", sum & 0xffff, file[0] << 8 | file[1]);
		return false;
	}
	base = (uint32_t)(file[2] << 8 | file[3]);
	length = (uint32_t)size - 4;
	say("haptic: Android's waveforms: %u bytes from %#x on\n", length, base);
	if (!wait_standby(250) || !put_bits(AW_SYSCTRL, SYSCTRL_RAMINIT, SYSCTRL_RAMINIT) ||
	    !put(AW_BASE_ADDRH, file[2]) || !put(AW_BASE_ADDRL, file[3]) ||
	    !put(AW_FIFO_AEH, (uint8_t)((base >> 1) >> 8)) || !put(AW_FIFO_AEL, (uint8_t)(base >> 1)) ||
	    !put(AW_FIFO_AFH, (uint8_t)((base - (base >> 2)) >> 8)) || !put(AW_FIFO_AFL, (uint8_t)(base - (base >> 2))) ||
	    !poke(AW_RAMADDRH, file[2]) || !poke(AW_RAMADDRL, file[3])) {
		give_up("the RAM could not be opened");
		return false;
	}
	for (uint32_t at = 0; at < length; at++) {
		if (!poke(AW_RAMDATA, file[4 + at])) {
			give_up("the RAM could not be written");
			return false;
		}
	}
	if (!poke(AW_RAMADDRH, file[2]) || !poke(AW_RAMADDRL, file[3])) {
		give_up("the RAM's address could not be set again");
		return false;
	}
	for (uint32_t at = 0; at < length; at++) {
		uint8_t back = 0;

		if (!regs_read(AW_RAMDATA, &back, 1) || back != file[4 + at]) {
			say("haptic: the RAM reads back differently at %#x (%#04x, written %#04x)\n", base + at, back, file[4 + at]);
			(void)put_bits(AW_SYSCTRL, SYSCTRL_RAMINIT, 0);
			give_up("the waveforms did not load");
			return false;
		}
	}
	if (!put_bits(AW_SYSCTRL, SYSCTRL_RAMINIT, 0)) {
		return false;
	}
	say("haptic: Android's waveforms are in the chip's RAM and read back as written\n");
	return healthy("after the waveforms");
}

/*
 * A waveform started without waiting for it (Xiaomi's aw8624_haptic_play_effect_seq and play_repeat_seq):
 * the sequence (this waveform, then the end), its repeats, the RAM play mode, active, the gain, go.
 */
static bool
start_waveform(unsigned waveform, bool forever, uint8_t gain)
{
	return put(AW_WAVSEQ1, (uint8_t)waveform) && put(AW_WAVSEQ2, 0x00) && put_bits(AW_WAVLOOP1, 0xf0, forever ? 0xf0 : 0x00) &&
	       put_bits(AW_SYSCTRL, SYSCTRL_PLAY_MODE, 0) && put_bits(AW_SYSCTRL, SYSCTRL_STANDBY, 0) && healthy("before playing") &&
	       put_bits(AW_SYSINTM, 0x20, 0) && put(AW_DATDBG, gain) && poke(AW_GO, 0x01);
}

/* Xiaomi's aw8624_haptic_stop: go off, the chip left to come to standby, then standby */
static bool
stop_waveform(void)
{
	if (!poke(AW_GO, 0x00) || !wait_standby(50) || !standby()) {
		give_up("it does not stop");
		return false;
	}
	return true;
}

/* One waveform played once, stopped at HAPTIC_MAX_MS at the latest. */
static bool
play_effect(unsigned effect)
{
	uint64_t began = mach_absolute_time(), ended;
	bool done;

	say("haptic: waveform %u is played once, at Xiaomi's gain %#x\n", effect, GAIN);
	if (!wait_standby(250) || !start_waveform(effect, false, GAIN)) {
		return false;
	}
	wait_ms(2);
	done = wait_standby(HAPTIC_MAX_MS);
	absolutetime_to_nanoseconds(mach_absolute_time() - began, &ended);
	if (!done) {
		say("haptic: STILL PLAYING after %u ms: stopped\n", HAPTIC_MAX_MS);
	}
	if (!stop_waveform()) {
		return false;
	}
	say("haptic: the waveform has played: back in standby after %llu ms (states seen meanwhile: %#x)\n",
	    ended / NSEC_PER_MSEC, states_seen);
	return healthy("after playing");
}

/* ---- haptic=5: the motor for iOS ------------------------------------------------------------------- */

#define ACTIVE_US               30000           /* a write that was not silence counts for this long */
#define SILENT_BELOW            256             /* of 32768: samples no larger are silence */
#define TAP_US                  60000           /* longer than this, and it is a vibration, not a tap */
#define SETTLE_US               8000            /* how long iOS's waveform is listened to before a tap: its loudness */
#define FOLLOW_US               20000           /* how often a vibration's strength follows iOS's */

#define TOLD_EVENTS             24              /* the first so many taps and vibrations are each said */

static volatile uint64_t heard_us;              /* when the last write that was not silence came */
static volatile uint64_t quiet_us;              /* when the last write that was silence came */
static volatile uint64_t onset_us;              /* when iOS's last waveform began: the first write not silence after a pause */
static volatile uint32_t heard_peak;            /* the largest sample of that waveform, or since the service last looked */
static uint64_t written_frames;
static unsigned taps, loops, capped;            /* what has been played, for the run's end */

static uint64_t
now_us(void)
{
	uint64_t ns;

	absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
	return ns / NSEC_PER_USEC;
}

/* From the audio server's I/O thread: never waits. */
static int
haptic_write(__unused dev_t dev, struct uio *uio, __unused int ioflag)
{
	int16_t samples[256];
	int error = 0;

	while (uio_resid(uio) >= 2) {
		user_ssize_t bytes = uio_resid(uio);
		uint32_t peak = 0;

		if (bytes > (user_ssize_t)sizeof(samples)) {
			bytes = sizeof(samples);
		}
		bytes &= ~(user_ssize_t)1;
		if ((error = uiomove((caddr_t)samples, (int)bytes, uio)) != 0) {
			break;
		}
		for (unsigned i = 0; i < bytes / 2; i++) {
			uint32_t v = samples[i] < 0 ? (uint32_t)-(int32_t)samples[i] : (uint32_t)samples[i];

			peak = v > peak ? v : peak;
		}
		written_frames += (uint64_t)bytes / 2;
		if (peak > SILENT_BELOW) {
			uint64_t now = now_us();

			if (heard_us == 0 || quiet_us > heard_us || now - heard_us >= ACTIVE_US) {
				onset_us = now;
				heard_peak = 0;
			}
			heard_us = now;
			if (peak > heard_peak) {
				heard_peak = peak;
			}
		} else {
			quiet_us = now_us();
		}
	}
	return error;
}

/*
 * A test's stand-in for iOS's waveform (hakios.mic.c, mic_tap=1: the microphone hears the motor): as if
 * a write of this loudness (of 32767) had just come, or silence for 0. The service plays it with all its
 * own limits, as it plays iOS's.
 */
void
hakios_haptic_pretend(uint32_t peak)
{
	uint64_t now = now_us();

	if (peak <= SILENT_BELOW) {
		quiet_us = now;
		return;
	}
	if (heard_us == 0 || quiet_us > heard_us || now - heard_us >= ACTIVE_US) {
		onset_us = now;
		heard_peak = 0;
	}
	heard_us = now;
	if (peak > heard_peak) {
		heard_peak = peak;
	}
}

static int
haptic_open(__unused dev_t dev, __unused int flags, __unused int type, __unused struct proc *p)
{
	return 0;
}

static const struct cdevsw haptic_cdevsw = {
	.d_open = haptic_open, .d_close = haptic_open, .d_read = eno_rdwrt, .d_write = haptic_write,
	.d_ioctl = eno_ioctl, .d_stop = eno_stop, .d_reset = eno_reset, .d_select = eno_select,
	.d_mmap = eno_mmap, .d_strategy = eno_strat, .d_type = 0
};

/* the chip's state (GLB_STATE: 0 standby), 0xff if it cannot be read */
static unsigned
chip_state(void)
{
	uint8_t state = 0xff;

	return get(AW_GLB_STATE, &state) ? state & 0x0f : 0xff;
}

/*
 * How strong to play what iOS plays. iOS drives its Taptic Engine with waveforms far under full
 * scale for most feedbacks (a tap's peak anywhere from 1000 to 33000 of 32768, a vibration's
 * about 17000), and followed in proportion (gain = Android's * peak / full scale) most of them came
 * out at a tenth of Android's gain or less: not felt at all. Android plays its own clicks and
 * vibrations at its gain, 0x80. So: Android's gain from a third of full scale up, and below that
 * down to five eighths of it for the softest -- never above Android's.
 */
static uint8_t
gain_for(uint32_t peak)
{
	uint32_t top = GAIN, low = top * 5 / 8, third = 32767 / 3;

	return (uint8_t)(peak >= third ? top : low + (top - low) * peak / third);
}

static void
serve(void)
{
	enum { IDLE, TAP, LOOP, REST } state = IDLE;
	uint64_t since = 0, rest_until = 0, last_look = 0, tap_onset = 0, followed = 0;
	uint8_t playing_gain = 0;
	unsigned follows = 0;                   /* how often the vibration playing now has changed strength */
	int major = cdevsw_add(-1, &haptic_cdevsw);

	if (major < 0 || devfs_make_node(makedev(major, 0), DEVFS_CHAR, UID_ROOT, GID_WHEEL, 0666, "hakios-haptic") == NULL) {
		say("haptic: /dev/hakios-haptic cannot be made\n");
		return;
	}
	say("haptic: THE MOTOR PLAYS WHAT IOS PLAYS ON ITS ACTUATOR (/dev/hakios-haptic) from here on, until the run ends\n");
	while (!broken) {
		uint64_t now = now_us();
		bool active = heard_us != 0 && now - heard_us < ACTIVE_US;
		uint32_t peak = heard_peak;
		uint8_t gain = gain_for(peak);

		switch (state) {
		case IDLE:
		case TAP:
			/*
			 * A waveform of iOS's that has begun (after a pause: a tap of a series is a tap of its own) is
			 * a tap -- once its first milliseconds are heard: a waveform begins softly, and they say how
			 * loud it is; one that goes on without a pause, a vibration.
			 */
			if (active && onset_us != tap_onset && now - onset_us >= SETTLE_US) {
				uint64_t began = onset_us;

				if (state == TAP && !wait_standby(0) && !stop_waveform()) {
					break;
				}
				if (!start_waveform(1, false, gain ? gain : 1)) {
					break;
				}
				state = TAP;
				since = now;
				tap_onset = began;
				taps++;
				if (taps + loops <= TOLD_EVENTS) {
					wait_ms(4);
					say("haptic: tap %u: iOS's waveform peaks at %u of 32768: Android's waveform 1 at gain %#04x; the "
					    "chip's state %u\n", taps, peak, gain ? gain : 1, chip_state());
				}
				break;
			}
			if (state == IDLE) {
				break;
			}
			if (active && quiet_us < tap_onset && now - since > TAP_US) {
				if (stop_waveform() && start_waveform(11, true, gain ? gain : 1)) {
					state = LOOP;
					since = followed = now;
					playing_gain = gain ? gain : 1;
					follows = 0;
					loops++;
					heard_peak = 0;
					if (taps + loops <= TOLD_EVENTS) {
						wait_ms(4);
						say("haptic: vibration %u: iOS goes on playing (peak %u): Android's waveform 11 over and over at "
						    "gain %#04x, %u ms at most; the chip's state %u\n", loops, peak, gain ? gain : 1,
						    HAPTIC_MAX_MS, chip_state());
					}
				}
			} else if (!active && now - since > TAP_US) {
				state = wait_standby(0) || stop_waveform() ? IDLE : state;
			}
			break;
		case LOOP:
			/* as strong as iOS's waveform is now (Android's gain at full scale, never more) */
			if (active && now - followed >= FOLLOW_US) {
				followed = now;
				heard_peak = 0;
				if (peak != 0 && (gain > playing_gain + 8 || gain + 8 < playing_gain)) {
					if (!put(AW_DATDBG, gain)) {
						break;
					}
					playing_gain = gain;
					follows++;
				}
			}
			if (!active || now - since > HAPTIC_MAX_MS * 1000ull) {
				bool cut = now - since > HAPTIC_MAX_MS * 1000ull;

				if (cut) {
					capped++;
					rest_until = now + HAPTIC_MAX_MS * 1000ull;
				}
				if (stop_waveform()) {
					state = rest_until > now ? REST : IDLE;
					if (taps + loops <= TOLD_EVENTS + 1) {
						say("haptic: vibration %u: %s after %llu ms (its strength changed %u times, last to gain %#04x); "
						    "the chip is in standby\n", loops, cut ? "CUT at the limit" : "iOS stopped, and so does the motor",
						    (now - since) / 1000, follows, playing_gain);
					}
				}
			}
			break;
		case REST:
			if (now >= rest_until) {
				state = IDLE;
			}
			break;
		}
		if (state != IDLE && state != REST && now - last_look > 50000) {
			last_look = now;
			if (!healthy("while playing")) {
				break;
			}
		}
		wait_ms(4);
	}
	say("haptic: the motor is given up on (%u taps and %u vibrations played, %u cut at %u ms)\n", taps, loops, capped,
	    HAPTIC_MAX_MS);
}

static void
haptic_thread(__unused void *parameter, __unused wait_result_t result)
{
	bool loaded = false;

	tlmm = ml_io_map(TLMM_PHYS, TLMM_SIZE);
	say("haptic: level %u: %s\n", level, level == 1 ? "a look, reading only" : level == 2 ? "the reset released, registers read" :
	    level == 3 ? "the chip configured and its waveforms loaded, nothing played" : level == 4 ? "one effect played" :
	    "the motor for iOS");
	show_pins();
	if (level >= 2 && !((rd(GPIO_CFG(PIN_RESET)) & CFG_OUTPUT) && (rd(GPIO_IO(PIN_RESET)) & 2))) {
		/* (Xiaomi's aw8624_hw_reset: the pin low, 5 ms, high, 5 ms) */
		say("haptic: the reset is released: gpio%u low for 5 ms, then high\n", PIN_RESET);
		reset_pin(false);
		wait_ms(5);
		reset_pin(true);
		wait_ms(5);
	}
	if (!chip_answers()) {
		say("haptic: (the reset pin %s)\n", (rd(GPIO_CFG(PIN_RESET)) & CFG_OUTPUT) && (rd(GPIO_IO(PIN_RESET)) & 2) ?
		    "drives high: the chip should answer" : "does not drive high: the chip is held in reset or not powered");
		if (released) {
			give_up("the chip does not answer");
		}
	} else {
		if (level >= 2) {
			show_registers();
		}
		/* (kept apart from `level == 4' so that level 5 can tell that the waveforms are really in the RAM) */
		loaded = level >= 3 && configure() && load_waveforms();
		if (loaded && level == 4) {
			uint32_t effect = 1;

			PE_parse_boot_argn("aw8624_effect", &effect, sizeof(effect));
			if (effect < 1 || effect > 10) {
				effect = 1;     /* (the short ones only) */
			}
			(void)play_effect(effect);
		} else if (level == 5 && !broken && loaded) {
			/* (not started when the waveforms did not load: it would play whatever the RAM held) */
			serve();
		} else if (level == 5 && !broken) {
			give_up("the waveforms are not loaded");
		}
	}
	say("haptic: done\n");
	thread_terminate(current_thread());
}

void
hakios_haptic_init(void)
{
	thread_t thread;

	if (!PE_parse_boot_argn("aw8624", &level, sizeof(level)) || level == 0) {
		return;
	}
	if (pe_arm_get_soc_base_phys() != 0x00100000ul) {
		say("haptic: aw8624=N is for miatoll's motor, and this is not miatoll\n");
		return;
	}
	if (level > 5) {
		level = 5;
	}
	if (kernel_thread_start(haptic_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}

/* At the run's end and on a panic's way out: the chip back into reset, if it was let out. */
void
hakios_haptic_quiesce(void)
{
	if (released && tlmm != 0) {
		leaving = true;
		reset_pin(false);
		if (level == 5) {
			say("haptic: at the run's end: %u taps and %u vibrations played (%u cut at %u ms), from %llu frames iOS "
			    "wrote; the chip is held in reset\n", taps, loops, capped, HAPTIC_MAX_MS, written_frames);
		}
	}
}
