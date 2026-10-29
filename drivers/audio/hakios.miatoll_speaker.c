/*
 * The speaker of the Redmi Note 9S (miatoll, curtana): one Texas Instruments TAS2562 mono amplifier, fed by
 * the audio DSP's tertiary MI2S interface (drivers/audio/hakios.sound.c, which on miatoll runs AFE port 0x1004).
 * This is the miatoll side of what drivers/audio/hakios.speaker.c is for the POCO's two Awinic amplifiers: the
 * same functions (sys/hakios.audio.h), for this chip.
 *
 * From the phone's device tree overlay (dtbo, fragments 34 and 35): "ti,tas2562" at 0x4c, on the serial engine 9
 * (qupv3_se9_i2c: GPIO 46 data, 47 clock, the TLMM's north tile) on some of these phones and on the serial engine
 * 4 (qupv3_se4_i2c: GPIO 115, 116, west tile, the bus of the vibration motor's AW8624 at 0x5a) on others; its
 * reset (shut-down, high = running) GPIO 51 (west), its interrupt GPIO 33 (not used: nothing here waits for it),
 * one channel, the left slot. Both buses are looked at; the amplifier is the one that answers.
 *
 * The registers (Linux's sound/soc/codecs/tas2562.h): page 0 -- 0x00 the page, 0x01 [0] software reset, 0x02
 * PWR_CTRL ([1:0] 0 active, 1 mute, 2 shut down; [2] voltage sense off, [3] current sense off), 0x06 TDM_CFG0
 * ([3:1] the sample rate: 4 for 44.1/48 kHz), 0x07 TDM_CFG1 ([5:1] the receive offset in bit clocks: 1 for I2S,
 * [0] the receive edge), 0x08 TDM_CFG2 ([5:4] which channel: 1 the left, [3:2] the word's length, [1:0] the slot's:
 * 3 and 2 for 32 bits), 0x09 TDM_CFG3 (the slots: left [3:0], right [7:4]); page 2 -- 0x0c..0x0f the digital volume,
 * a 32-bit Q30 number (2^30 is 0 dB).
 *
 * The level: the digital volume's 0 dB is "Android's level" here (the amplifier's own gain is left as the chip
 * comes out of its reset); hakios_speaker_volume(below) sets it `below' eighths of a dB under that, in steps of
 * 6 dB at most with `step_ms' between them. Limits: a write that is not acknowledged or does not read back, and
 * the amplifier is shut down and held in reset for the rest of the run, as it is at the run's end and on a panic's
 * way out.
 */
#include <kern/clock.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <machine/machine_routines.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>
#include <sys/hakios.audio.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern vm_offset_t pe_arm_get_soc_base_phys(void);
extern void loader_log_putc(char c);

#define MIATOLL_SOC_BASE        0x00100000ul

static int say(const char *format, ...) __printflike(1, 2);

static bool broken, found, started, leaving;

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
	if (leaving || !ml_get_interrupts_enabled()) {
		spin_us(ms * 1000);
	} else {
		delay_for_interval(ms, NSEC_PER_MSEC);
	}
}

/* ---- pins: the TLMM's tiles, 0x1000 a pin ---------------------------------------------------------- */

#define TLMM_WEST_PHYS          0x03500000UL
#define TLMM_NORTH_PHYS         0x03900000UL
#define TLMM_TILE_SIZE          0x80000UL
#define CFG(tile, n)            ((tile) + 0x1000UL * (n))
#define IO(tile, n)             ((tile) + 0x1000UL * (n) + 4)
#define CFG_PULL_UP             3
#define CFG_OUTPUT              (1u << 9)

#define PIN_RESET               51      /* west */

static vm_offset_t west, north;

struct bus {
	const char *name;
	vm_offset_t *tile;
	unsigned sda, scl;
	uint32_t taken_cfg[2], taken_io[2];
};

static struct bus buses[2] = {
	{ "I2C 9 (gpio46/47)", &north, 46, 47, { 0, 0 }, { 0, 0 } },
	{ "I2C 4 (gpio115/116)", &west, 115, 116, { 0, 0 }, { 0, 0 } },
};
static struct bus *bus;                 /* the one the amplifier answers on */

#define AMP_ADDRESS             0x4c

static void
reset_pin(bool high)
{
	wr(IO(west, PIN_RESET), high ? 2 : 0);
	wr(CFG(west, PIN_RESET), CFG_OUTPUT);
}

static void
half_bit(void)
{
	spin_us(5);
}

static void
line(struct bus *b, unsigned pin, bool high)
{
	wr(IO(*b->tile, pin), 0);
	wr(CFG(*b->tile, pin), CFG_PULL_UP | (high ? 0 : CFG_OUTPUT));
}

static bool
level_of(struct bus *b, unsigned pin)
{
	return rd(IO(*b->tile, pin)) & 1;
}

static void
i2c_start(struct bus *b)
{
	line(b, b->sda, true);
	line(b, b->scl, true);
	half_bit();
	line(b, b->sda, false);
	half_bit();
	line(b, b->scl, false);
	half_bit();
}

static void
i2c_stop(struct bus *b)
{
	line(b, b->sda, false);
	half_bit();
	line(b, b->scl, true);
	half_bit();
	line(b, b->sda, true);
	half_bit();
}

static bool
i2c_bit(struct bus *b, bool out)
{
	bool in;

	line(b, b->sda, out);
	half_bit();
	line(b, b->scl, true);
	for (unsigned stretched = 0; !level_of(b, b->scl) && stretched < 100; stretched++) {
		half_bit();
	}
	half_bit();
	in = level_of(b, b->sda);
	line(b, b->scl, false);
	return in;
}

static bool
i2c_write(struct bus *b, uint8_t byte)
{
	for (int bit = 7; bit >= 0; bit--) {
		i2c_bit(b, (byte >> bit) & 1);
	}
	return !i2c_bit(b, true);
}

static uint8_t
i2c_read(struct bus *b, bool more)
{
	uint8_t byte = 0;

	for (int bit = 7; bit >= 0; bit--) {
		byte = (uint8_t)(byte << 1 | i2c_bit(b, true));
	}
	i2c_bit(b, !more);
	return byte;
}

static bool
bus_take(struct bus *b)
{
	b->taken_cfg[0] = rd(CFG(*b->tile, b->sda));
	b->taken_cfg[1] = rd(CFG(*b->tile, b->scl));
	b->taken_io[0] = rd(IO(*b->tile, b->sda));
	b->taken_io[1] = rd(IO(*b->tile, b->scl));
	line(b, b->sda, true);
	line(b, b->scl, true);
	half_bit();
	if (!level_of(b, b->sda) || !level_of(b, b->scl)) {
		wr(IO(*b->tile, b->sda), b->taken_io[0] & 2);
		wr(IO(*b->tile, b->scl), b->taken_io[1] & 2);
		wr(CFG(*b->tile, b->sda), b->taken_cfg[0]);
		wr(CFG(*b->tile, b->scl), b->taken_cfg[1]);
		return false;
	}
	return true;
}

static void
bus_give_back(struct bus *b)
{
	wr(IO(*b->tile, b->sda), b->taken_io[0] & 2);
	wr(IO(*b->tile, b->scl), b->taken_io[1] & 2);
	wr(CFG(*b->tile, b->sda), b->taken_cfg[0]);
	wr(CFG(*b->tile, b->scl), b->taken_cfg[1]);
}

static bool
reg_read_on(struct bus *b, uint8_t reg, uint8_t *value)
{
	bool answered;

	if (!bus_take(b)) {
		return false;
	}
	i2c_start(b);
	answered = i2c_write(b, AMP_ADDRESS << 1) && i2c_write(b, reg);
	if (answered) {
		i2c_start(b);
		answered = i2c_write(b, AMP_ADDRESS << 1 | 1);
	}
	if (answered) {
		*value = i2c_read(b, false);
	}
	i2c_stop(b);
	bus_give_back(b);
	return answered;
}

static bool
reg_write_on(struct bus *b, uint8_t reg, uint8_t value)
{
	bool answered;

	if (!bus_take(b)) {
		return false;
	}
	i2c_start(b);
	answered = i2c_write(b, AMP_ADDRESS << 1) && i2c_write(b, reg) && i2c_write(b, value);
	i2c_stop(b);
	bus_give_back(b);
	return answered;
}

/* ---- the amplifier --------------------------------------------------------------------------------- */

#define TAS_PAGE                0x00
#define TAS_SW_RESET            0x01
#define TAS_PWR_CTRL            0x02
#define TAS_PB_CFG1             0x03
#define TAS_TDM_CFG0            0x06
#define TAS_TDM_CFG1            0x07
#define TAS_TDM_CFG2            0x08
#define TAS_TDM_CFG3            0x09
#define TAS_REV_ID              0x7d
#define TAS_DVC_CFG1            0x0c    /* page 2: the volume's highest byte, then 0x0d, 0x0e, 0x0f */

#define PWR_ACTIVE              0x00
#define PWR_MUTE                0x01
#define PWR_SHUTDOWN            0x02
#define PWR_SENSE_OFF           0x0c    /* the voltage and current sense off: nothing here reads them */

#define Q30_0DB                 0x40000000u
#define VOLUME_FLOOR            (110 * 8)       /* -110 dB, the volume's lowest */

static uint32_t volume_now = VOLUME_FLOOR;      /* eighths of a dB under 0 dB, as the chip has it */

static void
give_up(const char *why)
{
	if (broken) {
		return;
	}
	broken = true;
	say("speaker: %s: THE AMPLIFIER IS SHUT DOWN AND HELD IN RESET (gpio%u low) for the rest of the run\n", why, PIN_RESET);
	if (bus != NULL) {
		(void)reg_write_on(bus, TAS_PWR_CTRL, PWR_SENSE_OFF | PWR_SHUTDOWN);
	}
	if (west != 0) {
		reset_pin(false);
	}
	started = false;
}

static bool
put(uint8_t reg, uint8_t value)
{
	uint8_t back = 0;

	if (broken || bus == NULL) {
		return false;
	}
	if (!reg_write_on(bus, reg, value)) {
		say("speaker: %#04x <- %#04x NOT ACKNOWLEDGED\n", reg, value);
		give_up("a write was not acknowledged");
		return false;
	}
	if (reg == TAS_SW_RESET || reg == TAS_PAGE) {
		return true;
	}
	if (!reg_read_on(bus, reg, &back) || back != value) {
		say("speaker: %#04x <- %#04x, reads %#04x: NOT AS WRITTEN\n", reg, value, back);
		give_up("a register did not read back as written");
		return false;
	}
	return true;
}

static bool
get(uint8_t reg, uint8_t *value)
{
	if (broken || bus == NULL || !reg_read_on(bus, reg, value)) {
		return false;
	}
	return true;
}

static bool
put_bits(uint8_t reg, uint8_t mask, uint8_t bits)
{
	uint8_t value = 0;

	return get(reg, &value) && put(reg, (uint8_t)((value & ~mask) | bits));
}

/* 2^30 * 10^(-below/160): `below' eighths of a dB under 0 dB, in Q30 (a step of 1/8 dB is a factor of 0.985715) */
static uint32_t
q30_under(uint32_t below)
{
	uint64_t v = Q30_0DB;

	if (below >= VOLUME_FLOOR) {
		return 0x00000d43;      /* (Linux's table: its lowest, -110 dB) */
	}
	for (uint32_t i = 0; i < below; i++) {
		v = v * 1033605u >> 20;
	}
	return (uint32_t)v;
}

static bool
volume_set(uint32_t below)
{
	uint32_t q = q30_under(below);

	if (!put(TAS_PAGE, 2) || !put(TAS_DVC_CFG1 + 3, (uint8_t)q) || !put(TAS_DVC_CFG1 + 2, (uint8_t)(q >> 8)) ||
	    !put(TAS_DVC_CFG1 + 1, (uint8_t)(q >> 16)) || !put(TAS_DVC_CFG1, (uint8_t)(q >> 24)) || !put(TAS_PAGE, 0)) {
		return false;
	}
	volume_now = below;
	return true;
}

static bool
find(void)
{
	uint8_t rev = 0;

	if (pe_arm_get_soc_base_phys() != MIATOLL_SOC_BASE) {
		say("speaker: this is not miatoll: its TAS2562 is not looked for\n");
		return false;
	}
	if (west == 0) {
		west = ml_io_map(TLMM_WEST_PHYS, TLMM_TILE_SIZE);
		north = ml_io_map(TLMM_NORTH_PHYS, TLMM_TILE_SIZE);
	}
	/* (Xiaomi's driver: the reset pin high, then the chip answers after a few milliseconds) */
	reset_pin(false);
	wait_ms(2);
	reset_pin(true);
	wait_ms(5);
	for (unsigned i = 0; i < 2; i++) {
		if (reg_write_on(&buses[i], TAS_PAGE, 0) && reg_read_on(&buses[i], TAS_REV_ID, &rev)) {
			bus = &buses[i];
			say("speaker: the TAS2562 answers at %#x on %s (revision %#04x)\n", AMP_ADDRESS, bus->name, rev);
			return true;
		}
		say("speaker: nobody at %#x on %s\n", AMP_ADDRESS, buses[i].name);
	}
	reset_pin(false);
	return false;
}

/*
 * Found, reset and configured for what the DSP sends on the tertiary MI2S port: I2S at 48 kHz, 32-bit words, the
 * left channel; shut down, the volume at its lowest. (`acf': the POCO's amplifiers' file; this one takes none.)
 */
bool
hakios_speaker_prepare(__unused const uint8_t *acf, __unused uint64_t size)
{
	if (broken) {
		return false;
	}
	if (!found) {
		found = find();
		if (!found) {
			say("speaker: THE AMPLIFIER WAS NOT FOUND: nothing will be heard\n");
			return false;
		}
	}
	if (!put(TAS_PAGE, 0) || !put(TAS_SW_RESET, 1)) {
		return false;
	}
	wait_ms(2);
	if (!put(TAS_PAGE, 0) || !put(TAS_PWR_CTRL, PWR_SENSE_OFF | PWR_SHUTDOWN) ||
	    !put_bits(TAS_TDM_CFG0, 0x0e, 4u << 1) ||                   /* 44.1/48 kHz */
	    !put_bits(TAS_TDM_CFG1, 0x3f, 1u << 1) ||                   /* I2S: the data a bit clock after the word select, rising edge */
	    !put_bits(TAS_TDM_CFG2, 0x3f, 1u << 4 | 3u << 2 | 2u) ||    /* the left channel; 32-bit words in 32-bit slots */
	    !put(TAS_TDM_CFG3, 0x10) ||                                 /* left in slot 0, right in slot 1 */
	    !volume_set(VOLUME_FLOOR)) {
		return false;
	}
	{
		uint8_t gain = 0;

		(void)get(TAS_PB_CFG1, &gain);
		say("speaker: configured: I2S, 48 kHz, 32 bits, the left channel; shut down at -110 dB (the amplifier's gain %u.%u dB)\n",
		    (85 + 5 * ((gain >> 1) & 0x1f)) / 10, (85 + 5 * ((gain >> 1) & 0x1f)) % 10);
	}
	return true;
}

/* Powered up, unmuted, at the lowest volume: only while the DSP's bit clock runs. */
bool
hakios_speaker_start(void)
{
	if (broken || !found) {
		return false;
	}
	if (!volume_set(VOLUME_FLOOR) || !put(TAS_PWR_CTRL, PWR_SENSE_OFF | PWR_MUTE)) {
		return false;
	}
	wait_ms(5);
	if (!put(TAS_PWR_CTRL, PWR_SENSE_OFF | PWR_ACTIVE)) {
		return false;
	}
	started = true;
	say("speaker: the amplifier is up\n");
	return true;
}

/* `below' eighths of a dB under 0 dB, at most 6 dB a step, `step_ms' between steps. */
bool
hakios_speaker_volume(uint32_t below, uint32_t step_ms)
{
	if (broken || !found) {
		return false;
	}
	if (below > VOLUME_FLOOR) {
		below = VOLUME_FLOOR;
	}
	while (volume_now != below) {
		uint32_t next = volume_now > below ? (volume_now - below > 48 ? volume_now - 48 : below) :
		    (below - volume_now > 48 ? volume_now + 48 : below);

		if (!volume_set(next)) {
			return false;
		}
		if (volume_now != below && step_ms != 0) {
			wait_ms(step_ms);
		}
	}
	return true;
}

bool
hakios_speaker_healthy(void)
{
	return !broken;
}

/* Faded out, muted, shut down. */
void
hakios_speaker_stop(void)
{
	if (broken || !found) {
		return;
	}
	if (started) {
		(void)hakios_speaker_volume(VOLUME_FLOOR, 5);
		(void)put(TAS_PWR_CTRL, PWR_SENSE_OFF | PWR_MUTE);
	}
	(void)put(TAS_PWR_CTRL, PWR_SENSE_OFF | PWR_SHUTDOWN);
	started = false;
}

/* At once and without sleeping: on a panic's way out, and at the run's end. */
void
hakios_speaker_quiesce(void)
{
	if (found && west != 0) {
		leaving = true;
		reset_pin(false);
	}
}

/* The amplifier's own temperature and battery: not read here (its sense is off). Nothing to hold back for. */
uint32_t
hakios_speaker_monitor(__unused bool tell)
{
	return 0;
}

/* boot-arg speaker_probe=N on the POCO; miatoll's amplifier is looked for when the sound starts. */
void
hakios_speaker_init(void)
{
}
