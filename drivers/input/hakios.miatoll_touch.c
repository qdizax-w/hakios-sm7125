/*
 * The touch screen of miatoll (Redmi Note 9S / 9 Pro, SM7125), whichever panel the phone has, as a source of
 * touches for /dev/hakios-touch (sys/hakios.input.h). Both panels put the touch controller in the panel's own
 * chip (touch and display driver in one), on the same SPI pins, interrupt line and reset line; each needs its
 * firmware loaded by the host at every boot. Which one the phone has is said by a boot-arg, not found out: a
 * frame of one chip's protocol is a write in the other's, and a guess could write into the wrong chip.
 *
 *   ft8756=1   the Huaxing panel: FocalTech FT8756
 *   nt36675=1  the Tianma panel:  Novatek NT36675
 *
 * Shared, from Xiaomi's kernel for this phone (MiCode Xiaomi_Kernel_OpenSource, branch curtana-q-oss) and its
 * device tree (/soc/spi@a94000/touchscreen, compatible "focaltech,fts" "novatek,NVT-ts-spi"):
 *   power    none of ours: the chip is the panel's, powered with the display, which the UEFI has lit
 *   pins     SPI on TLMM GPIO 53 (MISO), 54 (MOSI), 55 (CLK), 56 (CS) -- the west tile -- driven by hand
 *            (mode 0, about 1 MHz against the 8 MHz its device tree allows: spi_transfer()); the interrupt line on
 *            GPIO 9 (north tile). Reset (GPIO 8) is never touched: the panel would go dark with it (Xiaomi's
 *            Novatek driver leaves it alone too, NVT_TOUCH_SUPPORT_HW_RST 0)
 * Nothing here takes an interrupt: a thread looks at the interrupt line every few milliseconds, which it
 * only reads; nothing else of the chip's or the panel's is written but the four SPI pins' configuration.
 *
 * FocalTech FT8756 (drivers/input/touchscreen/ft8756_spi):
 *   firmware the FT8756 keeps none: after power-up it waits in its boot ROM (chip ID reads ef ef, boot ID
 *            87 56, register 0xd0 = 1) for the host to load it -- Xiaomi's fts_fw_recovery(): code into
 *            PRAM and constants into DRAM, each checked with the ROM's ECC, then started (0x08), with no
 *            reset. The image is Xiaomi's fw_huaxing_v0e (tools/mk-fw-touch-miatoll.sh), fw/ft8756.bin
 *            in the small image
 *   frames   FTS SPI framing (focaltech_spi.c): address, 0x00 to write or 0xa0 to read with CRC, length,
 *            three dummy bytes, data; a read ends in a CRC-16/Kermit of the data; rx[3] bits 7 and 5 say
 *            busy or bad CRC
 *   reports  register 0x01: 0x9N, the number of points, then 6 bytes a point (event in bits 7..6 and x
 *            bits 11..8, x 7..0, id in bits 7..4 and y bits 11..8, y 7..0, pressure, area), in pixels
 *            of the 1080 x 2400 screen; "ef ef ef" there means the chip has lost its firmware
 *   line     low while a report waits (falling edge)
 *
 * Novatek NT36675 (drivers/input/touchscreen/nt36xxx_spi):
 *   frames   a byte of offset (bit 7 set to write, clear to read) into a 128-byte window that a write of
 *            0xff, address >> 15, address >> 7 places (nvt_set_page); a read gives a dummy byte, then the data
 *   chip     after an "eng" reset (0x7fff80 = 0x5a) and a boot loader reset (0x3f0fe = 0x69, the device tree's
 *            swrst-n8-addr; then 0x3f310 = 0, spi-rd-fast-addr) its trim ID at 0x1f64e is .. .. .. 75 66 03
 *   firmware none kept ("phase 2 host download", hardware CRC): the image's partitions (ILM, DLM, the info
 *            sections, the overlays, each with its SRAM address and CRC in the image's header) go into its
 *            SRAM, the ILM's and DLM's place, length and CRC into the boot loader's registers, CRC checking on,
 *            boot ready (0x3f10d = 1); it then says 0xa0 or above at event buffer + 0x60. The image is
 *            Xiaomi's novatek_ts_tianma_fw.bin from the phone's /vendor/firmware (pull-firmware.sh),
 *            fw/nt36675.bin in the small image
 *   reports  65 bytes at the event buffer (0x22d00): 6 bytes a point, id in bits 7..3 of the first (1 to 10)
 *            and its state in bits 2..0 (1 down, 2 moving), x = b1 << 4 | b3 >> 4, y = b2 << 4 | b3 & 15,
 *            in the range its firmware info (event buffer + 0x78) gives; ten reads of fd/fe in a row mean
 *            its watchdog reset it, and the firmware is loaded again (NVT_TOUCH_WDT_RECOVERY)
 *   line     high while a report waits (rising edge: the device tree's irq flags 0x2001)
 */
#include <kern/clock.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/param.h>
#include <sys/hakios.input.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern kern_return_t thread_terminate(thread_t thread);
extern const uint8_t *hakios_image_file(const char *dir, const char *name, uint64_t *size, int *disk_out);

static void
sleep_ms(uint32_t ms)
{
	delay_for_interval(ms, NSEC_PER_MSEC);
}

/* ---- pins (TLMM, SC7180's layout: 0x1000 a pin, configuration at +0, in/out at +4) ---------------- */

#define TLMM_WEST_PHYS          0x03500000UL
#define TLMM_NORTH_PHYS         0x03900000UL
#define PIN_MISO                53      /* west */
#define PIN_MOSI                54
#define PIN_CLK                 55
#define PIN_CS                  56
#define PIN_INT                 9       /* north; low = the chip has a report */

static vm_offset_t spi_pins, int_pin;   /* GPIO 53's registers (54-56 follow), GPIO 9's */

#define PIN_REGS(pin)           (spi_pins + 0x1000UL * ((pin) - PIN_MISO))

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

/* function 0 (a GPIO), no pull, drive strength in mA, direction */
static void
pin_configure(vm_offset_t regs, unsigned ma, bool output)
{
	wr(regs, (ma / 2 - 1) << 6 | (output ? 1u << 9 : 0));
}

static void
pin_set(unsigned pin, bool high)
{
	wr(PIN_REGS(pin) + 4, high ? 2 : 0);
}

static bool
pin_get(unsigned pin)
{
	return rd(PIN_REGS(pin) + 4) & 1;
}

/*
 * The chip says it has a report by pulling its line low, briefly: a look at the line every few
 * milliseconds misses most of those, and with them the reports that a finger went down or came
 * up. So the pin controller is to catch the falling edge itself and hold on to it in the pin's
 * interrupt status (TLMM, pinctrl-msm.c: msm_gpio_irq_set_type; raw status kept, the interrupt
 * itself left off: nothing reaches a processor), which is looked at and cleared here.
 */
#define TLMM_INTR_CFG           0x8
#define TLMM_INTR_STATUS        0xc
#define TLMM_INTR_FALLING       (1u << 1 | 2u << 2 | 1u << 4 | 3u << 5) /* polarity, edge, raw status, to the apps processors */
#define TLMM_INTR_RISING        (1u << 1 | 1u << 2 | 1u << 4 | 3u << 5) /* (msm_gpio_irq_set_type: detection 1, not 2) */

static bool line_active_high;   /* the Novatek's line is high while a report waits, the FocalTech's low */

static void
interrupt_catch(bool rising)
{
	line_active_high = rising;
	wr(int_pin + TLMM_INTR_CFG, rising ? TLMM_INTR_RISING : TLMM_INTR_FALLING);
	wr(int_pin + TLMM_INTR_STATUS, 0);
}

static bool
interrupt_pending(void)
{
	bool caught = rd(int_pin + TLMM_INTR_STATUS) & 1;

	if (caught) {
		wr(int_pin + TLMM_INTR_STATUS, 0);
	}
	return caught || (bool)(rd(int_pin + 4) & 1) == line_active_high;
}

/* ---- SPI by hand -------------------------------------------------------------------------------------- */

static void
spi_init(void)
{
	pin_set(PIN_CS, true);          /* the level first, so that a pin never drives the wrong one */
	pin_set(PIN_CLK, false);
	pin_set(PIN_MOSI, false);
	pin_configure(PIN_REGS(PIN_CS), 6, true);
	pin_configure(PIN_REGS(PIN_CLK), 6, true);
	pin_configure(PIN_REGS(PIN_MOSI), 6, true);
	pin_configure(PIN_REGS(PIN_MISO), 2, false);
	/* (the interrupt line is only read: its pull stays as the firmware set it up for the panel) */
}

/*
 * The clock's half periods are waited for by watching the time, not by sleeping: delay_for_interval() of 1 us
 * puts the thread to sleep and wakes it through the timers, which takes tens of microseconds, and a report is
 * some 70 bytes, over a thousand half periods -- tens of milliseconds a report, which is what made the touch
 * screen lag. Spinning, a half period is SPI_HALF_NS: about 1 MHz, an eighth of the 8 MHz the device tree
 * allows (spi-max-frequency), a report in well under a millisecond.
 */
#define SPI_HALF_NS             500

static void
spin_ns(uint64_t ns)
{
	uint64_t until;

	nanoseconds_to_absolutetime(ns, &until);
	until += mach_absolute_time();
	while (mach_absolute_time() < until) {
	}
}

/* mode 0: data out while the clock is low, sampled as it rises */
static void
spi_transfer(const uint8_t *tx, uint8_t *rx, uint32_t length)
{
	pin_set(PIN_CS, false);
	spin_ns(2000);
	for (uint32_t i = 0; i < length; i++) {
		uint8_t out = tx[i], in = 0;

		for (int bit = 7; bit >= 0; bit--) {
			pin_set(PIN_MOSI, out >> bit & 1);
			spin_ns(SPI_HALF_NS);
			pin_set(PIN_CLK, true);
			spin_ns(SPI_HALF_NS);
			in = (uint8_t)(in << 1 | pin_get(PIN_MISO));
			pin_set(PIN_CLK, false);
		}
		rx[i] = in;
	}
	spin_ns(2000);
	pin_set(PIN_CS, true);
}

/* ---- FTS frames ------------------------------------------------------------------------------------- */

#define FTS_HEADER              4       /* address, control, length (2) */
#define FTS_DUMMY               3
#define FTS_WRITE               0x00
#define FTS_READ_CRC            0xa0
#define FTS_CS_HIGH_US          150     /* between frames (CS_HIGH_DELAY) */
#define FTS_PACKET              (32 * 1024 - 16)        /* FTS_FLASH_PACKET_LENGTH_SPI */

static uint8_t frame_tx[FTS_HEADER + FTS_DUMMY + 1 + FTS_PACKET + 2];
static uint8_t frame_rx[sizeof(frame_tx)];

/* CRC-16/Kermit, as the chip puts after the data it is read */
static uint16_t
fts_crc(const uint8_t *data, uint32_t length)
{
	uint16_t crc = 0xffff;

	for (uint32_t i = 0; i < length; i++) {
		crc ^= data[i];
		for (int bit = 0; bit < 8; bit++) {
			crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0x8408) : (uint16_t)(crc >> 1);
		}
	}
	return crc;
}

/* fts_write(): buf[0] is the address, the rest data; one byte alone is a four-byte frame */
static bool
fts_write(const uint8_t *buf, uint32_t length)
{
	uint32_t data = length - 1, total = FTS_HEADER;

	if (length == 0 || data > 1 + FTS_PACKET) {
		return false;
	}
	memset(frame_tx, 0, FTS_HEADER + FTS_DUMMY + data);
	frame_tx[0] = buf[0];
	frame_tx[1] = FTS_WRITE;
	frame_tx[2] = (uint8_t)(data >> 8);
	frame_tx[3] = (uint8_t)data;
	if (data > 0) {
		total += FTS_DUMMY;
		memcpy(frame_tx + total, buf + 1, data);
		total += data;
	}
	for (int attempt = 0; attempt < 3; attempt++) {
		spi_transfer(frame_tx, frame_rx, total);
		spin_ns(FTS_CS_HIGH_US * 1000ull);       /* (at least this long: a sleep would oversleep it, and the report waits) */
		if (!(frame_rx[3] & 0xa0)) {
			return true;
		}
	}
	return false;
}

static bool
fts_read(uint8_t address, uint8_t *data, uint32_t length)
{
	const uint32_t offset = FTS_HEADER + FTS_DUMMY;
	uint32_t total = offset + length + 2;

	if (length == 0 || total > sizeof(frame_tx)) {
		return false;
	}
	for (int attempt = 0; attempt < 3; attempt++) {
		memset(frame_tx, 0, total);
		frame_tx[0] = address;
		frame_tx[1] = FTS_READ_CRC;
		frame_tx[2] = (uint8_t)(length >> 8);
		frame_tx[3] = (uint8_t)length;
		spi_transfer(frame_tx, frame_rx, total);
		spin_ns(FTS_CS_HIGH_US * 1000ull);       /* (at least this long: a sleep would oversleep it, and the report waits) */
		if (!(frame_rx[3] & 0xa0) &&
		    fts_crc(frame_rx + offset, length) == (uint16_t)(frame_rx[offset + length] | frame_rx[offset + length + 1] << 8)) {
			memcpy(data, frame_rx + offset, length);
			return true;
		}
	}
	return false;
}

static bool
fts_chip_id(uint8_t *high, uint8_t *low)
{
	return fts_read(0xa3, high, 1) && fts_read(0x9f, low, 1);
}

/* ---- loading the firmware (focaltech_flash.c, fts_fw_recovery with the FT8756's settings) ----------------- */

#define FW_APP_INFO             0x100   /* code length / 2 and its complement, then at +8 the constants' */
#define FW_DRAM_BASE            0xd00000
#define FW_ECC_LENGTH           32766   /* ecclen_max */
#define FW_ECC_OK               0xa5    /* eccok_val */

static uint16_t
fw_length_at(const uint8_t *fw, uint32_t offset)
{
	uint16_t length = (uint16_t)(fw[offset] << 8 | fw[offset + 1]);
	uint16_t complement = (uint16_t)(fw[offset + 2] << 8 | fw[offset + 3]);

	return (uint16_t)(length + complement) == 0xffff ? length : 0;
}

/* fts_ecc_cal_host(): over big-endian 16-bit words, polynomial 0x8408, from 0 */
static uint16_t
fw_ecc_host(const uint8_t *data, uint32_t length)
{
	uint16_t ecc = 0;

	for (uint32_t i = 0; i < length; i += 2) {
		ecc ^= (uint16_t)(data[i] << 8 | data[i + 1]);
		for (int bit = 0; bit < 16; bit++) {
			ecc = (ecc & 1) ? (uint16_t)((ecc >> 1) ^ 0x8408) : (uint16_t)(ecc >> 1);
		}
	}
	return ecc;
}

/* fts_ecc_cal_tp(): the ROM works out the ECC of what it holds (0xcc), says when (0xce), and what (0xcd) */
static bool
fw_ecc_chip(uint32_t address, uint32_t length, uint16_t *ecc)
{
	uint8_t cmd[7] = { 0xcc, (uint8_t)(address >> 16), (uint8_t)(address >> 8), (uint8_t)address,
		           (uint8_t)(length >> 16), (uint8_t)(length >> 8), (uint8_t)length };
	uint8_t value[2] = { 0, 0 };
	int i;

	if (!fts_write(cmd, sizeof(cmd))) {
		return false;
	}
	sleep_ms(2);
	for (i = 0; i < 100; i++) {
		if (!fts_read(0xce, value, 1)) {
			return false;
		}
		if (value[0] == FW_ECC_OK) {
			break;
		}
		sleep_ms(1);
	}
	if (i == 100 || !fts_read(0xcd, value, 2)) {
		return false;
	}
	*ecc = (uint16_t)(value[0] << 8 | value[1]);
	return true;
}

/* fts_dpram_write() then fts_ecc_check(): in packets to `base`, the check over what was written */
static bool
fw_write_checked(const char *what, uint32_t base, const uint8_t *data, uint32_t length)
{
	static uint8_t cmd[1 + FTS_PACKET];

	for (uint32_t offset = 0; offset < length; offset += FTS_PACKET) {
		uint32_t n = MIN(length - offset, FTS_PACKET), address = base + offset;
		uint8_t at[4] = { 0xad, (uint8_t)(address >> 16), (uint8_t)(address >> 8), (uint8_t)address };

		cmd[0] = 0xae;
		memcpy(cmd + 1, data + offset, n);
		if (!fts_write(at, sizeof(at)) || !fts_write(cmd, 1 + n)) {
			printf("ft8756: %s: writing at %#x failed\n", what, address);
			return false;
		}
	}
	/* (Xiaomi's driver checks the constants at address 0 too, not at their base: as it does) */
	for (uint32_t offset = 0; offset < length; offset += FW_ECC_LENGTH) {
		uint32_t n = MIN(length - offset, FW_ECC_LENGTH);
		uint16_t chip = 0, host = fw_ecc_host(data + offset, n);

		if (!fw_ecc_chip(offset, n, &chip)) {
			printf("ft8756: %s: the chip did not work out its ECC at %#x\n", what, offset);
			return false;
		}
		if (chip != host) {
			printf("ft8756: %s: ECC %04x, not %04x, at %#x\n", what, chip, host, offset);
			return false;
		}
	}
	return true;
}

static bool
fw_load(void)
{
	uint64_t size = 0;
	const uint8_t *fw = hakios_image_file("fw", "ft8756.bin", &size, NULL);
	uint8_t id[2] = { 0, 0 }, state = 0, start = 0x08;
	uint32_t code, constants;

	if (fw == NULL) {
		printf("ft8756: no fw/ft8756.bin in the small image (tools/mk-fw-touch-miatoll.sh, then userland/build.sh)\n");
		return false;
	}
	/* fts_fw_recovery(): the boot ROM's ID, and its state register saying it waits for a download */
	if (!fts_read(0x90, id, 2) || !fts_read(0xd0, &state, 1) || id[0] != 0x87 || id[1] != 0x56 || state != 1) {
		printf("ft8756: not the boot ROM waiting for firmware (boot ID %02x %02x, state %02x)\n", id[0], id[1], state);
		return false;
	}
	code = (uint32_t)fw_length_at(fw, FW_APP_INFO) * 2;             /* half_length */
	constants = (uint32_t)fw_length_at(fw, FW_APP_INFO + 8) * 2;     /* drwr_support */
	if (size < 0x120 || code == 0 || code + constants > size) {
		printf("ft8756: fw/ft8756.bin (%llu bytes) is not a firmware image for it\n", size);
		return false;
	}
	printf("ft8756: loading the firmware: %u bytes of code, %u of constants\n", code, constants);
	if (!fw_write_checked("code", 0, fw, code) ||
	    (constants != 0 && !fw_write_checked("constants", FW_DRAM_BASE, fw + code, constants))) {
		return false;
	}
	if (!fts_write(&start, 1)) {
		printf("ft8756: the start command was not taken\n");
		return false;
	}
	sleep_ms(10);
	return true;
}

/* the firmware running: the chip ID 87 56; in the boot ROM it reads ef ef, and then it is loaded */
static bool
chip_ready(void)
{
	uint8_t high = 0, low = 0;

	if (!fts_chip_id(&high, &low)) {
		printf("ft8756: no answer on SPI (GPIO %u-%u)\n", PIN_MISO, PIN_CS);
		return false;
	}
	if (high == 0x87 && low == 0x56) {
		return true;
	}
	if (high != 0xef || low != 0xef) {
		printf("ft8756: chip ID %02x %02x, neither the FT8756 (87 56) nor its boot ROM (ef ef)\n", high, low);
		return false;
	}
	for (int attempt = 0; attempt < 3; attempt++) {
		if (!fw_load()) {
			continue;
		}
		for (int i = 0; i < 40; i++) {
			if (fts_chip_id(&high, &low) && high == 0x87 && low == 0x56) {
				printf("ft8756: the firmware runs\n");
				return true;
			}
			sleep_ms(25);
		}
		printf("ft8756: loaded, but the chip ID is still %02x %02x\n", high, low);
	}
	return false;
}

/* ---- reports ---------------------------------------------------------------------------------------- */

#define MAX_POINTS              10
#define REPORT_LENGTH           (2 + 6 * MAX_POINTS)
#define SCREEN_WIDTH            1080
#define SCREEN_HEIGHT           2400
#define EVENT_UP                1

static struct {
	bool down;
	uint32_t x, y;
} contact[MAX_POINTS];

static uint64_t reports_read, reports_bad;

/* a finger seen at (x, y), already scaled to 0..65535 */
static void
contact_seen(unsigned id, uint32_t x, uint32_t y)
{
	if (!contact[id].down) {
		contact[id].down = true;
		hakios_touch_post(HAKIOS_TOUCH_DOWN, id, x, y);
	} else if (contact[id].x != x || contact[id].y != y) {
		hakios_touch_post(HAKIOS_TOUCH_MOVE, id, x, y);
	}
	contact[id].x = x;
	contact[id].y = y;
}

/* every finger down that the last report did not have is lifted */
static void
contacts_lift_unseen(const bool *seen)
{
	for (unsigned id = 0; id < MAX_POINTS; id++) {
		if (contact[id].down && !seen[id]) {
			contact[id].down = false;
			hakios_touch_post(HAKIOS_TOUCH_UP, id, contact[id].x, contact[id].y);
		}
	}
}

static bool
contacts_any_down(void)
{
	for (unsigned id = 0; id < MAX_POINTS; id++) {
		if (contact[id].down) {
			return true;
		}
	}
	return false;
}

/* false: the chip has lost its firmware (reads ef ef ef) */
static bool
report_read(void)
{
	uint8_t r[REPORT_LENGTH];
	bool seen[MAX_POINTS] = { false };

	if (!fts_read(0x01, r, sizeof(r))) {
		reports_bad++;
		return true;
	}
	if (r[0] == 0xef && r[1] == 0xef && r[2] == 0xef) {
		return false;
	}
	if ((r[0] & 0xf0) != 0x90) {
		if (reports_bad++ < 5) {
			printf("ft8756: a report starting %02x %02x\n", r[0], r[1]);
		}
		return true;
	}
	reports_read++;
	for (unsigned i = 0; i < MAX_POINTS; i++) {
		const uint8_t *p = r + 2 + 6 * i;
		unsigned event = p[0] >> 6, id = p[2] >> 4;
		uint32_t x = (uint32_t)(p[0] & 0x0f) << 8 | p[1], y = (uint32_t)(p[2] & 0x0f) << 8 | p[3];

		if (id >= MAX_POINTS) {
			continue;               /* an unused slot (id 0xf) */
		}
		if (event == EVENT_UP) {
			continue;               /* lifted: not seen, released below */
		}
		if (x >= SCREEN_WIDTH || y >= SCREEN_HEIGHT) {
			continue;
		}
		x = x * 65535 / (SCREEN_WIDTH - 1);
		y = y * 65535 / (SCREEN_HEIGHT - 1);
		seen[id] = true;
		contact_seen(id, x, y);
	}
	contacts_lift_unseen(seen);
	return true;
}

static void
touch_thread(__unused void *arg, __unused wait_result_t wr)
{
	uint64_t last_line = 0;

	spi_init();
	if (!chip_ready()) {
		printf("ft8756: no touch screen\n");
		thread_terminate(current_thread());
		return;
	}
	printf("ft8756: touch screen ready (1080 x 2400)\n");
	interrupt_catch(false);
	for (;;) {
		/* (and while a finger is down, every time: a lift whose report was missed is not left standing) */
		if (interrupt_pending() || contacts_any_down()) {
			if (!report_read()) {
				printf("ft8756: the chip lost its firmware: loading it again\n");
				if (!chip_ready()) {
					printf("ft8756: no touch screen any more\n");
					break;
				}
			}
		}
		if (reports_read >= last_line + 1000) {
			last_line = reports_read;
			printf("ft8756: %llu reports, %llu bad\n", reports_read, reports_bad);
		}
		sleep_ms(4);
	}
	thread_terminate(current_thread());
}

/* ---- Novatek NT36675 (the Tianma panel): frames ---------------------------------------------------------- */

#define NVT_CHUNK               4096    /* bytes of one SPI write (Xiaomi's driver: up to 63 KB; any size will do) */
#define NVT_ENG_RST_ADDR        0x7fff80
#define NVT_SWRST_N8_ADDR       0x3f0fe /* dt: novatek,swrst-n8-addr */
#define NVT_SPI_RD_FAST_ADDR    0x3f310 /* dt: novatek,spi-rd-fast-addr */
#define NVT_TRIM_ID_ADDR        0x1f64e
/* NT36675_memory_map (nt36xxx_mem_map.h) */
#define NVT_EVENT_BUF_ADDR      0x22d00
#define NVT_BOOT_RDY_ADDR       0x3f10d
#define NVT_ILM_LENGTH_ADDR     0x3f118
#define NVT_DLM_LENGTH_ADDR     0x3f130
#define NVT_ILM_DES_ADDR        0x3f128
#define NVT_DLM_DES_ADDR        0x3f12c
#define NVT_G_ILM_CHECKSUM_ADDR 0x3f100
#define NVT_G_DLM_CHECKSUM_ADDR 0x3f104
#define NVT_BLD_CRC_EN_ADDR     0x3f30e
/* the event buffer's map */
#define NVT_EVENT_HOST_CMD      0x50
#define NVT_EVENT_RESET_DONE    0x60
#define NVT_EVENT_FWINFO        0x78
#define NVT_RESET_STATE_INIT    0xa0
#define NVT_RESET_STATE_MAX     0xaf
#define NVT_POINT_DATA_LEN      65

static uint8_t nvt_tx[1 + NVT_CHUNK + 1], nvt_rx[sizeof(nvt_tx)];

/* CTP_SPI_WRITE: the offset byte with bit 7 set, then the data */
static void
nvt_write(uint8_t offset, const uint8_t *data, uint32_t length)
{
	nvt_tx[0] = offset | 0x80;
	memcpy(nvt_tx + 1, data, length);
	spi_transfer(nvt_tx, nvt_rx, 1 + length);
}

/* CTP_SPI_READ: the offset byte, a dummy byte back, then the data */
static void
nvt_read(uint8_t offset, uint8_t *data, uint32_t length)
{
	memset(nvt_tx, 0, 2 + length);
	nvt_tx[0] = offset & 0x7f;
	spi_transfer(nvt_tx, nvt_rx, 2 + length);
	memcpy(data, nvt_rx + 2, length);
}

/* nvt_set_page: the 128-byte window at address */
static void
nvt_set_page(uint32_t address)
{
	uint8_t page[2] = { (uint8_t)(address >> 15), (uint8_t)(address >> 7) };

	nvt_write(0x7f, page, sizeof(page));    /* (0xff with bit 7 already set) */
}

static void
nvt_write_addr(uint32_t address, uint8_t value)
{
	nvt_set_page(address);
	nvt_write(address & 0x7f, &value, 1);
}

static void
nvt_bootloader_reset(void)
{
	nvt_write_addr(NVT_SWRST_N8_ADDR, 0x69);
	sleep_ms(5);
	nvt_write_addr(NVT_SPI_RD_FAST_ADDR, 0x00);
}

/* nvt_ts_check_chip_ver_trim(), for the NT36675 alone */
static bool
nvt_chip_is_nt36675(void)
{
	uint8_t id[6];

	for (int attempt = 0; attempt < 5; attempt++) {
		nvt_bootloader_reset();
		nvt_set_page(NVT_TRIM_ID_ADDR);
		nvt_read(NVT_TRIM_ID_ADDR & 0x7f, id, sizeof(id));
		if (id[3] == 0x75 && id[4] == 0x66 && id[5] == 0x03) {
			return true;
		}
		printf("nt36675: trim ID %02x %02x %02x %02x %02x %02x\n", id[0], id[1], id[2], id[3], id[4], id[5]);
		sleep_ms(10);
	}
	return false;
}

/* ---- Novatek: the firmware (nt36xxx_fw_update.c: nvt_bin_header_parser, nvt_download_firmware_hw_crc) ------ */

#define NVT_MAX_PARTITIONS      32

struct nvt_partition {
	uint32_t bin, sram, size, crc;
};

static uint32_t
le32_at(const uint8_t *p)
{
	return p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

/* the image's partitions: ILM and DLM, the info sections, the overlays; 0 if the image is not one */
static unsigned
nvt_partitions(const uint8_t *fw, uint64_t size, struct nvt_partition *part)
{
	uint32_t end, info = 0, overlays, n, need = 0;

	if (size < 0x1000) {
		return 0;
	}
	/* nvt_get_fw_need_write_size(): the last 4 KB sector that ends in "NVT" or "MOD"; its version byte and bar */
	for (uint64_t sector = size / 0x1000; sector > 0; sector--) {
		const uint8_t *flag = fw + sector * 0x1000 - 3;

		if (memcmp(flag, "NVT", 3) == 0 || memcmp(flag, "MOD", 3) == 0) {
			need = (uint32_t)(sector * 0x1000);
			break;
		}
	}
	if (need == 0 || (uint8_t)(fw[need - 0x1000] + fw[need - 0x1000 + 1]) != 0xff) {
		return 0;
	}
	end = le32_at(fw);
	for (uint32_t pos = 0x30; pos < end; pos += 0x10) {
		info++;
	}
	overlays = (fw[0x28] & 0x10) ? (fw[0x28] & 0x0f) : 0;
	n = 2 + info + overlays;
	if (n > NVT_MAX_PARTITIONS || 0x30 + 0x10 * info > size) {
		return 0;
	}
	for (uint32_t i = 0; i < n; i++) {
		const uint8_t *h;

		if (i < 2) {                                            /* BIN, SRAM, size; the CRC at 0x18 + 4i */
			h = fw + 12 * i;
			part[i].bin = le32_at(h);
			part[i].sram = le32_at(h + 4);
			part[i].size = le32_at(h + 8);
			part[i].crc = le32_at(fw + 0x18 + 4 * i);
			continue;
		}
		if (i < 2 + info) {                                     /* SRAM, size, BIN, CRC */
			h = fw + 0x30 + 0x10 * (i - 2);
		} else {                                                /* the same, at the DLM's start */
			uint64_t at = (uint64_t)part[1].bin + 0x10 * (i - 2 - info);

			if (at + 0x10 > size) {
				return 0;
			}
			h = fw + at;
		}
		part[i].sram = le32_at(h);
		part[i].size = le32_at(h + 4);
		part[i].bin = le32_at(h + 8);
		part[i].crc = le32_at(h + 12);
	}
	for (uint32_t i = 0; i < n; i++) {
		if ((uint64_t)part[i].bin + part[i].size > size) {
			return 0;
		}
	}
	return n;
}

/* nvt_set_bld_crc_bank(): where a partition goes, its length and its CRC, for the boot loader to check */
static void
nvt_crc_bank(uint32_t des_addr, uint32_t length_addr, uint32_t checksum_addr, const struct nvt_partition *p)
{
	uint8_t des[3] = { (uint8_t)p->sram, (uint8_t)(p->sram >> 8), (uint8_t)(p->sram >> 16) };
	uint8_t length[3] = { (uint8_t)p->size, (uint8_t)(p->size >> 8), (uint8_t)((p->size >> 16) & 1) };
	uint8_t crc[4] = { (uint8_t)p->crc, (uint8_t)(p->crc >> 8), (uint8_t)(p->crc >> 16), (uint8_t)(p->crc >> 24) };

	nvt_set_page(des_addr);         /* (the three are in one window) */
	nvt_write(des_addr & 0x7f, des, sizeof(des));
	nvt_write(length_addr & 0x7f, length, sizeof(length));
	nvt_write(checksum_addr & 0x7f, crc, sizeof(crc));
}

static bool
nvt_fw_load(void)
{
	static struct nvt_partition part[NVT_MAX_PARTITIONS];
	uint64_t size = 0;
	const uint8_t *fw = hakios_image_file("fw", "nt36675.bin", &size, NULL);
	unsigned n;
	uint8_t value, state[5];

	if (fw == NULL) {
		printf("nt36675: no fw/nt36675.bin in the small image (pull-firmware.sh, then userland/build.sh)\n");
		return false;
	}
	n = nvt_partitions(fw, size, part);
	if (n == 0) {
		printf("nt36675: fw/nt36675.bin (%llu bytes) is not a firmware image for it\n", size);
		return false;
	}
	printf("nt36675: loading the firmware: %llu bytes, %u partitions\n", size, n);
	nvt_bootloader_reset();
	for (unsigned i = 0; i < n; i++) {
		/* (one byte more than the header says, as Xiaomi's driver writes it; never past the image) */
		uint32_t left = part[i].size == 0 ? 0 : (uint32_t)MIN((uint64_t)part[i].size + 1, size - part[i].bin);
		uint32_t sram = part[i].sram, bin = part[i].bin;

		while (left > 0) {
			uint32_t chunk = MIN(left, NVT_CHUNK);

			nvt_set_page(sram);
			nvt_write(sram & 0x7f, fw + bin, chunk);
			sram += chunk;
			bin += chunk;
			left -= chunk;
		}
	}
	nvt_crc_bank(NVT_ILM_DES_ADDR, NVT_ILM_LENGTH_ADDR, NVT_G_ILM_CHECKSUM_ADDR, &part[0]);
	nvt_crc_bank(NVT_DLM_DES_ADDR, NVT_DLM_LENGTH_ADDR, NVT_G_DLM_CHECKSUM_ADDR, &part[1]);
	/* nvt_bld_crc_enable() */
	nvt_set_page(NVT_BLD_CRC_EN_ADDR);
	nvt_read(NVT_BLD_CRC_EN_ADDR & 0x7f, &value, 1);
	value |= 0x80;
	nvt_write(NVT_BLD_CRC_EN_ADDR & 0x7f, &value, 1);
	/* nvt_fw_crc_enable(): the reset state cleared, firmware CRC checking on */
	nvt_set_page(NVT_EVENT_BUF_ADDR);
	value = 0x00;
	nvt_write(NVT_EVENT_RESET_DONE, &value, 1);
	value = 0xae;
	nvt_write(NVT_EVENT_HOST_CMD, &value, 1);
	/* nvt_boot_ready() */
	nvt_write_addr(NVT_BOOT_RDY_ADDR, 1);
	sleep_ms(5);
	/* nvt_check_fw_reset_state(RESET_STATE_INIT) */
	nvt_set_page(NVT_EVENT_BUF_ADDR | NVT_EVENT_RESET_DONE);
	for (int i = 0; i <= 10; i++) {
		nvt_read(NVT_EVENT_RESET_DONE, state, sizeof(state));
		if (state[0] >= NVT_RESET_STATE_INIT && state[0] <= NVT_RESET_STATE_MAX) {
			return true;
		}
		sleep_ms(10);
	}
	printf("nt36675: the firmware did not start (reset state %02x %02x %02x %02x %02x)\n",
	    state[0], state[1], state[2], state[3], state[4]);
	return false;
}

/* ---- Novatek: reports ------------------------------------------------------------------------------------- */

static uint32_t nvt_x_max = SCREEN_WIDTH, nvt_y_max = SCREEN_HEIGHT;
static unsigned nvt_watchdog_reads;

/* nvt_get_fw_info(): its version and coordinate range; then the window back on the event buffer */
static void
nvt_fw_info(void)
{
	uint8_t info[16];

	nvt_set_page(NVT_EVENT_BUF_ADDR | NVT_EVENT_FWINFO);
	nvt_read(NVT_EVENT_FWINFO, info, sizeof(info));
	if ((uint8_t)(info[0] + info[1]) == 0xff) {
		nvt_x_max = (uint32_t)info[4] << 8 | info[5];
		nvt_y_max = (uint32_t)info[6] << 8 | info[7];
		printf("nt36675: firmware 0x%02x, %u x %u\n", info[0], nvt_x_max, nvt_y_max);
	} else {
		printf("nt36675: no firmware info (%02x %02x): %u x %u taken\n", info[0], info[1], SCREEN_WIDTH, SCREEN_HEIGHT);
	}
	if (nvt_x_max < 2 || nvt_y_max < 2) {
		nvt_x_max = SCREEN_WIDTH;
		nvt_y_max = SCREEN_HEIGHT;
	}
	nvt_set_page(NVT_EVENT_BUF_ADDR);
}

static bool
nvt_start(void)
{
	/* the "eng" reset, then 10 ms, as the probe does it; the ID; the firmware */
	nvt_write_addr(NVT_ENG_RST_ADDR, 0x5a);
	sleep_ms(11);
	if (!nvt_chip_is_nt36675()) {
		printf("nt36675: not an NT36675 on SPI (GPIO %u-%u): is this the Huaxing panel (ft8756=1)?\n", PIN_MISO, PIN_CS);
		return false;
	}
	for (int attempt = 0; attempt < 3; attempt++) {
		if (nvt_fw_load()) {
			nvt_fw_info();
			return true;
		}
	}
	return false;
}

/* false: its watchdog reset it, and the firmware is to be loaded again */
static bool
nvt_report_read(void)
{
	uint8_t r[NVT_POINT_DATA_LEN];
	bool seen[MAX_POINTS] = { false }, watchdog = true;

	nvt_read(0x00, r, sizeof(r));
	for (unsigned i = 0; i < 6; i++) {
		watchdog &= r[i] == 0xfd || r[i] == 0xfe;
	}
	if (watchdog) {
		return ++nvt_watchdog_reads <= 10;
	}
	nvt_watchdog_reads = 0;
	reports_read++;
	for (unsigned i = 0; i < MAX_POINTS; i++) {
		const uint8_t *p = r + 6 * i;
		unsigned slot = p[0] >> 3, state = p[0] & 7;
		uint32_t x = (uint32_t)p[1] << 4 | p[3] >> 4, y = (uint32_t)p[2] << 4 | (p[3] & 0x0f);

		if (slot == 0 || slot > MAX_POINTS || (state != 1 && state != 2)) {
			continue;               /* (an empty slot, a lift, or a packet of another kind: palm, gesture) */
		}
		if (x > nvt_x_max || y > nvt_y_max) {
			continue;
		}
		seen[slot - 1] = true;
		contact_seen(slot - 1, x * 65535 / nvt_x_max, y * 65535 / nvt_y_max);
	}
	contacts_lift_unseen(seen);
	return true;
}

static void
nvt_thread(__unused void *arg, __unused wait_result_t wr)
{
	uint64_t last_line = 0;

	spi_init();
	if (!nvt_start()) {
		printf("nt36675: no touch screen\n");
		thread_terminate(current_thread());
		return;
	}
	printf("nt36675: touch screen ready\n");
	interrupt_catch(true);
	for (;;) {
		/* (and while a finger is down, every time: a lift whose report was missed is not left standing) */
		if (interrupt_pending() || contacts_any_down()) {
			if (!nvt_report_read()) {
				printf("nt36675: its watchdog reset it: loading the firmware again\n");
				nvt_watchdog_reads = 0;
				if (!nvt_fw_load()) {
					printf("nt36675: no touch screen any more\n");
					break;
				}
				nvt_fw_info();
			}
		}
		if (reports_read >= last_line + 1000) {
			last_line = reports_read;
			printf("nt36675: %llu reports\n", reports_read);
		}
		sleep_ms(4);
	}
	thread_terminate(current_thread());
}

/* ---- start ------------------------------------------------------------------------------------------------ */

void
hakios_miatoll_touch_init(void)
{
	uint32_t ft8756 = 0, nt36675 = 0;
	thread_t thread;

	PE_parse_boot_argn("ft8756", &ft8756, sizeof(ft8756));
	PE_parse_boot_argn("nt36675", &nt36675, sizeof(nt36675));
	if (!ft8756 && !nt36675) {
		return;
	}
	if (ft8756 && nt36675) {
		printf("miatoll touch: ft8756=1 and nt36675=1 both: a phone has one panel, so neither is started\n");
		return;
	}
	spi_pins = ml_io_map(TLMM_WEST_PHYS + 0x1000UL * PIN_MISO, 0x1000UL * (PIN_CS - PIN_MISO + 1));
	int_pin = ml_io_map(TLMM_NORTH_PHYS + 0x1000UL * PIN_INT, 0x1000UL);
	if (spi_pins == 0 || int_pin == 0) {
		printf("miatoll touch: cannot map the pins\n");
		return;
	}
	if (kernel_thread_start(ft8756 ? touch_thread : nvt_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}
