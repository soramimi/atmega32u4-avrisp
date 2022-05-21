/*
 * Copyright (c) 2012 Fredrik Atmer, Bathroom Epiphanies Inc
 * http://bathroomepiphanies.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

// CDC - Copyright (C) 2022 S.Fuchita (@soramimi_jp)

#include "usb.h"
#include <avr/interrupt.h>
#include <string.h>


uint8_t data_tx_buffer_i;
uint8_t data_tx_buffer_n;

uint8_t data_rx_buffer_i;
uint8_t data_rx_buffer_n;

uint8_t data_tx_buffer[64];
uint8_t data_rx_buffer[64];

bool tx_enabled = false;

void usb_poll_tx()
{
	char tmp[32];
	for (uint8_t i = 0; i < data_tx_buffer_n; i++) {
		tmp[i] = data_tx_buffer[data_tx_buffer_i];
		data_tx_buffer_i = (data_tx_buffer_i + 1) % sizeof(data_tx_buffer);
	}
	usb_data_tx(tmp, data_tx_buffer_n);
	data_tx_buffer_n = 0;
}

void usb_poll_rx()
{
	char tmp[RX_EP_SIZE];
	uint8_t n = sizeof(data_rx_buffer) - data_rx_buffer_n;
	if (n >= RX_EP_SIZE) {
		n = usb_data_rx(tmp, n);
		for (uint8_t i = 0; i < n; i++) {
//			if (data_rx_buffer_n < sizeof(data_rx_buffer)) {
				int j = (data_rx_buffer_i + data_rx_buffer_n) % sizeof(data_rx_buffer);
				data_rx_buffer[j] = tmp[i];
				data_rx_buffer_n++;
//			}
		}
	}
}

#define PROG_FLICKER true

#define LOW 0
#define HIGH 1
#define INPUT 0
#define OUTPUT 1


// Configure SPI clock (in Hz).
// E.g. for an ATtiny @ 128 kHz: the datasheet states that both the high and low
// SPI clock pulse must be > 2 CPU cycles, so take 3 cycles i.e. divide target
// f_cpu by 6:
//     #define SPI_CLOCK            (128000/6)
//
// A clock slow enough for an ATtiny85 @ 1 MHz, is a reasonable default:

#define SPI_CLOCK (1000000 / 6)

// Select hardware or software SPI, depending on SPI clock.
// Currently only for AVR, for other architectures (Due, Zero,...), hardware SPI
// is probably too fast anyway.

#if defined(ARDUINO_ARCH_AVR)

#if SPI_CLOCK > (F_CPU / 128)
#define USE_HARDWARE_SPI
#endif

#endif

// Configure which pins to use:

// The standard pin configuration.
#ifndef ARDUINO_HOODLOADER2

#define LED_HB 9
#define LED_ERR 8
#define LED_PMODE 7

// Uncomment following line to use the old Uno style wiring
// (using pin 11, 12 and 13 instead of the SPI header) on Leonardo, Due...

// #define USE_OLD_STYLE_WIRING

//#ifdef USE_OLD_STYLE_WIRING

//#define PIN_MOSI 11
//#define PIN_MISO 12
//#define PIN_SCK 13

//#endif

// HOODLOADER2 means running sketches on the ATmega16U2 serial converter chips
// on Uno or Mega boards. We must use pins that are broken out:
#else

#define RESET 4
#define LED_HB 7
#define LED_ERR 6
#define LED_PMODE 5

#endif

// By default, use hardware SPI pins:
//#ifndef PIN_MOSI
#define PIN_RESET 1
#define PIN_MOSI 2
#define PIN_MISO 3
#define PIN_SCK 4

void pinMode(char pin, char value)
{
	switch (pin) {
	case PIN_RESET:
		if (value) { DDRB |= 1 << 6; } else { DDRB &= ~(1 << 6); }
		break;
	case PIN_MOSI:
		if (value) { DDRB |= 1 << 2; } else { DDRB &= ~(1 << 2); }
		break;
	case PIN_MISO:
		if (value) { DDRB |= 1 << 3; } else { DDRB &= ~(1 << 3); }
		break;
	case PIN_SCK:
		if (value) { DDRB |= 1 << 1; } else { DDRB &= ~(1 << 1); }
		break;
	case LED_HB:
		break;
	case LED_ERR:
		if (value) { DDRB |= 1 << 0; } else { DDRB &= ~(1 << 0); }
		break;
	case LED_PMODE:
		if (value) { DDRD |= 1 << 5; } else { DDRD &= ~(1 << 5); }
		break;
	}
}

void digitalWrite(char pin, char value)
{
	switch (pin) {
	case PIN_RESET:
		if (value) { PORTB |= 1 << 6; } else { PORTB &= ~(1 << 6); }
		break;
	case PIN_MOSI:
		if (value) { PORTB |= 1 << 2; } else { PORTB &= ~(1 << 2); }
		break;
	case PIN_MISO:
		if (value) { PORTB |= 1 << 3; } else { PORTB &= ~(1 << 3); }
		break;
	case PIN_SCK:
		if (value) { PORTB |= 1 << 1; } else { PORTB &= ~(1 << 1); }
		break;
	case LED_HB:
		break;
	case LED_ERR:
		if (value) { PORTB |= 1 << 0; } else { PORTB &= ~(1 << 0); }
		break;
	case LED_PMODE:
		if (value) { PORTD |= 1 << 5; } else { PORTD &= ~(1 << 5); }
		break;
	}
}

bool digitalRead(char pin)
{
	switch (pin) {
	case PIN_RESET:
		return (PINB >> 6) & 1;
	case PIN_MOSI:
		return (PINB >> 2) & 1;
	case PIN_MISO:
		return (PINB >> 3) & 1;
	case PIN_SCK:
		return (PINB >> 1) & 1;
	case LED_HB:
		break;
	case LED_ERR:
		return (PINB >> 0) & 1;
	case LED_PMODE:
		return (PIND >> 5) & 1;
	}
	return false;
}

void led_pmode(bool f)
{
	digitalWrite(LED_PMODE, f ? LOW : HIGH); // low active
}

void led_error(bool f)
{
	digitalWrite(LED_ERR, f ? LOW : HIGH); // low active
}

// Force bitbanged SPI if not using the hardware SPI pins:
//#if (PIN_MISO != MISO) || (PIN_MOSI != MOSI) || (PIN_SCK != SCK)
//#undef USE_HARDWARE_SPI
//#endif

// Configure the serial port to use.
//
// Prefer the USB virtual serial port (aka. native USB port), if the Arduino has one:
//   - it does not autoreset (except for the magic baud rate of 1200).
//   - it is more reliable because of USB handshaking.
//
// Leonardo and similar have an USB virtual serial port: 'Serial'.
// Due and Zero have an USB virtual serial port: 'SerialUSB'.
//
// On the Due and Zero, 'Serial' can be used too, provided you disable autoreset.
// To use 'Serial': #define SERIAL Serial

//#ifdef SERIAL_PORT_USBVIRTUAL
//#define SERIAL SERIAL_PORT_USBVIRTUAL
//#else
//#define SERIAL Serial
//#endif

// Configure the baud rate:

#define BAUDRATE 19200
// #define BAUDRATE	115200
// #define BAUDRATE	1000000

#define HWVER 2
#define SWMAJ 1
#define SWMIN 18

// STK Definitions
#define STK_OK 0x10
#define STK_FAILED 0x11
#define STK_UNKNOWN 0x12
#define STK_INSYNC 0x14
#define STK_NOSYNC 0x15
#define CRC_EOP 0x20 // ok it is a space...






bool usb_read_available()
{
	return data_rx_buffer_n > 0;
}

char usb_read_byte()
{
	if (usb_read_available()) {
		char c = data_rx_buffer[data_rx_buffer_i];
		data_rx_buffer_i = (data_rx_buffer_i + 1) % sizeof(data_rx_buffer);
		data_rx_buffer_n--;
		return c;
	}
	return 0;
}

bool usb_write_byte(char c)
{
	if (data_tx_buffer_n < sizeof(data_tx_buffer)) {
		int8_t i = (data_tx_buffer_i + data_tx_buffer_n) % sizeof(data_tx_buffer);
		data_tx_buffer[i] = c;
		data_tx_buffer_n++;
		return true;
	}
	return false;
}

void usb_write_string(char const *p)
{
	while (*p) {
		if (usb_write_byte(*p)) {
			p++;
		} else {
			usb_poll_rx();
		}
	}
}

void sleep_ms(int ms)
{
	while (ms > 0) {
		_delay_ms(1);
		usb_poll_rx();
		ms--;
	}
}

void sleep_us(int us)
{
	while (us >= 10) {
		_delay_us(10);
		us -= 10;
	}
	while (us > 0) {
		_delay_us(1);
		us--;
	}
}


#ifdef USE_HARDWARE_SPI
#include "SPI.h"
#else

#define SPI_MODE0 0x00

#if !defined(ARDUINO_API_VERSION) || ARDUINO_API_VERSION != 10001 // A SPISettings class is declared by ArduinoCore-API 1.0.1
class SPISettings {
public:
	// clock is in Hz
	SPISettings(uint32_t clock, uint8_t dataMode)
		: clockFreq(clock)
	{
		(void)dataMode;
	}

	uint32_t getClockFreq() const
	{
		return clockFreq;
	}

private:
	uint32_t clockFreq;
};
#endif // !defined(ARDUINO_API_VERSION)

#if 0
class SPI {
public:
	void begin()
	{
		digitalWrite(PIN_SCK, LOW);
		digitalWrite(PIN_MOSI, LOW);
		pinMode(PIN_SCK, OUTPUT);
		pinMode(PIN_MOSI, OUTPUT);
		pinMode(PIN_MISO, INPUT);

		// SPI: Mode 0, F_CLK/128
		SPCR = 0x53;
		SPSR = 0x00;
	}

	uint8_t transfer(uint8_t b)
	{
		SPDR = b;
		while (!(SPSR & 0x80));
		return SPDR;
	}
};
#else
class SPI {
public:
	void begin()
	{
		digitalWrite(PIN_SCK, LOW);
		digitalWrite(PIN_MOSI, LOW);
		pinMode(PIN_SCK, OUTPUT);
		pinMode(PIN_MOSI, OUTPUT);
		pinMode(PIN_MISO, INPUT);
	}

	uint8_t transfer(uint8_t b)
	{
		for (uint8_t i = 0; i < 8; ++i) {
			digitalWrite(PIN_MOSI, (b & 0x80) ? HIGH : LOW);
			digitalWrite(PIN_SCK, HIGH);
			usb_poll_rx();
//			_delay_us(1);
			b = (b << 1) | digitalRead(PIN_MISO);
			digitalWrite(PIN_SCK, LOW); // slow pulse
			usb_poll_rx();
//			_delay_us(1);
		}
		return b;
	}
};
#endif

static SPI SPI;

#endif

int ISPError = 0;
int pmode = 0;
// address for reading and writing, set by 'U' command
unsigned int here;
uint8_t buff[256]; // global block storage

#define beget16(addr) (*addr * 256 + *(addr + 1))
typedef struct param {
	uint8_t devicecode;
	uint8_t revision;
	uint8_t progtype;
	uint8_t parmode;
	uint8_t polling;
	uint8_t selftimed;
	uint8_t lockbytes;
	uint8_t fusebytes;
	uint8_t flashpoll;
	uint16_t eeprompoll;
	uint16_t pagesize;
	uint16_t eepromsize;
	uint32_t flashsize;
} parameter;

parameter param;

// this provides a heartbeat on pin 9, so you can tell the software is running.
uint8_t hbval = 128;
int8_t hbdelta = 8;
//void heartbeat()
//{
//	static unsigned long last_time = 0;
//	unsigned long now = millis();
//	if ((now - last_time) < 40) {
//		return;
//	}
//	last_time = now;
//	if (hbval > 192) {
//		hbdelta = -hbdelta;
//	}
//	if (hbval < 32) {
//		hbdelta = -hbdelta;
//	}
//	hbval += hbdelta;
//	analogWrite(LED_HB, hbval);
//}

static bool rst_active_high;

void reset_target(bool reset)
{
	digitalWrite(PIN_RESET, ((reset && rst_active_high) || (!reset && !rst_active_high)) ? HIGH : LOW);
}

uint8_t getch()
{
	while (!usb_read_available()) {
		usb_poll_rx();
	}
	return usb_read_byte();
}
void fill(int n)
{
	for (int x = 0; x < n; x++) {
		buff[x] = getch();
	}
}

#define PTIME 30
void pulse_pmode(int times)
{
	return;
	do {
		led_pmode(true);
		sleep_ms(PTIME);
		led_pmode(false);
		sleep_ms(PTIME);
	} while (times--);
}

void pulse_error(int times)
{
	return;
	do {
		led_error(true);
		sleep_ms(PTIME);
		led_error(false);
		sleep_ms(PTIME);
	} while (times--);
}

void prog_lamp(bool state)
{
	if (PROG_FLICKER) {
		led_pmode(state);
	}
}

uint8_t spi_transaction(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
	SPI.transfer(a);
	SPI.transfer(b);
	SPI.transfer(c);
	return SPI.transfer(d);
}

void empty_reply()
{
	if (CRC_EOP == getch()) {
		usb_write_byte((char)STK_INSYNC);
		usb_write_byte((char)STK_OK);
	} else {
		ISPError++;
		usb_write_byte((char)STK_NOSYNC);
	}
}

void breply(uint8_t b)
{
	if (CRC_EOP == getch()) {
		usb_write_byte((char)STK_INSYNC);
		usb_write_byte((char)b);
		usb_write_byte((char)STK_OK);
	} else {
		ISPError++;
		usb_write_byte((char)STK_NOSYNC);
	}
}

void get_version(uint8_t c)
{
	switch (c) {
	case 0x80:
		breply(HWVER);
		break;
	case 0x81:
		breply(SWMAJ);
		break;
	case 0x82:
		breply(SWMIN);
		break;
	case 0x93:
		breply('S'); // serial programmer
		break;
	default:
		breply(0);
	}
}

void set_parameters()
{
	// call this after reading parameter packet into buff[]
	param.devicecode = buff[0];
	param.revision = buff[1];
	param.progtype = buff[2];
	param.parmode = buff[3];
	param.polling = buff[4];
	param.selftimed = buff[5];
	param.lockbytes = buff[6];
	param.fusebytes = buff[7];
	param.flashpoll = buff[8];
	// ignore buff[9] (= buff[8])
	// following are 16 bits (big endian)
	param.eeprompoll = beget16(&buff[10]);
	param.pagesize = beget16(&buff[12]);
	param.eepromsize = beget16(&buff[14]);

	// 32 bits flashsize (big endian)
	param.flashsize = buff[16] * 0x01000000
		+ buff[17] * 0x00010000
		+ buff[18] * 0x00000100
		+ buff[19];

	// AVR devices have active low reset, AT89Sx are active high
	rst_active_high = (param.devicecode >= 0xe0);
}

void start_pmode()
{

	// Reset target before driving PIN_SCK or PIN_MOSI

	// SPI.begin() will configure SS as output, so SPI master mode is selected.
	// We have defined RESET as pin 10, which for many Arduinos is not the SS pin.
	// So we have to configure RESET as output here,
	// (reset_target() first sets the correct level)
	reset_target(true);
	pinMode(PIN_RESET, OUTPUT);
	SPI.begin();
//	SPI.beginTransaction(SPISettings(SPI_CLOCK, SPI_MODE0));

	// See AVR datasheets, chapter "SERIAL_PRG Programming Algorithm":

	// Pulse RESET after PIN_SCK is low:
	digitalWrite(PIN_SCK, LOW);
	sleep_ms(20); // discharge PIN_SCK, value arbitrarily chosen
	reset_target(false);
	// Pulse must be minimum 2 target CPU clock cycles so 100 usec is ok for CPU
	// speeds above 20 KHz
	sleep_us(100);
	reset_target(true);

	// Send the enable programming command:
	sleep_ms(50); // datasheet: must be > 20 msec
	spi_transaction(0xAC, 0x53, 0x00, 0x00);
	pmode = 1;
}

void end_pmode()
{
//	SPI.end();
	// We're about to take the target out of reset so configure SPI pins as input
	pinMode(PIN_MOSI, INPUT);
	pinMode(PIN_SCK, INPUT);
	reset_target(false);
	pinMode(PIN_RESET, INPUT);
	pmode = 0;
}

void universal()
{
	uint8_t ch;

	fill(4);
	ch = spi_transaction(buff[0], buff[1], buff[2], buff[3]);
	breply(ch);
}

void flash(uint8_t hilo, unsigned int addr, uint8_t data)
{
	spi_transaction(0x40 + 8 * hilo, addr >> 8 & 0xFF, addr & 0xFF, data);
}
void commit(unsigned int addr)
{
	if (PROG_FLICKER) {
		prog_lamp(LOW);
	}
	spi_transaction(0x4C, (addr >> 8) & 0xFF, addr & 0xFF, 0);
	if (PROG_FLICKER) {
		sleep_ms(PTIME);
		prog_lamp(HIGH);
	}
}

unsigned int current_page()
{
	if (param.pagesize == 32) {
		return here & 0xFFFFFFF0;
	}
	if (param.pagesize == 64) {
		return here & 0xFFFFFFE0;
	}
	if (param.pagesize == 128) {
		return here & 0xFFFFFFC0;
	}
	if (param.pagesize == 256) {
		return here & 0xFFFFFF80;
	}
	return here;
}

uint8_t write_flash_pages(int length)
{
	int x = 0;
	unsigned int page = current_page();
	while (x < length) {
		if (page != current_page()) {
			commit(page);
			page = current_page();
		}
		flash(LOW, here, buff[x++]);
		flash(HIGH, here, buff[x++]);
		here++;
	}

	commit(page);

	return STK_OK;
}

void write_flash(int length)
{
	fill(length);
	if (CRC_EOP == getch()) {
		usb_write_byte((char)STK_INSYNC);
		usb_write_byte((char)write_flash_pages(length));
	} else {
		ISPError++;
		usb_write_byte((char)STK_NOSYNC);
	}
}

// write (length) bytes, (start) is a byte address
uint8_t write_eeprom_chunk(unsigned int start, unsigned int length)
{
	// this writes byte-by-byte, page writing may be faster (4 bytes at a time)
	fill(length);
	prog_lamp(LOW);
	for (unsigned int x = 0; x < length; x++) {
		unsigned int addr = start + x;
		spi_transaction(0xC0, (addr >> 8) & 0xFF, addr & 0xFF, buff[x]);
		sleep_ms(45);
	}
	prog_lamp(HIGH);
	return STK_OK;
}

#define EECHUNK (32)
uint8_t write_eeprom(unsigned int length)
{
	// here is a word address, get the byte address
	unsigned int start = here * 2;
	unsigned int remaining = length;
	if (length > param.eepromsize) {
		ISPError++;
		return STK_FAILED;
	}
	while (remaining > EECHUNK) {
		write_eeprom_chunk(start, EECHUNK);
		start += EECHUNK;
		remaining -= EECHUNK;
	}
	write_eeprom_chunk(start, remaining);
	return STK_OK;
}

void program_page()
{
	char result = (char)STK_FAILED;
	unsigned int length = 256 * getch();
	length += getch();
	char memtype = getch();
	// flash memory @here, (length) bytes
	if (memtype == 'F') {
		write_flash(length);
		return;
	}
	if (memtype == 'E') {
		result = (char)write_eeprom(length);
		if (CRC_EOP == getch()) {
			usb_write_byte((char)STK_INSYNC);
			usb_write_byte(result);
		} else {
			ISPError++;
			usb_write_byte((char)STK_NOSYNC);
		}
		return;
	}
	usb_write_byte((char)STK_FAILED);
	return;
}

uint8_t flash_read(uint8_t hilo, unsigned int addr)
{
	return spi_transaction(0x20 + hilo * 8, (addr >> 8) & 0xFF, addr & 0xFF, 0);
}

char flash_read_page(int length)
{
	for (int x = 0; x < length; x += 2) {
		usb_poll_rx();
		uint8_t low = flash_read(LOW, here);
		usb_write_byte((char)low);
		uint8_t high = flash_read(HIGH, here);
		usb_write_byte((char)high);
		here++;
	}
	return STK_OK;
}

char eeprom_read_page(int length)
{
	// here again we have a word address
	int start = here * 2;
	for (int x = 0; x < length; x++) {
		int addr = start + x;
		uint8_t ee = spi_transaction(0xA0, (addr >> 8) & 0xFF, addr & 0xFF, 0xFF);
		usb_write_byte((char)ee);
	}
	return STK_OK;
}

void read_page()
{
	char result = (char)STK_FAILED;
	int length = 256 * getch();
	length += getch();
	char memtype = getch();
	if (CRC_EOP != getch()) {
		ISPError++;
		usb_write_byte((char)STK_NOSYNC);
		return;
	}
	usb_write_byte((char)STK_INSYNC);
	if (memtype == 'F') {
		result = flash_read_page(length);
	}
	if (memtype == 'E') {
		result = eeprom_read_page(length);
	}
	usb_write_byte(result);
}

void read_signature()
{
	if (CRC_EOP != getch()) {
		ISPError++;
		usb_write_byte((char)STK_NOSYNC);
		return;
	}
	usb_write_byte((char)STK_INSYNC);
	uint8_t high = spi_transaction(0x30, 0x00, 0x00, 0x00);
	usb_write_byte((char)high);
	uint8_t middle = spi_transaction(0x30, 0x00, 0x01, 0x00);
	usb_write_byte((char)middle);
	uint8_t low = spi_transaction(0x30, 0x00, 0x02, 0x00);
	usb_write_byte((char)low);
	usb_write_byte((char)STK_OK);
}
//////////////////////////////////////////
//////////////////////////////////////////

////////////////////////////////////
////////////////////////////////////
void avrisp()
{
	uint8_t ch = getch();
	switch (ch) {
	case '0': // signon
		ISPError = 0;
		empty_reply();
		break;
	case '1':
		if (getch() == CRC_EOP) {
			usb_write_byte((char)STK_INSYNC);
			usb_write_string("AVR ISP");
			usb_write_byte((char)STK_OK);
		} else {
			ISPError++;
			usb_write_byte((char)STK_NOSYNC);
		}
		break;
	case 'A':
		get_version(getch());
		break;
	case 'B':
		fill(20);
		set_parameters();
		empty_reply();
		break;
	case 'E': // extended parameters - ignore for now
		fill(5);
		empty_reply();
		break;
	case 'P':
		if (!pmode) {
			start_pmode();
		}
		empty_reply();
		break;
	case 'U': // set address (word)
		here = getch();
		here += 256 * getch();
		empty_reply();
		break;

	case 0x60: // STK_PROG_FLASH
		getch(); // low addr
		getch(); // high addr
		empty_reply();
		break;
	case 0x61: // STK_PROG_DATA
		getch(); // data
		empty_reply();
		break;

	case 0x64: // STK_PROG_PAGE
		program_page();
		break;

	case 0x74: // STK_READ_PAGE 't'
		read_page();
		break;

	case 'V': // 0x56
		universal();
		break;
	case 'Q': // 0x51
		ISPError = 0;
		end_pmode();
		empty_reply();
		break;

	case 0x75: // STK_READ_SIGN 'u'
		read_signature();
		break;

	case 'z':
		if (!pmode) {
			start_pmode();
		}
		read_signature();
		end_pmode();
		break;

	// expecting a command, not CRC_EOP
	// this is how we can get back in sync
	case CRC_EOP:
		ISPError++;
		usb_write_byte((char)STK_NOSYNC);
		break;

	// anything else we will return STK_UNKNOWN
	default:
		ISPError++;
		if (CRC_EOP == getch()) {
			usb_write_byte((char)STK_UNKNOWN);
		} else {
			usb_write_byte((char)STK_NOSYNC);
		}
	}
}

void isp_setup()
{
//	SERIAL_begin(/*BAUDRATE*/);
	pinMode(LED_PMODE, OUTPUT);
	pulse_pmode(2);
	pinMode(LED_ERR, OUTPUT);
	pulse_error(2);
//	pinMode(LED_HB, OUTPUT);
//	pulse(LED_HB, 2);
}

void isp_loop(void)
{
	// is pmode active?
	if (pmode) {
		led_pmode(true);
	} else {
		led_pmode(false);
	}
	// is there an error?
	if (ISPError) {
		led_error(true);
	} else {
		led_error(false);
	}

	// light the heartbeat LED
//	heartbeat();
	if (usb_read_available()) {
		avrisp();
	}
}




#define CLOCK 16000000UL
#define SCALE 125
static unsigned short _scale = 0;
static volatile unsigned long _system_tick_count;
static volatile unsigned long _tick_count;
static volatile unsigned long _time_s;
static unsigned short _time_ms;
uint8_t interval_1ms_flag;
ISR(TIMER0_OVF_vect, ISR_NOBLOCK)
{
	_system_tick_count++;
	_scale += 16;
	if (_scale >= SCALE) {
		_scale -= SCALE;
		_tick_count++;
		_time_ms++;
		if (_time_ms >= 1000) {
			_time_ms = 0;
			_time_s++;
		}
		interval_1ms_flag = 1;
	}
}

extern "C" void led(char f)
{
	if (f) {
		PORTB |= 0x01;
	} else {
		PORTB &= ~0x01;
	}
}

static inline int clamp(int v, int min, int max)
{
	if (v > max) v = max;
	if (v < min) v = min;
	return v;
}

void setup()
{
	// 16 MHz clock
	CLKPR = 0x80;
	CLKPR = 0;
	// Disable JTAG
	MCUCR |= 0x80;
	MCUCR |= 0x80;

	PORTB = 0x00;
	PORTC = 0x00;
	DDRB = 0x01;
	DDRC = 0x04;
	TCCR0B = 0x02; // 1/8 prescaling
	TIMSK0 |= 1 << TOIE0;

	data_rx_buffer_i = 0;
	data_rx_buffer_n = 0;
	data_tx_buffer_i = 0;
	data_tx_buffer_n = 0;

	usb_init();
	while (!usb_configured()) {
		_delay_ms(100);
	}

	isp_setup();


}

void loop()
{
	usb_poll_tx();
	usb_poll_rx();
#if 1
	isp_loop();
#else
	if (usb_read_available()) {
		char c = usb_read_byte();
		usb_write_byte(c);
		sleep_ms(10);
	}
#endif
}

int main()
{
	setup();
	sei();
	while (1) {
		loop();
	}
}

