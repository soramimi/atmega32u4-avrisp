// Arduino ISP Compatible for ATMEGA32U4
// Copyright (C) 2022 S.Fuchita (@soramimi_jp)

#include "usb.h"
#include <avr/interrupt.h>
#include <string.h>

uint16_t led_setup_count = 0;
uint16_t led_pmode_count = 0;
uint16_t led_error_count = 0;

extern "C" uint8_t usb_read_available_();
extern "C" uint8_t usb_read_byte_();

uint8_t data_tx_buffer[TX_EP_SIZE];
uint8_t data_tx_buffer_i;
uint8_t data_tx_buffer_n;

uint8_t data_rx_buffer[256];
int data_rx_buffer_i;
int data_rx_buffer_n;

extern "C" void clear_buffers()
{
	data_tx_buffer_i = 0;
	data_tx_buffer_n = 0;
	data_rx_buffer_i = 0;
	data_rx_buffer_n = 0;
}

void usb_poll_tx()
{
	uint8_t tmp[TX_EP_SIZE];
	for (uint8_t i = 0; i < data_tx_buffer_n; i++) {
		tmp[i] = data_tx_buffer[data_tx_buffer_i];
		data_tx_buffer_i = (data_tx_buffer_i + 1) % TX_EP_SIZE;
	}
	usb_data_tx(tmp, data_tx_buffer_n);
	data_tx_buffer_n = 0;
}


static inline void usb_poll_rx()
{
	int space = sizeof(data_rx_buffer) - 1 - data_rx_buffer_n;
	while (space > 0) {
		uint8_t tmp[16];
		int n = sizeof(tmp);
		n = usb_data_rx(tmp, n < space ? n : space);
		if (n == 0) break;
		for (uint8_t i = 0; i < n; i++) {
			int j = (data_rx_buffer_i + data_rx_buffer_n) % sizeof(data_rx_buffer);
			data_rx_buffer[j] = tmp[i];
			data_rx_buffer_n++;
		}
		space -= n;
	}
}

void usb_poll()
{
	usb_poll_tx();
	usb_poll_rx();
}

static inline int usb_read_available()
{
	return data_rx_buffer_n + usb_read_available_();
}

static inline uint8_t usb_read_byte()
{
	for (int i = 0; i < 2; i++) {
		if (data_rx_buffer_n > 0) {
			uint8_t c = data_rx_buffer[data_rx_buffer_i];
			data_rx_buffer_i = (data_rx_buffer_i + 1) % sizeof(data_rx_buffer);
			data_rx_buffer_n--;
			return c;
		}
		usb_poll_rx();
	}
	return 0;
}

void usb_write_byte(char c)
{
	while (1) {
		if (data_tx_buffer_n < sizeof(data_tx_buffer)) {
			int8_t i = (data_tx_buffer_i + data_tx_buffer_n) % sizeof(data_tx_buffer);
			data_tx_buffer[i] = c;
			data_tx_buffer_n++;
			if (data_tx_buffer_n >= TX_EP_SIZE) {
				usb_poll_tx();
			}
			return;
		}
		usb_poll();
	}
}

#define LOW 0
#define HIGH 1
#define INPUT 0
#define OUTPUT 1

#define PIN_RESET 6
#define PIN_MOSI 2
#define PIN_MISO 3
#define PIN_SCK 1
#define LED_PMODE 0
#define LED_ERROR 5

static inline void pinMode(char pin, char value)
{
	if (pin == LED_ERROR) {
		if (value) { DDRD |= 1 << pin; } else { DDRD &= ~(1 << pin); }
	} else {
		if (value) { DDRB |= 1 << pin; } else { DDRB &= ~(1 << pin); }
	}
}

static inline void digitalWrite(char pin, char value)
{
	if (pin == LED_ERROR) {
		if (value) { PORTD |= 1 << pin; } else { PORTD &= ~(1 << pin); }
	} else {
		if (value) { PORTB |= 1 << pin; } else { PORTB &= ~(1 << pin); }
	}
}

static inline bool digitalRead(char pin)
{
	if (pin == LED_ERROR) {
		return (PIND >> pin) & 1;
	} else {
		return (PINB >> pin) & 1;
	}
}

static inline void led_pmode(bool f)
{
	digitalWrite(LED_PMODE, f ? LOW : HIGH); // low active
}

static inline void led_error(bool f)
{
	digitalWrite(LED_ERROR, f ? LOW : HIGH); // low active
}

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

void usb_write_string(char const *p)
{
	while (*p) {
		usb_write_byte(*p);
		p++;
	}
}

void msleep(int ms)
{
	while (ms > 0) {
		usb_poll();
		_delay_ms(1);
		ms--;
	}
}

void usleep(int us)
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

#define USE_HARDWARE_SPI

#ifdef USE_HARDWARE_SPI
class SPI {
public:
	void begin()
	{
		digitalWrite(PIN_SCK, LOW);
		digitalWrite(PIN_MOSI, LOW);
		pinMode(PIN_SCK, OUTPUT);
		pinMode(PIN_MOSI, OUTPUT);
		pinMode(PIN_MISO, INPUT);

		// SPI: Mode 0, F_CLK/64
		SPCR = 0x52;
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
			_delay_us(1);
			b = (b << 1) | digitalRead(PIN_MISO);
			digitalWrite(PIN_SCK, LOW); // slow pulse
			_delay_us(1);
		}
		return b;
	}
};
#endif

static SPI SPI;

int error = 0;
bool pmode = 0;
// address for reading and writing, set by 'U' command
unsigned int here;
uint8_t buff[256]; // global block storage

#define beget16(addr) (*addr * 256 + *(addr + 1))
struct parameter_t {
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
};

parameter_t param;

static bool rst_active_high;

void reset_target(bool reset)
{
	digitalWrite(PIN_RESET, ((reset && rst_active_high) || (!reset && !rst_active_high)) ? HIGH : LOW);
}

uint8_t getch()
{
	while (usb_read_available() == 0);
	return usb_read_byte();
}

void fill(int n)
{
	for (int x = 0; x < n; x++) {
		buff[x] = getch();
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
		error++;
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
		error++;
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
	msleep(20); // discharge PIN_SCK, value arbitrarily chosen
	reset_target(false);
	// Pulse must be minimum 2 target CPU clock cycles so 100 usec is ok for CPU
	// speeds above 20 KHz
	usleep(100);
	reset_target(true);

	// Send the enable programming command:
	msleep(50); // datasheet: must be > 20 msec
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
	spi_transaction(0x4C, (addr >> 8) & 0xFF, addr & 0xFF, 0);
	msleep(10);
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
		error++;
		usb_write_byte((char)STK_NOSYNC);
	}
}

// write (length) bytes, (start) is a byte address
uint8_t write_eeprom_chunk(unsigned int start, unsigned int length)
{
	// this writes byte-by-byte, page writing may be faster (4 bytes at a time)
	fill(length);
	for (unsigned int x = 0; x < length; x++) {
		unsigned int addr = start + x;
		spi_transaction(0xC0, (addr >> 8) & 0xFF, addr & 0xFF, buff[x]);
		msleep(45);
	}
	return STK_OK;
}

#define EECHUNK (32)
uint8_t write_eeprom(unsigned int length)
{
	// here is a word address, get the byte address
	unsigned int start = here * 2;
	unsigned int remaining = length;
	if (length > param.eepromsize) {
		error++;
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
	} else if (memtype == 'E') {
		result = (char)write_eeprom(length);
		if (CRC_EOP == getch()) {
			usb_write_byte((char)STK_INSYNC);
			usb_write_byte(result);
		} else {
			error++;
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
		error++;
		usb_write_byte((char)STK_NOSYNC);
		return;
	}
	usb_write_byte((char)STK_INSYNC);
	if (memtype == 'F') {
		result = flash_read_page(length);
	} else if (memtype == 'E') {
		result = eeprom_read_page(length);
	}
	usb_write_byte(result);
}

void read_signature()
{
	if (CRC_EOP != getch()) {
		error++;
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

void avrisp()
{
	uint8_t ch = getch();
	switch (ch) {
	case '0': // signon
		error = 0;
		empty_reply();
		break;
	case '1':
		if (getch() == CRC_EOP) {
			usb_write_byte((char)STK_INSYNC);
			usb_write_string("AVR ISP");
			usb_write_byte((char)STK_OK);
		} else {
			error++;
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
		error = 0;
		end_pmode();
		empty_reply();
		break;

	case 0x75: // STK_READ_SIGN 'u'
		read_signature();
		break;

	// expecting a command, not CRC_EOP
	// this is how we can get back in sync
	case CRC_EOP:
		error++;
		usb_write_byte((char)STK_NOSYNC);
		break;

	// anything else we will return STK_UNKNOWN
	default:
		error++;
		if (CRC_EOP == getch()) {
			usb_write_byte((char)STK_UNKNOWN);
		} else {
			usb_write_byte((char)STK_NOSYNC);
		}
	}
}

#define SCALE 125
uint16_t _scale = 0;
ISR(TIMER0_OVF_vect, ISR_NOBLOCK)
{
	_scale += 16;
	if (_scale >= SCALE) {
		_scale -= SCALE;
		if (led_setup_count > 0) {
			led_setup_count--;
			bool f = (led_setup_count & 0xff) > 127;
			led_pmode(f);
			led_error(f);
		} else {
			if (led_pmode_count > 0) {
				led_pmode_count--;
				led_pmode((led_pmode_count & 0xff) > 127);
			}
			if (led_error_count > 0) {
				led_error_count--;
				led_error((led_error_count & 0xff) > 127);
			}
		}
	}
}

void isp_setup()
{
	pinMode(LED_PMODE, OUTPUT);
	pinMode(LED_ERROR, OUTPUT);
	led_setup_count = 1024;
}

void isp_loop()
{
	if (usb_read_available() > 0) {

		if (pmode) {
			if (led_pmode_count < 1024) {
				led_pmode_count += 1024;
			}
		}

		avrisp();
	}

	if (!pmode && led_pmode_count > 1) {
		led_pmode_count = 1;
	}

	if (error) {
		if (led_error_count < 1024) {
			led_error_count += 1024;
		}
	} else if (led_error_count > 1) {
		led_error_count = 1;
	}
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

	clear_buffers();

	usb_init();
	while (!is_usb_configured()) {
		_delay_ms(100);
	}

	isp_setup();
}

void loop()
{
	usb_poll();
	isp_loop();
}

int main()
{
	setup();
	sei();
	while (1) {
		loop();
	}
}
