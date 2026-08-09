
#include "serial.h"
#include <chrono>
#include <cstdio>
#include <deque>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>
#include <cstring>

#ifdef _WIN32
#else
#include <unistd.h>
#endif

#define PIN_RST  (0)
#define PIN_SCK  (1)
#define PIN_MOSI (2)
#define PIN_MISO (3)

#define CMD_READ        (0x80)
#define CMD_READ_PULLUP (0x90)
#define CMD_WRITE_LOW   (0xa0)
#define CMD_WRITE_HIGH  (0xb0)

constexpr int BITBANG_MODE_TIMEOUT_MS = 10;
constexpr int PIN_IO_TIMEOUT_MS = 10;

void msleep(int ms)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

class Connection {
private:
	Serial serial_;
public:
	Connection()
	{
	}
	~Connection()
	{
		close();
	}
public:
	bool open(Serial::Option *option)
	{
		return serial_.open(option);
	}
	
	void close()
	{
		serial_.close();
	}

	int write(void const *ptr, int len)
	{
		return serial_.write(ptr, len);
	}
	
	int read(void *ptr, int len, int timeout)
	{
		return serial_.read(ptr, len, timeout);
	}
};

bool enter_bitbang_mode(Connection *conn)
{
	static constexpr std::string_view command = ".BITBANG.";
	static constexpr std::string_view expect = "BITBANG\r\n";
	
	conn->write(command.data(), (int)command.size());

	char buf[expect.size()];
	int n = conn->read(buf, (int)sizeof(buf), BITBANG_MODE_TIMEOUT_MS);
	if (n == (int)expect.size() && memcmp(buf, expect.data(), expect.size()) == 0) {
		return true;
	}
	return false;
}

int read_pin(Connection *conn, int pin, bool pullup)
{
	char c = char(pullup ? CMD_READ_PULLUP : CMD_READ) | (pin & 0x0f);
	char d = 0;
	conn->write(&c, 1);
	int n = conn->read(&d, 1, PIN_IO_TIMEOUT_MS);
	return n == 1 ? (unsigned char)d : -1;
}

bool write_pin(Connection *conn, int pin, bool v)
{
	char c = char(v ? CMD_WRITE_HIGH : CMD_WRITE_LOW) | (pin & 0x0f);
	char d;
	conn->write(&c, 1);
	int n = conn->read(&d, 1, PIN_IO_TIMEOUT_MS);
	return n == 1;
}

void main2(Connection *conn)
{
	if (!enter_bitbang_mode(conn)) {
		fprintf(stderr, "failed to enter bitbang mode\n");
		return;
	}
	
	for (int i = 0; i < 10; i++) {
		if (!write_pin(conn, PIN_RST, true)) {
			fprintf(stderr, "failed to write pin at iteration %d\n", i);
			return;
		}
		msleep(500);
		if (!write_pin(conn, PIN_RST, false)) {
			fprintf(stderr, "failed to write pin at iteration %d\n", i);
			return;
		}
		msleep(500);
	}	
}

int main()
{
	Serial::Option opt;
	Connection conn;
#ifdef _WIN32
	opt.port = "\\\\.\\COM4";
#else
	opt.port = "/dev/ttyACM0";
#endif
	opt.speed = 115200;
	if (!conn.open(&opt)) {
		fprintf(stderr, "failed to open %s\n", opt.port.c_str());
		return 1;
	}
	main2(&conn);
	conn.close();
	return 0;
}
