
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
#include <assert.h>

#ifdef _WIN32
#else
#include <assert.h>
#include <unistd.h>
#endif

constexpr int BITBANG_MODE_TIMEOUT_MS = 10;
constexpr int PIN_IO_TIMEOUT_MS = 10;

void msleep(int ms)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

class Connection {
public:
	static constexpr int PIN_RST  = 0;
	static constexpr int PIN_SCK  = 1;
	static constexpr int PIN_MOSI = 2;
	static constexpr int PIN_MISO = 3;
	
	static constexpr int CMD_READ        = 0x80;
	static constexpr int CMD_READ_PULLUP = 0x90;
	static constexpr int CMD_WRITE_LOW   = 0xa0;
	static constexpr int CMD_WRITE_HIGH  = 0xb0;
		
private:
	Serial serial_;
private:
	int write(void const *ptr, int len)
	{
		return serial_.write(ptr, len);
	}
	
	int read(void *ptr, int len, int timeout)
	{
		return serial_.read(ptr, len, timeout);
	}
	bool sync()
	{
		int n;
		char tmp[100];
		for (int i = 0; i < 50; i++) {
			tmp[0] = '0';
			serial_.write(tmp, 1);
			n = serial_.read(tmp, sizeof(tmp), BITBANG_MODE_TIMEOUT_MS);
			if (n == 0) {
				tmp[0] = ' ';
				serial_.write(tmp, 1);
				n = serial_.read(tmp, sizeof(tmp), BITBANG_MODE_TIMEOUT_MS);
				if (n == 2 && tmp[0] == 0x14 && tmp[1] == 0x10) return true; // OK
			}
			msleep(10);
		}
		return false;
	}
public:
	Connection()
	{
	}
	
	~Connection()
	{
		close();
	}
	
	bool open(Serial::Option *option)
	{
		return serial_.open(option);
	}
	
	void close()
	{
		serial_.close();
	}

	bool enter_bitbang_mode()
	{
		static constexpr std::string_view command = ".BITBANG.";
		static constexpr std::string_view expect = "BITBANG\r\n";
		
		if (sync()) {
			serial_.write(command.data(), (int)command.size());
			
			char buf[expect.size()];
			int n = serial_.read(buf, (int)sizeof(buf), BITBANG_MODE_TIMEOUT_MS);
			if (n == (int)expect.size() && memcmp(buf, expect.data(), expect.size()) == 0) {
				return true;
			}
		}
		
		return false;
	}
	
	int read_pin(int pin, bool pullup)
	{
		char c = char(pullup ? CMD_READ_PULLUP : CMD_READ) | (pin & 0x0f);
		char d = 0;
		serial_.write(&c, 1);
		int n = serial_.read(&d, 1, PIN_IO_TIMEOUT_MS);
		return n == 1 ? (unsigned char)d : -1;
	}
	
	bool write_pin(int pin, bool v)
	{
		char c = char(v ? CMD_WRITE_HIGH : CMD_WRITE_LOW) | (pin & 0x0f);
		char d;
		serial_.write(&c, 1);
		int n = serial_.read(&d, 1, PIN_IO_TIMEOUT_MS);
		return n == 1;
	}
};

// JJY（日本標準時電波）の信号を FTDI ピンで制御するクラス
class JJY {
public:
	enum Freq {
		FREQ_40KHZ = 0,
		FREQ_60KHZ = 1,
	};
	struct Option {
		Serial::Option serial_options;
		Freq freq = FREQ_40KHZ;
	};
private:
	JJY::Option const &opts_;
	Connection *conn_;
public:
	JJY(JJY::Option const &opts, Connection *conn)
		: opts_(opts)
		, conn_(conn)
	{
		init();
	}
	
	// 各ピンを出力に設定
	void init()
	{
		assert(conn_);
		enable(false);
		freq(opts_.freq);
		pulse(false);
		msleep(10);
		enable(true);
	}
	
	// 信号出力の有効/無効を切り替え
	void enable(bool f)
	{
		assert(conn_);
		conn_->write_pin(Connection::PIN_RST, f);
	}
	
	// 周波数選択
	void freq(Freq f)
	{
		assert(conn_);
		conn_->write_pin(Connection::PIN_SCK, f);
	}
	
	// 変調パルス出力（キャリアの ON/OFF）
	void pulse(bool f)
	{
		assert(conn_);
		conn_->write_pin(Connection::PIN_MOSI, f);
	}
};

namespace {

// 年月日から简化ユリウス通日（CJD）へ変換
unsigned long convert_ymd_to_cjd(int year, int month, int day)
{
	if (month < 3) {
		month += 9;
		year--;
	} else {
		month -= 3;
	}
	year += 4800;
	int c = year / 100;
	return c * 146097 / 4 + (year - c * 100) * 1461 / 4 + (153 * month + 2) / 5 + day - 32045;
}

// 简化ユリウス通日（CJD）から年月日へ逆変換
void convert_cjd_to_ymd(unsigned long j, int *year, int *month, int *day)
{
	int y, m, d;
	y = (j * 4 + 128179) / 146097;
	d = (j * 4 - y * 146097 + 128179) / 4 * 4 + 3;
	j = d / 1461;
	d = (d - j * 1461) / 4 * 5 + 2;
	m = d / 153;
	d = (d - m * 153) / 5 + 1;
	y = (y - 48) * 100 + j;
	if (m < 10) {
		m += 3;
	} else {
		m -= 9;
		y++;
	}
	*year = y;
	*month = m;
	*day = d;
}

// 64bit 値の偶数パリティ（1 のビット数が奇数なら 1）を計算
bool parity(uint64_t bits)
{
	bits ^= bits >> 32;
	bits ^= bits >> 16;
	bits ^= bits >> 8;
	bits ^= bits >> 4;
	bits ^= bits >> 2;
	bits ^= bits >> 1;
	return bits & 1;
}

// 1秒間に送出する JJY コードの要素
enum class Playing : uint8_t {
	Marker, // 位置マーカー（200ms パルス）
	Value0, // データ 0（800ms パルス）
	Value1, // データ 1（500ms パルス）
};

// 日時を保持する構造体
struct DateTime {
	int year = 0;
	int month = 0;
	int day = 0;
	int hour = 0;
	int minute = 0;
	int second = 0;
	int ms = 0;
};

// 指定日時から JJY の 60 フレーム分の変調パターンを生成する
// JJY フォーマット: 分（8bit）、時（7bit）、年日（9+4bit）、年（8bit）、曜日（3bit）、パリティ等
void make_data(DateTime const &dt, std::vector<Playing> *out)
{
	out->clear();
	out->reserve(60);
	
	unsigned long day = convert_ymd_to_cjd(dt.year, dt.month, dt.day);
	int wd = (day + 1) % 7; // 0=日曜, ... 6=土曜
	day = day - convert_ymd_to_cjd(dt.year, 1, 1) + 1;
	
	uint64_t bits;
	auto push = [&](bool v){
		bits <<= 1;
		if (v) bits |= 1;
	};
	
	/* 01/52 */ push((dt.minute / 10) & 4);
	/* 02/51 */ push((dt.minute / 10) & 2);
	/* 03/50 */ push((dt.minute / 10) & 1);
	/* 04/49 */ push(false);
	/* 05/48 */ push((dt.minute % 10) & 8);
	/* 06/47 */ push((dt.minute % 10) & 4);
	/* 07/46 */ push((dt.minute % 10) & 2);
	/* 08/45 */ push((dt.minute % 10) & 1);
	
	/* 10/44 */ push(false);
	/* 11/43 */ push(false);
	/* 12/42 */ push((dt.hour / 10) & 2);
	/* 13/41 */ push((dt.hour / 10) & 1);
	/* 14/40 */ push(false);
	/* 15/39 */ push((dt.hour % 10) & 8);
	/* 16/38 */ push((dt.hour % 10) & 4);
	/* 17/37 */ push((dt.hour % 10) & 2);
	/* 18/36 */ push((dt.hour % 10) & 1);
	
	/* 20/35 */ push(false);
	/* 21/34 */ push(false);
	/* 22/33 */ push((day / 100) & 2);
	/* 23/32 */ push((day / 100) & 1);
	/* 24/31 */ push(false);
	/* 25/30 */ push((day / 10 % 10) & 8);
	/* 26/29 */ push((day / 10 % 10) & 4);
	/* 27/28 */ push((day / 10 % 10) & 2);
	/* 28/27 */ push((day / 10 % 10) & 1);
	
	/* 30/26 */ push((day % 10) & 8);
	/* 31/25 */ push((day % 10) & 4);
	/* 32/24 */ push((day % 10) & 2);
	/* 33/23 */ push((day % 10) & 1);
	/* 34/22 */ push(false);
	/* 35/21 */ push(false);
	/* 36/20 */ push(false); // p1
	/* 37/19 */ push(false); // p2
	/* 38/18 */ push(false);
	
	/* 40/17 */ push(false);
	/* 41/16 */ push((dt.year / 10 % 10) & 8);
	/* 42/15 */ push((dt.year / 10 % 10) & 4);
	/* 43/14 */ push((dt.year / 10 % 10) & 2);
	/* 44/13 */ push((dt.year / 10 % 10) & 1);
	/* 45/12 */ push((dt.year % 10) & 8);
	/* 46/11 */ push((dt.year % 10) & 4);
	/* 47/10 */ push((dt.year % 10) & 2);
	/* 48/09 */ push((dt.year % 10) & 1);
	
	/* 50/08 */ push(wd & 4);
	/* 51/07 */ push(wd & 2);
	/* 52/06 */ push(wd & 1);
	/* 53/05 */ push(false);
	/* 54/04 */ push(false);
	/* 55/03 */ push(false);
	/* 56/02 */ push(false);
	/* 57/01 */ push(false);
	/* 58/00 */ push(false);
	
	// 偶数パリティを計算して設定
	if (parity(bits & (0xffLL << 45))) bits |= 1 << 19;
	if (parity(bits & (0x7fLL << 36))) bits |= 1 << 20;
	
	// 60 フレーム（0〜59秒）の変調パターンを生成
	for (int i = 0; i < 60; i++) {
		if (i == 0 || i % 10 == 9) {
			out->push_back(Playing::Marker);
		} else {
			bool v = bits & (1LL << 52);
			out->push_back(v ? Playing::Value1 : Playing::Value0);
			bits <<= 1;
		}
	}
}

// ローカルタイムゾーンでの現在日時を取得（ミリ秒まで）
void getCurrentDateTime(DateTime *dt)
{
#if 0
	time_t t = time(nullptr);
	auto *tm = localtime(&t);
	std::chrono::system_clock::duration d = std::chrono::system_clock::now().time_since_epoch();
	long long ms = std::chrono::duration_cast<std::chrono::milliseconds>(d).count();
	ms += tm->tm_gmtoff * 1000;
	dt->ms = ms % 1000;
	dt->second = ms / 1000 % 60;
	dt->minute = ms / 60000 % 60;
	dt->hour = ms / 3600000 % 24;
	long long j = ms / 86400000 + 2440588;
	convert_cjd_to_ymd(j, &dt->year, &dt->month, &dt->day);
#else
	std::chrono::system_clock::duration d = std::chrono::system_clock::now().time_since_epoch();
	long long ms = std::chrono::duration_cast<std::chrono::milliseconds>(d).count();
	time_t t = ms / 1000;
	struct tm tm;
#ifdef _WIN32
	localtime_s(&tm, &t);
#else
	localtime_r(&t, &tm);
#endif
	dt->year = tm.tm_year + 1900;
	dt->month = tm.tm_mon + 1;
	dt->day = tm.tm_mday;
	dt->hour = tm.tm_hour;
	dt->minute = tm.tm_min;
	dt->second = tm.tm_sec;
	dt->ms = ms % 1000;
#endif
}

} // namespace

std::unique_ptr<JJY> jjy;

void jjy_loop()
{
	static int sec = -1;
	static int dur = 0;
	static bool pulse = false;
	static std::vector<Playing> playing;
	
	DateTime dt;
	getCurrentDateTime(&dt);
	
	auto Print = [&dt](char c){
		printf("\r" "%d-%02d-%02d %02d:%02d:%02d %c ", dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second, c);
		fflush(stdout);
	};
	
	if (dt.second != sec) {
		// 秒が変わったら、該当フレームのパルスを開始
		sec = dt.second;
		if (dt.second == 0 || playing.size() != 60) {
			make_data(dt, &playing);
		}
		
		pulse = true;
		jjy->pulse(pulse);
		char c = ' ';
		
		switch (playing[dt.second]) {
		case Playing::Marker: dur = 200; c = 'M'; break;
		case Playing::Value0: dur = 800; c = '0'; break;
		case Playing::Value1: dur = 500; c = '1'; break;
		}
		
		putchar('\n');		
		Print(c);
	} else if (dt.ms < dur) {
		// パルス幅が終了するまで待機
		msleep(dur - dt.ms);
	} else if (pulse) {
		// パルス OFF にして次の秒まで待機
		dur = 1000;
		pulse = false;
		jjy->pulse(pulse);
		Print(' ');
	} else {
		std::this_thread::yield();
	}
}

void main2(JJY::Option const &opts, Connection *conn)
{
	if (!conn->enter_bitbang_mode()) {
		fprintf(stderr, "failed to enter bitbang mode\n");
		return;
	}
	
	jjy = std::make_unique<JJY>(opts, conn);
	jjy->freq(opts.freq);
	
	while (1) {
		jjy_loop();
	}
}

void test(JJY::Option const &opts, Connection *conn)
{
	if (!conn->enter_bitbang_mode()) {
		fprintf(stderr, "failed to enter bitbang mode\n");
		return;
	}
	
	conn->write_pin(Connection::PIN_RST, true);
	conn->write_pin(Connection::PIN_SCK, false);
	conn->write_pin(Connection::PIN_MOSI, true);
	conn->write_pin(Connection::PIN_SCK, true);
	conn->write_pin(Connection::PIN_SCK, false);
	conn->write_pin(Connection::PIN_MOSI, false);
}

int main(int argc, char **argv)
{
	JJY::Option opts;
	opts.serial_options.speed = 115200;
	
	int argi = 1;
	while (argi < argc) {
		std::string_view arg = argv[argi++];
		if (arg == "-p") {
			if (argi < argc) {
				opts.serial_options.port = argv[argi++];
			} else {
				fprintf(stderr, "-p requires a port name\n");
				return 1;
			}
		} else if (arg == "-s") {
			if (argi < argc) {
				opts.serial_options.speed = atoi(argv[argi++]);
			} else {
				fprintf(stderr, "-s requires a speed value\n");
				return 1;
			}
		} else if (arg == "-f") {
			if (argi < argc) {
				std::string_view f = argv[argi++];
				if (f == "40") {
					opts.freq = JJY::FREQ_40KHZ;
				} else if (f == "60") {
					opts.freq = JJY::FREQ_60KHZ;
				}
			} else {
				fprintf(stderr, "-f requires a frequency value (40 or 60)\n");
				return 1;
			}
		}
	}

#ifdef _WIN32
	if (strncmp(opts.serial_options.port.c_str(), "COM", 3) == 0) {
		opts.serial_options.port = "\\\\.\\" + opts.serial_options.port;
	}
#else
#endif

	Connection conn;
	if (!conn.open(&opts.serial_options)) {
		fprintf(stderr, "failed to open %s\n", opts.serial_options.port.c_str());
		return 1;
	}
	
	if (1) {
		main2(opts, &conn);
	} else {
		test(opts, &conn);
	}
	
	conn.close();
	return 0;
}
