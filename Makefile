MCU = atmega32u4
F_CPU = 16000000

OBJ = \
	usb.o \
	main.o \
	waitloop.o

CFLAGS = -Os -mmcu=$(MCU) -DF_CPU=$(F_CPU)
CC = avr-gcc $(CFLAGS)
CXX = avr-g++ $(CFLAGS) -std=c++11

all: main.hex
	avr-size -C --mcu $(MCU) main.elf

main.hex: main.elf
	avr-objcopy -O ihex $< $@

main.elf: $(OBJ)
	$(CC) -c usb.c
	$(CXX) $(OBJ) -o main.elf

main.o: main.cpp
	$(CXX) -c $^ -o $@

queue16.o: queue16.cpp
	$(CXX) -c $^ -o $@

waitloop.o: waitloop.cpp
	$(CXX) -c $^ -o $@

usb.o: usb.c
	$(CC) -c $^ -o $@

clean:
	rm *.o
	rm *.elf
	rm *.hex

write: main.hex
	avrdude -c avrisp -P /dev/ttyUSB0 -b 19200 -p m32u4 -U hfuse:w:0xd9:m  -U lfuse:w:0x5e:m -U flash:w:main.hex

fetch:
	-avrdude -c avrisp -P /dev/ttyACM0 -b 19200 -p m32u2

fetch2:
	avrdude -c avrisp -P /dev/ttyUSB0 -b 19200 -p m32u2

write2:
	avrdude -c avrisp -P /dev/ttyUSB0 -b 19200 -p m32u4 -U hfuse:w:0xd9:m  -U lfuse:w:0x5e:m -U flash:w:ArduinoISP.ino.HEX

test:
	ftavr --avr-write-fuse-h 99 --avr-write-fuse-l ff

write3:
	avrdude -c avrisp -P /dev/ttyACM0 -b 19200 -p m328p -U efuse:w:0xff:m -U hfuse:w:0xd9:m  -U lfuse:w:0x62:m
