# Hey Emacs, this is a -*- makefile -*-
#----------------------------------------------------------------------------
# WinAVR Makefile Template written by Eric B. Weddington, Jörg Wunsch, et al.
#
# Released to the Public Domain
#
# Additional material for this makefile was written by:
# Peter Fleury
# Tim Henigan
# Colin O'Flynn
# Reiner Patommel
# Markus Pfaff
# Sander Pool
# Frederik Rouleau
# Carlos Lamas
#
#
# Fredrik Atmer then deleted most of the content for the AVR-Keyboard
#
#----------------------------------------------------------------------------
# On command line:
#
# make = Make software.
#
# make clean = Clean out built project files.
#
# To rebuild project do "make clean" then "make all".
#----------------------------------------------------------------------------

# Keyboard type (with micro controller code and speed)
#BOARD = phantom
#LAYOUT = ansi_iso
#LAYOUT = ansi_iso_win
#MCU = atmega32u4
#F_CPU = 16000000
#B_LOADER = \"jmp\ 0x7E00\"

# BOARD = hid_liber
# LAYOUT = ansi_iso_jis
# MCU = atmega32u2
# F_CPU = 16000000
# B_LOADER = \"jmp\ 0x7000\"

#BOARD = .
#LAYOUT = test
MCU = atmega32u4
F_CPU = 16000000
#B_LOADER = \"jmp\ 0x7000\"

#BOARD = sskb
#LAYOUT = symmetric
#LAYOUT = iso
#MCU = at90usb1286
#F_CPU = 16000000
#B_LOADER = \"jmp\ 0x1FC00\"

#BOARD = pontus
#LAYOUT = pontus
#MCU = at90usb1286
#F_CPU = 16000000
#B_LOADER = \"jmp\ 0x1FC00\"

# List C source files here.
#SRC =	main.c \
#	usb.c \
#	board.c

#	print.c \

#CDEFS = -DF_CPU=$(F_CPU)UL -D__INCLUDE_KEYBOARD=\"$(BOARD)/board.h\" -D__INCLUDE_LAYOUT=\"$(BOARD)/$(LAYOUT).h\" -D__BOOTLOADER_JUMP=$(B_LOADER)

# Optimization level, can be [0, 1, 2, 3, s]. 
#     0 = turn off optimization. s = optimize for size.
#     (Note: 3 is not always the best optimization level. See avr-libc FAQ.)
#OPTLEVEL = 2

#---------------- Compiler Options C ----------------
#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC manual and avr-libc documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
#CFLAGS = $(CDEFS)
#CFLAGS += -O$(OPTLEVEL)
#CFLAGS += -ffunction-sections
#CFLAGS += -Wall
#CFLAGS += -Wa,-adhlns=$(<:%.c=%.lst)
#CFLAGS += -std=gnu99

#---------------- Linker Options ----------------
#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LDFLAGS = -Os -Wl,-Map=main.map,--cref
LDFLAGS += -Wl,--relax
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -lm

# Define programs and commands.
SHELL = sh

# Define all object files.
OBJ = \
	usb.o \
	main.o \
	waitloop.o

CFLAGS = -Os -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Wall -Wextra -Werror=return-type -Wno-array-bounds
CC = avr-gcc $(CFLAGS)
CXX = avr-g++ $(CFLAGS) -std=c++11

# Change the build target to build a HEX file or a library.
all: main.hex
	avr-size -C main.elf

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
	-avrdude -c avrisp -P /dev/ttyACM0 -b 19200 -p m32u2 -v -v -v -v 

#	avrpi -w main.hex --avr-write-fuse-e f4 --avr-write-fuse-h d9 --avr-write-fuse-l 5e


fetch2:
	avrdude -c avrisp -P /dev/ttyUSB0 -b 19200 -p m32u2

write2:
	avrdude -c avrisp -P /dev/ttyUSB0 -b 19200 -p m32u4 -U hfuse:w:0xd9:m  -U lfuse:w:0x5e:m -U flash:w:ArduinoISP.ino.HEX
