
# Change this to whatever AVR programmer you want to use.
PROGRAMMER = usbtiny

OUT=AC_Timer

CHIP = attiny85

CC = avr-gcc
OBJCPY = avr-objcopy
AVRDUDE = avrdude 
OPTS = -Os -g -std=c11 -Wall -Wno-main

CFLAGS = -mmcu=$(CHIP) $(OPTS)

%.o: %.c Makefile
	$(CC) $(CFLAGS) -c -o $@ $<

%.hex: %.elf
	$(OBJCPY) -j .text -j .data -O ihex $^ $@

%.elf: %.o
	$(CC) $(CFLAGS) -o $@ $^

all:	$(OUT).hex $(OUT).hex

clean:
	rm -f *.hex *.elf *.o

flash:	$(OUT).hex
	$(AVRDUDE) -c $(PROGRAMMER) -p $(CHIP) -U flash:w:$(OUT).hex

fuse:
	$(AVRDUDE) -c $(PROGRAMMER) -p $(CHIP) -U hfuse:w:0xdc:m -U lfuse:w:0x62:m -U efuse:w:0xff:m

init:	fuse flash
