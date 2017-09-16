.DEFAULT_GOAL := no_args

TARGET = build/new_code

flash_image:
	make clean
	make -f make_flash.mk

debug_image:
	make clean
	make -f make_debug.mk

debug_server:
	sudo st-util

debug_gdb:
	arm-none-eabi-gdb $(TARGET).elf -ex "target remote localhost:4242" -ex "monitor reset halt" -ex "monitor semihosting enable" -ex "load"

flash:
	st-flash write $(TARGET).bin 0x8000000

no_args:
	@echo "USAGE:"
	@echo "make debug_image <OR> make flash_image"
	@echo "    Compiles your code, creates firmware images in build/"
	@echo "    Note: flash_image must be used to flash an image for"
	@echo "    standalone use on the chip (no debugger). debug_image"
	@echo "    retargets printf to the debugger, but will make the"
	@echo "    chip crash if a debugger is not attached. As far as"
	@echo "    I know, printf goes into the void in flash_images."
	@echo "    (It might go to a UART?)"
	@echo "    Note2: Every time you create a new image it just deletes"
	@echo "    the old one and replaces it with a new 'new_code.elf/bin'"
	@echo "make debug_server -- start the stlink debugger server"
	@echo "make debug_gdb -- connect GDB to the stlink debugger server"
	@echo "    Note: the device will start halted, use 'c', <enter>"
	@echo "    to execute your firmware image. printf outputs will"
	@echo "    appear in the debug_server process, not this one"
	@echo "    Note2: for some reason you have to unplug, replug the"
	@echo "    board before doing this for it to work reliably"
	@echo "make flash -- flash an image onto the chip"
	@echo "    This will burn a standalone image onto the chip."
	@echo "    Note that"

clean:
	rm -fR .dep build
