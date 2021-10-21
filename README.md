# soft_65c816_core_SoC_23LC512

This project comprises a 65c816 Soft Core SoC with Static SRAM Driver (23LC512) and XMODEM Bootloader to load your programs into RAM from a PC via a terminal program. The target here is a Xilinx Arty A7, using a soft 65c816 core, with distributed memory for the RAM and ROM, though the design has an additional 64k mapped from 0x10000-0x20000.

The 65c816 is the 8/16-bit big brother of the 65c02, used in the SNES and Apple II GS. https://www.westerndesigncenter.com/wdc/documentation/w65c816s.pdf The 65c816 core is adapted from a SNES FPGA project.

The SoC has a small monitor in ROM ($C000-$FFFF), a soft ACIA at $8000, and (distributed) RAM from $0000-$7FFF. The LEDs indicate reset status, and BTN0 serves as the reset (active high). The constraints file shows which pins to connect from the 'c816 to the Arty. The monitor resides in the COE file.

This design also requires a 1.8432mhz external oscillator (clock) to be connected to the Arty so that the ACIA will work properly (currently at pin IO33).

Note: You must clock the device itself ABOVE 2mhz. I've tested it up to 7mhz.

You can load your programs into RAM as .prg files. Enter "X" at the monitor prompt, and you can load your program via the XMODEM protocol. I use ExtraPuTTY. Note: you can control where in RAM your programs load via your .prg files. I've been coding using Visual Studio and RetroAssembler. https://enginedesigns.net/retroassembler/

Run your program by using the address + "R" at the monitor prompt. For example, if you load your program at $1000, you would type 1000R and then enter. Make sure to jump or rts out to $C385 at the end of your program. This is the exit point that will return you to your monitor.

The project includes a QSPI (Quad SPI) driver written for the specs of the 23LC512 by Microchip. 

"L" clears the Zero Page and resets the stack from the monitor. I am currently working on a design that allows the user to replace the ROM monitor at will.

The other monitor function work as a typical WozMon would: (e.g.), https://www.sbprojects.net/projects/apple1/wozmon.php

Feel free to contact me through the 6502 forum: http://forum.6502.org/memberlist.php?mode=viewprofile&u=3597
