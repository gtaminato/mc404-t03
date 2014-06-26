mount:
	arm-eabi-as ra136034.s -g -o ra136034.o
	arm-eabi-ld ra136034.o -o ra136034 -g --section-start=.iv=0x778005e0 -Ttext=0x77800700 -Tdata=0x77801800 -e 0x778005e0