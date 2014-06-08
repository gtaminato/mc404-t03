arm-as=arm-eabi-as
arm-ld=arm-eabi-ld
arm-cc=arm-eabi-gcc
asflags=-g
ldflags=-g -Ttext=0x77802000

all: dummy_user

dummy_user: user.o modsi3.o divsi3.o syswrapper.o start.o
	$(arm-ld) start.o user.o syswrapper.o modsi3.o divsi3.o -o dummy_user $(ldflags) 

divsi3.o: divsi3.s
	$(arm-as) $(asflags) divsi3.s -o divsi3.o

modsi3.o: modsi3.s
	$(arm-as) $(asflags) modsi3.s -o modsi3.o

syswrapper.o: syswrapper.s
	$(arm-as) $(asflags) syswrapper.s -o syswrapper.o

user.o: user.s
	$(arm-as) $(asflags) user.s -o user.o

start.o: start.s
	$(arm-as) $(asflags) start.s -o start.o

clean:
	rm -f *.o dummy_user
