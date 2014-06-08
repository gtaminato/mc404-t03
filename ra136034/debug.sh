source /home/specg12-1/mc404/simulador/set_path.sh

echo 'Compile and link do_nothing:----------------------------'
arm-eabi-as do_nothing.s -g -o do_nothing.o
arm-eabi-ld do_nothing.o -o do_nothing -Tdata=0x77801900

echo 'Compile and link my program system:---------------------'
arm-eabi-as ra146446.s -g -o ra146446.o
arm-eabi-ld ra146446.o -o ra146446 -g --section-start=.iv=0x778005e0 -Ttext=0x77800700 -Tdata=0x77801800 -e 0x778005e0
echo 'Make SD card:-------------------------------------------'
mksd.sh --so ra146446 --user do_nothing

rm *.o
echo '--------------------------------------------------------'
arm-sim --rom=/home/specg12-1/mc404/simulador/dumboot.bin --sd=disk.img -g