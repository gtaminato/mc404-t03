

source /home/specg12-1/mc404/simulador/set_path.sh

# echo 'Compile and link do_nothing:----------------------------'
# arm-eabi-as do_nothing.s -g -o do_nothing.o
# arm-eabi-ld do_nothing.o -o do_nothing -Tdata=0x77801900

echo 'Compile and link my program system:---------------------'
arm-eabi-as ra136034.s -g -o ra136034.o
# arm-eabi-ld ra136034.o -o ra136034 -g --section-start=.iv=0x778005e0 -Ttext=0x77800700 -Tdata=0x77801800 -e 0x778005e0
# echo 'Make SD card:-------------------------------------------'
# mksd.sh --so ra136034 --user do_nothing
# 
# rm *.o
# echo '--------------------------------------------------------'
# arm-sim --rom=/home/specg12-1/mc404/simulador/dumboot.bin --sd=disk.img

