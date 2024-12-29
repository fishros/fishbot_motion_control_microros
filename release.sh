export PATH=/home/runner/.platformio/penv/bin:/usr/local/bin/:$PATH

# 替换VERSION_CODE
sed -i "s/{VERSION}/$GITHUB_REF_NAME/g" include/fishbot_config.h
pip3 install esptool
pio lib install # Install dependencies
cp .config/microros/colcon.meta .pio/libdeps/fishbot_motion_control_humble/micro_ros_platformio/metas/colcon.meta
rm -rf .pio/libdeps/fishbot_motion_control_humble/micro_ros_platformio/libmicroros/
pio run

rm -rf bin && mkdir bin
export TNAME='fishbot_motion_control'
export TVERSION=$GITHUB_REF_NAME
export TDATA=`date +%y%m%d`
export BINNAME=`echo $TNAME`_$TVERSION.$TDATA.bin
export ELFNAME=`echo $TNAME`_$TVERSION.$TDATA.elf

# export boot_app0_dir="/root/.platformio/packages/framework-arduinoespressif32/tools/partitions"
esptool.py  --chip esp32 merge_bin -o bin/$BINNAME --flash_mode dio --flash_size 4MB 0x1000 .pio/build/fishbot_motion_control_humble/bootloader.bin 0x8000 .pio/build/fishbot_motion_control_humble/partitions.bin  0x10000 .pio/build/fishbot_motion_control_humble/firmware.bin 
cp .pio/build/fishbot_motion_control_humble/firmware.elf bin/$ELFNAME
echo "Build Finish bin/`ls bin`"