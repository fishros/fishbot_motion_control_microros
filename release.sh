export PATH=/usr/local/bin/:$PATH

source /opt/ros/humble/setup.bash
mkdir -p /github/home/.platformio/penv/bin/
touch /github/home/.platformio/penv/bin/activate
pio run
cp .config/microros/colcon.meta .pio/libdeps/featheresp32/micro_ros_platformio/metas/colcon.meta
rm -rf .pio/libdeps/featheresp32/micro_ros_platformio/libmicroros/
pio run

rm -rf bin && mkdir bin
export TNAME='fishbot_motion_control'
export TVERSION=$GITHUB_REF_NAME
export TDATA=`date +%y%m%d`
export BINNAME=`echo $TNAME`_$TVERSION.$TDATA.bin

# export boot_app0_dir="/root/.platformio/packages/framework-arduinoespressif32/tools/partitions"
esptool.py  --chip esp32 merge_bin -o bin/$BINNAME --flash_mode dio --flash_size 4MB 0x1000 .pio/build/featheresp32/bootloader.bin 0x8000 .pio/build/featheresp32/partitions.bin  0x10000 .pio/build/featheresp32/firmware.bin 
echo "Build Finish bin/`ls bin`"