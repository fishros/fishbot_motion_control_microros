source /opt/ros/humble/setup.bash
mkdir -p /github/home/.platformio/penv/bin/
touch /github/home/.platformio/penv/bin/activate
pio run
rm -rf .pio/libdeps/featheresp32/micro_ros_platformio
pio run