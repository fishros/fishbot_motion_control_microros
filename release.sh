source /opt/ros/humble/setup.bash
link -s /opt/ros/humble/setup.bash /github/home/.platformio/penv/bin/activate
pio run
rm -rf .pio/libdeps/featheresp32/micro_ros_platformio
pio run