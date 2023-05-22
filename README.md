# FishBot运动控制程序MicroROS版

配套运动控制板（可以在小鱼的店铺直接购买，性价比接地气，直达链接：[https://item.taobao.com/item.htm?id=695473143304](https://item.taobao.com/item.htm?id=695473143304)）：

![](./docs/images/1670950515258-0c1474f6-2d5a-4030-a1df-87bfdff78ba5-image-resized.png)


## 上位机运行指令

需要提前安装docker，可以使用一键安装进行。


WIFI,UDP模式

```
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO udp4 --port 8888 -v6
```

Serial,串口模式

```bash
# Serial micro-ROS Agent
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO serial --dev /dev/ttyUSB0 -v6 -b 921600
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO serial --dev /dev/ttyUSB0 -v6 -b 115200
```

## 其他指令 

### Merge BootLoad&Frameware

```
export boot_app0_dir="/home/fishros/.platformio/packages/framework-arduinoespressif32/tools/partitions"
docker run -it --rm --privileged -v=/dev:/dev  -v $boot_app0_dir:$boot_app0_dir -v `pwd`:`pwd` -w `pwd` fishros2/fishbot-tool esptool.py  --chip esp32 merge_bin -o bin/fishbot_motion_control_four_driver_v1.0.0.`date +%y%m%d`.bin --flash_mode dio --flash_size 4MB 0x1000 .pio/build/featheresp32/bootloader.bin 0x8000 .pio/build/featheresp32/partitions.bin 0xe000 $boot_app0_dir/boot_app0.bin 0x10000 .pio/build/featheresp32/firmware.bin
```

