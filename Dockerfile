FROM ros:humble-ros-base-jammy


RUN sudo apt update && apt install python3-pip -y && echo "" >> /root/.bashrc && pip3 install -U PlatformIO \
&& pio pkg install --global --platform "platformio/espressif32" \
&& pio pkg install --global --tool "platformio/contrib-piohome" \
&& pio pkg install --global --tool "platformio/framework-arduinoespressif32" \
&& pio pkg install --global --tool "platformio/tool-scons" \
&& pio pkg install --global --tool "platformio/tool-mkfatfs" \
&& pio pkg install --global --tool "platformio/tool-mkspiffs" \
&& pio pkg install --global --tool "platformio/tool-mklittlefs"


RUN echo "" >> /root/.bashrc &&  echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

ENTRYPOINT []



