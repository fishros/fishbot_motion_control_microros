FROM ros:humble-ros-base-jammy


RUN sudo apt update && apt install python3-pip -y && echo "" >> /root/.bashrc &&  pip3 install -U PlatformIO esptool

RUN apt update && apt install ros-humble-ros-base zip -y 

RUN echo "" >> /root/.bashrc &&  echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

ENTRYPOINT []



