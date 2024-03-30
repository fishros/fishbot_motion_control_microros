FROM ros:humble-ros-base-jammy


RUN sudo apt update && apt install python3-pip python3-penv -y && echo "" >> /root/.bashrc &&  pip3 install -U PlatformIO

RUN echo "" >> /root/.bashrc &&  echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

ENTRYPOINT []



