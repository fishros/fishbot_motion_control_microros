FROM ros:humble-ros-base-jammy


RUN sudo apt update && sudo apt install  python3-pip -y && pip3 install -U PlatformIO && 
    echo "" >> /root/.bashrc &&
    echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

ENTRYPOINT ['/bin/bash']



