FROM ros:humble-ros-base-jammy


RUN sudo apt update && apt install python3-pip -y && echo "" >> /root/.bashrc && pip3 install -U PlatformIO \
&& pio pkg install --global --platform "platformio/espressif32" \
&& pio pkg install --global --tool "platformio/contrib-piohome" \
&& pio pkg install --global --tool "platformio/framework-arduinoespressif32" \
&& pio pkg install --global --tool "platformio/tool-scons" \
&& pio pkg install --global --tool "platformio/tool-mkfatfs" \
&& pio pkg install --global --tool "platformio/tool-mkspiffs" \
&& pio pkg install --global --tool "platformio/tool-mklittlefs"

RUN sudo apt update && apt install ros-humble-ros-bash -y

RUN sudo apt update && sudo apt install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-flake8 \
    python3-flake8-docstrings \
    python3-pip \
    python3-pytest \
    python3-pytest-cov \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    wget \
    libacl1-dev
    

RUN sudo apt install -y \
    python3-flake8-blind-except \
    python3-flake8-builtins \
    python3-flake8-class-newline \
    python3-flake8-comprehensions \ 
    python3-flake8-deprecated \
    python3-flake8-import-order \
    python3-flake8-quotes \
    python3-pytest-repeat \
    python3-pytest-rerunfailures

RUN echo "" >> /root/.bashrc &&  echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

ENTRYPOINT []



