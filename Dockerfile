FROM ros:humble
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
 git libqt5svg5-dev python3-pip python3-opencv libopencv-contrib-dev python3-tk \
 vim xterm less

RUN apt-get update && apt-get install -y --no-install-recommends \
 ros-humble-navigation2 \
 ros-humble-py-trees \
 ros-humble-py-trees-ros

WORKDIR /root
RUN echo "source /opt/ros/humble/setup.bash" >> .bashrc

COPY . /usr/local/lib/pytwb
WORKDIR /usr/local/lib/pytwb
RUN source /opt/ros/humble/setup.bash && pip3 install -e .
WORKDIR /root
