ARG base_img=ste93/convince:qt_6.6.3_ubuntu22.04_iron_stable
FROM $base_img
LABEL maintainer="Stefano Bernagozzi"
USER root

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update
#  && apt upgrade -y
RUN apt-get install -y libczmq-dev ros-iron-zmqpp-vendor-dbgsym ros-iron-zmqpp-vendor libczmq-dev libczmq4 

USER user1


RUN /bin/bash -c "git clone --recurse-submodules https://github.com/BehaviorTree/Groot.git && \
    cd Groot && \
    cmake -S . -B build && \
    cmake --build build"


ENV Qt6_DIR=/home/user1/Qt6.6.3/lib/cmake
# ARG UC3_BRANCH=dev/integ-meeting

RUN /bin/bash -c "echo 'updated on 202409251159' && \
    source /opt/ros/iron/setup.bash && \
    cd /home/user1 && git clone https://github.com/convince-project/UC3.git -b dev/docker-compose-no-halt && \
    source /opt/ros/iron/setup.bash && \
    cd /home/user1/UC3/laboratory-tour && \
    colcon build --packages-select notify_user_interfaces_dummy && \
    source /home/user1/UC3/laboratory-tour/install/setup.bash && \
    colcon build --packages-ignore BT face_expressions_wrapper bt_executable_reload"

# RUN /bin/bash -c "cd /home/user1/ROSMonitoring/generator/ros2_devel && \
#     source /opt/ros/iron/setup.bash && \
#     python3 generator --config_file /home/user1/convince_bts/simple_example/monitoring/monitor.yaml && \
#     cd /home/user1/convince_bts/simple_example/monitor_ws/src/ && \
#     colcon build"



RUN echo "source /home/user1/UC3/laboratory-tour/install/setup.bash" >> ~/.bashrc
RUN echo "export PATH=\$PATH:/home/user1/Groot/build" >> ~/.bashrc
