ARG base_img=ste93/convince:laboratory_tour_qt_6.6.3_ubuntu22.04_iron_stable
FROM $base_img
LABEL maintainer="Stefano Bernagozzi"
USER root
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt upgrade -y
USER user1
RUN /bin/bash -c "source /opt/ros/iron/setup.bash && \
    cd /home/user1 && git clone https://github.com/autonomy-and-verification-uol/ROSMonitoring.git -b ros2"

RUN /bin/bash -c "git clone --recurse-submodules https://github.com/BehaviorTree/Groot.git && \
    cd Groot && \
    cmake -S . -B build && \
    cmake --build build"

ENV Qt6_DIR=/home/user1/Qt6.6.3/lib/cmake
ARG UC3_BRANCH=main

RUN /bin/bash -c "echo 'updated on 202406271144' && \
    source /opt/ros/iron/setup.bash && \
    cd /home/user1 && git clone https://github.com/convince-project/UC3.git -b ${UC3_BRANCH}"
RUN /bin/bash -c "source /opt/ros/iron/setup.bash && \
	cd /home/user1/UC3/laboratory-tour && \
    colcon build"

# RUN /bin/bash -c "cd /home/user1/ROSMonitoring/generator/ros2_devel && \
    # source /opt/ros/iron/setup.bash && \
    # python3 generator --config_file /home/user1/convince_bts/simple_example/monitoring/monitor.yaml && \
    # cd /home/user1/convince_bts/simple_example/monitor_ws/src/ && \
    # colcon build"

RUN echo "source /home/user1/UC3/laboratory-tour/install/setup.bash" >> ~/.bashrc
