FROM elandini84/r1images:tourCore2_ubuntu22.04_iron_stable

USER root

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y libqt6scxml6 qt6-base-dev qt6-scxml-dev libqt6scxml6-bin libzmq3-dev

USER user1

RUN pip install reelay websocket-client jedi prompt_toolkit

RUN /bin/bash -c "source /opt/ros/iron/setup.bash && \
    cd /home/user1 && git clone https://github.com/autonomy-and-verification-uol/ROSMonitoring.git -b ros2"

RUN /bin/bash -c "git clone --recurse-submodules https://github.com/BehaviorTree/Groot.git && \
    cd Groot && \
    cmake -S . -B build && \
    cmake --build build"

RUN /bin/bash -c "echo 'updated on 202401261057' && \
    source /opt/ros/iron/setup.bash && \
    cd /home/user1 && git clone https://github.com/hsp-iit/convince_bts.git && \
    cd /home/user1/convince_bts/simple_example && \
    colcon build"

RUN /bin/bash -c "cd /home/user1/ROSMonitoring/generator/ros2_devel && \
    source /opt/ros/iron/setup.bash && \
    python3 generator --config_file /home/user1/convince_bts/simple_example/monitoring/monitor.yaml && \
    cd /home/user1/convince_bts/simple_example/monitor_ws/src/ && \
    colcon build"


RUN echo "source /home/user1/convince_bts/simple_example/install/setup.bash" >> ~/.bashrc 
