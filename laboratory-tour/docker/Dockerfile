FROM elandini84/r1images:tourCore2_ubuntu22.04_iron_devel

USER root

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt upgrade -y && apt-get install -y ninja-build libzmq3-dev build-essential libgl1-mesa-dev
# libqt6scxml6 qt6-base-dev qt6-scxml-dev libqt6scxml6-bin

USER user1

RUN pip install reelay websocket-client jedi prompt_toolkit

RUN /bin/bash -c "cd /home/user1 && wget https://download.qt.io/official_releases/qt/6.6/6.6.3/single/qt-everywhere-src-6.6.3.tar.xz && \
                    tar -xvf qt-everywhere-src-6.6.3.tar.xz"
RUN /bin/bash -c "cd /home/user1/qt-everywhere-src-6.6.3 && \
                        ./configure -list-features"

RUN /bin/bash -c "cd /home/user1/qt-everywhere-src-6.6.3 && \
                    ./configure -prefix /home/user1/Qt6.6.3 -opensource -confirm-license -nomake examples -nomake tests -DBUILD_qt3d=OFF -DBUILD_qtquick3dphysics=OFF -DBUILD_qtquick3d=OFF  && \
                    cmake --build . --parallel"

RUN /bin/bash -c "cd /home/user1/qt-everywhere-src-6.6.3 && \
                    cmake --install ."

RUN /bin/bash -c "source /opt/ros/iron/setup.bash && \
    cd /home/user1 && git clone https://github.com/autonomy-and-verification-uol/ROSMonitoring.git -b ros2"

RUN /bin/bash -c "git clone --recurse-submodules https://github.com/BehaviorTree/Groot.git && \
    cd Groot && \
    cmake -S . -B build && \
    cmake --build build"

ENV Qt6_DIR=/home/user1/Qt6.6.3/lib/cmake

RUN /bin/bash -c "echo 'updated on 202401261057' && \
    source /opt/ros/iron/setup.bash && \
    cd /home/user1 && git clone https://github.com/convince-project/UC3.git -b laboratory_tour_cpp_datamodel"
RUN /bin/bash -c "source /opt/ros/iron/setup.bash && \
	cd /home/user1/UC3/laboratory-tour && \
    colcon build"

# RUN /bin/bash -c "cd /home/user1/ROSMonitoring/generator/ros2_devel && \
#     source /opt/ros/iron/setup.bash && \
#     python3 generator --config_file /home/user1/convince_bts/simple_example/monitoring/monitor.yaml && \
#     cd /home/user1/convince_bts/simple_example/monitor_ws/src/ && \
#     colcon build"

RUN echo "source /home/user1/UC3/laboratory-tour/install/setup.bash" >> ~/.bashrc
