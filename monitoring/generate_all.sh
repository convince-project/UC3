#!/bin/bash
#
#

cd /home/user1/ROSMonitoring/generator/ros2_devel 

    /home/user1/monitoring-python-env/bin/python generator --config_file /home/user1/UC3/monitoring/monitor_prop1.yaml && \
    /home/user1/monitoring-python-env/bin/python generator --config_file /home/user1/UC3/monitoring/monitor_prop2.yaml && \
    /home/user1/monitoring-python-env/bin/python generator --config_file /home/user1/UC3/monitoring/monitor_prop3.yaml && \
    /home/user1/monitoring-python-env/bin/python generator --config_file /home/user1/UC3/monitoring/monitor_prop4.yaml && \
    /home/user1/monitoring-python-env/bin/python generator --config_file /home/user1/UC3/monitoring/monitor_prop5.yaml && \
    /home/user1/monitoring-python-env/bin/python generator --config_file /home/user1/UC3/monitoring/monitor_prop6.yaml && \
    /home/user1/monitoring-python-env/bin/python generator --config_file /home/user1/UC3/monitoring/monitor_prop7.yaml && \
    /home/user1/monitoring-python-env/bin/python generator --config_file /home/user1/UC3/monitoring/monitor_prop9.yaml && \
    /home/user1/monitoring-python-env/bin/python generator --config_file /home/user1/UC3/monitoring/monitor_prop8.yaml && \
    /home/user1/monitoring-python-env/bin/python generator --config_file /home/user1/UC3/monitoring/monitor_prop10POI1.yaml && \
    /home/user1/monitoring-python-env/bin/python generator --config_file /home/user1/UC3/monitoring/monitor_prop10POI2.yaml && \
    /home/user1/monitoring-python-env/bin/python generator --config_file /home/user1/UC3/monitoring/monitor_prop10POI3.yaml && \
    /home/user1/monitoring-python-env/bin/python generator --config_file /home/user1/UC3/monitoring/monitor_prop10POI4.yaml && \
    /home/user1/monitoring-python-env/bin/python generator --config_file /home/user1/UC3/monitoring/monitor_prop10POI5.yaml && \
    /home/user1/monitoring-python-env/bin/python generator --config_file /home/user1/UC3/monitoring/monitor_prop11.yaml && \
    /home/user1/monitoring-python-env/bin/python generator --config_file /home/user1/UC3/monitoring/monitor_prop12.yaml && \
    /home/user1/monitoring-python-env/bin/python generator --config_file /home/user1/UC3/monitoring/monitor_prop13.yaml && \
    /home/user1/monitoring-python-env/bin/python generator --config_file /home/user1/UC3/monitoring/monitor_prop14.yaml && \
    /home/user1/monitoring-python-env/bin/python generator --config_file /home/user1/UC3/monitoring/monitor_prop15.yaml && \
    cd /home/user1/UC3/monitoring/monitor_prop1_ws/src/ && \
    colcon build && \
    cd /home/user1/UC3/monitoring/monitor_prop2_ws/src/ && \
    colcon build && \
    cd /home/user1/UC3/monitoring/monitor_prop3_ws/src/ && \
    colcon build && \
    cd /home/user1/UC3/monitoring/monitor_prop4_ws/src/ && \
    colcon build && \

    cd /home/user1/UC3/monitoring/monitor_prop5_ws/src/ && \
    colcon build && \
    cd /home/user1/UC3/monitoring/monitor_prop6_ws/src/ && \
    colcon build && \
    cd /home/user1/UC3/monitoring/monitor_prop7_ws/src/ && \
    colcon build && \
    cd /home/user1/UC3/monitoring/monitor_prop8_ws/src/ && \
    colcon build && \
    cd /home/user1/UC3/monitoring/monitor_prop9_ws/src/ && \
    colcon build && \
    cd /home/user1/UC3/monitoring/monitor_prop10POI1_ws/src/ && \
    colcon build && \
    cd /home/user1/UC3/monitoring/monitor_prop10POI2_ws/src/ && \
    colcon build && \
    cd /home/user1/UC3/monitoring/monitor_prop10POI3_ws/src/ && \
    colcon build && \
    cd /home/user1/UC3/monitoring/monitor_prop10POI4_ws/src/ && \
    colcon build && \
    cd /home/user1/UC3/monitoring/monitor_prop10POI5_ws/src/ && \
    colcon build && \
    cd /home/user1/UC3/monitoring/monitor_prop11_ws/src/ && \
    colcon build && \
    cd /home/user1/UC3/monitoring/monitor_prop12_ws/src/ && \
    colcon build && \
    cd /home/user1/UC3/monitoring/monitor_prop13_ws/src/ && \
    colcon build && \
    cd /home/user1/UC3/monitoring/monitor_prop14_ws/src/ && \
    colcon build && \
    cd /home/user1/UC3/monitoring/monitor_prop15_ws/src/ && \
    colcon build 

