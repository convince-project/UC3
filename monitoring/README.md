# Monitoring in UC3

## Launching the docker
To correctly monitor the use case, there is a pre-built docker image `ste93/convince:tour_ubuntu_24.04_qt_6.8.3_jazzy_verification_devel`
To pull the image:
```
docker pull ste93/convince:tour_ubuntu_24.04_qt_6.8.3_jazzy_verification_devel
```

To run the docker, use this command where the *<local_cyclone_dds_settings.xml>* corresponds to the local cyclone dds settings:

```
docker run --rm -it --privileged --network host --pid host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v <local_cyclone_dds_settings.xml>:/home/user1/UC3/docker/cyclone_dds_settings.xml -e QT_X11_NO_MITSHM=1 ste93/convince:tour_ubuntu_24.04_qt_6.8.3_jazzy_verification_devel
```

Inside the docker already pre-built there should be properties monitor and model monitor. 

If you want to run directly the monitors go to the section [Running](#Running)

## Building

### Building property monitors
**N.B. These steps needs to be done for each property** 
Inside the docker:
You should have inside the `/home/user1/UC3/monitoring folder` one *.yaml* file and one *.py* file for each property, on how to build those files see [MOON Documentation](https://github.com/convince-project/moon) To build each property execute the following steps


Source ros2 executable:
```
source /opt/ros/jazzy/setup.bash
```

Go to the generator folder 
```
cd /home/user1/ROSMonitoring/generator/ros2_devel
```

Generate the files for the specific property:
```
/home/user1/monitoring-python-env/bin/python generator --config_file /home/user1/UC3/monitoring/<your_property>.yaml
```

Then go to the folder with the generated files and build it. Here *<your_property_ws>* is the workspace defined inside the *.yaml* file:
```
cd /home/user1/UC3/monitoring/<your_property_ws>/src/
colcon build
```

### Building model monitors

## Running

### Running property monitor
Go into the monitoring folder:
```
cd /home/user1/UC3/monitoring
```
Then run the oracle with the correct property name, the correct name is the name of the file *.py* without the extension:
```
/home/user1/monitoring-python-env/bin/python /home/user1/ROSMonitoring/oracle/TLOracle/oracle.py --online --dense --property prop3
```

Once launched the oracle, you need to run the correct monitor:

Source the correct setup file 
```
source /home/user1/UC3/monitoring/<your_property_ws>/src/install/setup.bash
```
Then go into the correct folder:
```
cd /home/user1/UC3/monitoring/<your_property_ws>/src/
```
and launch the monitor:
```
ros2 launch monitor/launch/monitor.launch
```

### Running model monitor