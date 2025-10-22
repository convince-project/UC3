#include <iostream>
#include "CheckNetworkComponent.h"
#include <yarp/os/Network.h>

#include <thread>
#include <chrono>

int main(int argc, char *argv[])
{

    CheckNetworkComponent component;
    component.set_name("CheckNetworkComponent");
    yarp::os::Network yarp(yarp::os::YARP_CLOCK_SYSTEM);
    std::cout << "YARP configured successfully in main" << std::endl;
    if (!component.setup(argc, argv)) {
        RCLCPP_ERROR(component.getLogger(), "[main] Unable to start CheckNetworkComponent");
        return 1;
    }
    component.start();
    return 0;
}

