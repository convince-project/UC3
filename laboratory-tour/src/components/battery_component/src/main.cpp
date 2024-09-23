#include "BatteryComponent.h"
int main(int argc, char *argv[])
{
    yarp::os::Network yarp(yarp::os::YARP_CLOCK_SYSTEM);
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);
    BatteryComponent BatteryComponent;
    if (!BatteryComponent.start(argc, argv)) {
        return 1;
    }
    BatteryComponent.spin();

    BatteryComponent.close();

    return 0;
}
