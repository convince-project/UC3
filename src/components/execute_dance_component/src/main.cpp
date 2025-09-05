#include "ExecuteDanceComponent.h"


int main(int argc, char *argv[])
{
    yarp::os::Network yarp(yarp::os::YARP_CLOCK_SYSTEM);
    ExecuteDanceComponent executeDanceComponent;
    if (!executeDanceComponent.start(argc, argv)) {
        return 1;
    }
    executeDanceComponent.spin();
    executeDanceComponent.close();
    return 0;
}
