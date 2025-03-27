#include "DanceComponent.h"
int main(int argc, char *argv[])
{
    // yarp::os::Network yarp(yarp::os::YARP_CLOCK_SYSTEM);
    // yarp::os::ResourceFinder rf;
    // rf.configure(argc, argv);


    DanceComponent danceComponent;
    if (!danceComponent.start(argc, argv)) {
        return 1;
    }
    danceComponent.spin();

    danceComponent.close();

    return 0;
}
