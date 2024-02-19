#include "NavigationComponent.h"
int main(int argc, char *argv[])
{
    yarp::os::Network yarp(yarp::os::YARP_CLOCK_SYSTEM);
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);


    NavigationComponent navigationComponent;
    if (!navigationComponent.start(argc, argv)) {
        return 1;
    }
    if (!navigationComponent.ConfigureYARP(rf)){
        return 1;
    }
    navigationComponent.spin();

    navigationComponent.close();

    return 0;
}
