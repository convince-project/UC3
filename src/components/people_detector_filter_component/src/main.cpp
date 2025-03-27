#include "PeopleDetectorFilterComponent.h"
int main(int argc, char *argv[])
{
    yarp::os::Network yarp(yarp::os::YARP_CLOCK_SYSTEM);
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);


    PeopleDetectorFilterComponent peopleDetectorFilterComponent;
    if (!peopleDetectorFilterComponent.start(argc, argv)) {
        return 1;
    }
    if (!peopleDetectorFilterComponent.ConfigureYARP(rf)){
        return 1;
    }
    if (!peopleDetectorFilterComponent.startComputeOutput()){
        return 1;
    }

    peopleDetectorFilterComponent.spin();

    peopleDetectorFilterComponent.close();

    return 0;
}
