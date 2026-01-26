#include "ExecuteAudioComponent.h"


int main(int argc, char *argv[])
{
    yarp::os::Network yarp(yarp::os::YARP_CLOCK_SYSTEM);
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);
    ExecuteAudioComponent executeAudioComponent;
    if (!executeAudioComponent.ConfigureYARP(rf)){
        yError() << "[main] Unable to configure YARP in ExecuteAudioComponent";
        return 1;
    }
    std::cout << "YARP configured successfully in main" << std::endl;

    if (!executeAudioComponent.start(argc, argv)) {
        yError() << "[main] Unable to start ExecuteAudioComponent";
        return 1;
    }
    executeAudioComponent.spin();
    std::cout << "ExecuteAudioComponent started successfully in main" << std::endl;
    executeAudioComponent.close();
    return 0;
}