#include "TextToSpeechComponent.hpp"

int main(int argc, char* argv[])
{
    yarp::os::Network yarp(yarp::os::YARP_CLOCK_SYSTEM);
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);
    TextToSpeechComponent textToSpeechComponent;
    if (!textToSpeechComponent.ConfigureYARP(rf)){
        yError() << "[main] Unable to configure YARP in TextToSpeechComponent";
        return 1;
    }
    yInfo() << "[TextToSpeechComponent main] Configured YARP";
    if (!textToSpeechComponent.start(argc, argv)) {
        yError() << "[main] Unable to start TextToSpeechComponent";
        return 1;
    }
    yInfo() << "[TextToSpeechComponent main] started";
    // blocking call
    textToSpeechComponent.spin();

    textToSpeechComponent.close();
    return 0;
}