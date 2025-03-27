#include "yarp/os/Network.h"
#include "FaceExpressionsWrapper.hpp"

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;
    yarp::os::ResourceFinder rf;
    if (! rf.configure(argc, argv))
    {
        yWarning() << "[FaceExpressionsWrapper][main] Unable to find .ini file, going with default values";
    }
    FaceExpressionsWrapper wrapper;
    if(! wrapper.configure(rf))
    {
        yError() << "[FaceExpressionsWrapper][main] Unable to configure module";
        return -1;
    }
    if (! wrapper.start(argc, argv))
    {
        yError() << "[FaceExpressionsWrapper][main] Unable to start module";
        return -1;
    }
    yInfo() << "[FaceExpressionsWrapper][main] Spinning node";
    wrapper.spin(); //blocking call

    wrapper.close();
    return 0;
}
