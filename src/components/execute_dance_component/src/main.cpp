#include "ExecuteDanceComponent.h"
#include <yarp/os/Network.h>


int main(int argc, char *argv[])
{
    yarp::os::Network::init();
    ExecuteDanceComponent executeDanceComponent;
    if (!executeDanceComponent.start(argc, argv)) {
        return 1;
    }
    executeDanceComponent.spin();
    // executeDanceComponent.close();
    return 0;
}
