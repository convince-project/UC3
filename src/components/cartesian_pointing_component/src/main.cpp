#include "CartesianPointingComponent.h"
#include <yarp/os/Network.h>

int main(int argc, char *argv[])
{
    yarp::os::Network::init();

    CartesianPointingComponent node;
    if (!node.start(argc, argv)) {
        return 1;
    }
    node.spin();
    return 0;
}
