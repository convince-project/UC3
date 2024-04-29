#include "ExecuteDanceComponent.h"


int main(int argc, char *argv[])
{
    ExecuteDanceComponent executeDanceComponent;
    if (!executeDanceComponent.start(argc, argv)) {
        return 1;
    }
    executeDanceComponent.spin();
    executeDanceComponent.close();
    return 0;
}
