#include "TimeComponent.h"
int main(int argc, char *argv[])
{
    TimeComponent timeComponent;
    if (!timeComponent.start(argc, argv)) {
        return 1;
    }
    timeComponent.spin();

    timeComponent.close();

    return 0;
}
