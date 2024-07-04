#include "LogComponent.h"
int main(int argc, char *argv[])
{
    LogComponent logComponent;
    if (!logComponent.start(argc, argv)) {
        return 1;
    }
    logComponent.spin();

    logComponent.close();

    return 0;
}
