#include "SchedulerComponent.h"
int main(int argc, char *argv[])
{

    SchedulerComponent SchedulerComponent;
    if (!SchedulerComponent.start(argc, argv)) {
        return 1;
    }
    SchedulerComponent.spin();

    SchedulerComponent.close();

    return 0;
}
