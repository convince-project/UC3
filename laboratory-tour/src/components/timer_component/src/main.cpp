#include "TimerComponent.h"
int main(int argc, char *argv[])
{

    TimerComponent TimerComponent;
    if (!TimerComponent.start(argc, argv)) {
        return 1;
    }
    TimerComponent.spin();

    TimerComponent.close();

    return 0;
}
