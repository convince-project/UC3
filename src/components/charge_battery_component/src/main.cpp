#include "ChargeBatteryComponent.h"
int main(int argc, char *argv[])
{

    ChargeBatteryComponent ChargeBatteryComponent;
    if (!ChargeBatteryComponent.start(argc, argv)) {
        return 1;
    }
    ChargeBatteryComponent.spin();

    ChargeBatteryComponent.close();

    return 0;
}
