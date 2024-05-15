#include "BatteryChargingComponent.h"
int main(int argc, char *argv[])
{

    BatteryChargingComponent BatteryChargingComponent;
    if (!BatteryChargingComponent.start(argc, argv)) {
        return 1;
    }
    BatteryChargingComponent.spin();

    BatteryChargingComponent.close();

    return 0;
}
