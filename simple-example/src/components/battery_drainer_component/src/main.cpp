#include "BatteryDrainerComponent.cpp"
int main(int argc, char *argv[])
{

    BatteryDrainerComponent batteryDrainerComponent;
    if (!batteryDrainerComponent.start(argc, argv)) {
        return 1;
    }
    batteryDrainerComponent.spin();

    batteryDrainerComponent.close();

    return 0;
}
