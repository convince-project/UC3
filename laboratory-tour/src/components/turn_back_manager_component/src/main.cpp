#include "TurnBackManagerComponent.h"
int main(int argc, char *argv[])
{
    TurnBackManagerComponent turnBackManagerComponent;
    if (!turnBackManagerComponent.start(argc, argv)) {
        return 1;
    }
    turnBackManagerComponent.spin();

    turnBackManagerComponent.close();

    return 0;
}
