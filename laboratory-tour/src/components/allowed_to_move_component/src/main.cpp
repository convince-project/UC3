#include "AllowedToMoveComponent.h"
int main(int argc, char *argv[])
{

    AllowedToMoveComponent AllowedToMoveComponent;
    if (!AllowedToMoveComponent.start(argc, argv)) {
        return 1;
    }
    AllowedToMoveComponent.spin();

    AllowedToMoveComponent.close();

    return 0;
}
