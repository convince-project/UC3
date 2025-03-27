#include "NotifyUserComponent.h"
int main(int argc, char *argv[])
{

    NotifyUserComponent NotifyUserComponent;
    if (!NotifyUserComponent.start(argc, argv)) {
        return 1;
    }
    NotifyUserComponent.spin();

    NotifyUserComponent.close();

    return 0;
}
