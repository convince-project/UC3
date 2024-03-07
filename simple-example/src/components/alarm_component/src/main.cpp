#include "AlarmComponent.h"
int main(int argc, char *argv[])
{

    AlarmComponent AlarmComponent;
    if (!AlarmComponent.start(argc, argv)) {
        return 1;
    }
    AlarmComponent.spin();

    AlarmComponent.close();

    return 0;
}
