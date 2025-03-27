#include "TimerCheckForPeopleComponent.h"
int main(int argc, char *argv[])
{

    TimerCheckForPeopleComponent TimerCheckForPeopleComponent;
    if (!TimerCheckForPeopleComponent.start(argc, argv)) {
        return 1;
    }
    TimerCheckForPeopleComponent.spin();

    TimerCheckForPeopleComponent.close();

    return 0;
}
