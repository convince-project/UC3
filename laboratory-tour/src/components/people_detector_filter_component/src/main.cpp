#include "PeopleDetectorFilterComponent.h"
int main(int argc, char *argv[])
{

    PeopleDetectorFilterComponent peopleDetectorFilterComponent;
    if (!peopleDetectorFilterComponent.start(argc, argv)) {
        return 1;
    }

    peopleDetectorFilterComponent.spin();

    peopleDetectorFilterComponent.close();

    return 0;
}
