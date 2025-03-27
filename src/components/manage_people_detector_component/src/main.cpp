#include "ManagePeopleDetectorComponent.h"
int main(int argc, char *argv[])
{

    ManagePeopleDetectorComponent ManagePeopleDetectorComponent;
    if (!ManagePeopleDetectorComponent.start(argc, argv)) {
        return 1;
    }
    ManagePeopleDetectorComponent.spin();

    ManagePeopleDetectorComponent.close();

    return 0;
}
