#include "NarrateComponent.h"
int main(int argc, char *argv[])
{

    NarrateComponent narrateComponent;
    if (!narrateComponent.start(argc, argv)) {
        return 1;
    }
    narrateComponent.spin();

    narrateComponent.close();

    return 0;
}
