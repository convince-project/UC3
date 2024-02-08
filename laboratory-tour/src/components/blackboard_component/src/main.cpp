#include "BlackboardComponent.h"
int main(int argc, char *argv[])
{

    BlackboardComponent BlackboardComponent;
    if (!BlackboardComponent.start(argc, argv)) {
        return 1;
    }
    BlackboardComponent.spin();

    BlackboardComponent.close();

    return 0;
}
