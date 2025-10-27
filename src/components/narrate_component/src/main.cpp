#include "NarrateComponent.h"
int main(int argc, char *argv[])
{

    NarrateComponent narrateComponent;
    yarp::os::Network yarp(yarp::os::YARP_CLOCK_SYSTEM);
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);
    if (!narrateComponent.configureYARP(rf)){
        yError() << "[main] Unable to configure YARP in NarrateComponent";
        return 1;
    }
    std::cout << "YARP configured successfully in main" << std::endl;
    if (!narrateComponent.start(argc, argv)) {
        yError() << "[main] Unable to start NarrateComponent";
        return 1;
    }
    std::cout << "NarrateComponent started successfully in main" << std::endl;
    // blocking call
    // rclcpp::executors::MultiThreadedExecutor m_executor;
    // m_executor.add_node(narrateComponent.getNode());
    // m_executor.spin();

    narrateComponent.spin();
    std::cout << "NarrateComponent spinning in main" << std::endl;

    narrateComponent.close();
    return 0;
}
