#include "DialogComponent.hpp"

int main(int argc, char* argv[])
{
    yarp::os::Network yarp(yarp::os::YARP_CLOCK_SYSTEM);
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);
    DialogComponent dialogComponent;
    if (!dialogComponent.ConfigureYARP(rf)){
        yError() << "[main] Unable to configure YARP in DialogComponent";
        return 1;
    }
    if (!dialogComponent.start(argc, argv)) {
        yError() << "[main] Unable to start DialogComponent";
        return 1;
    }
    // blocking call
    rclcpp::executors::MultiThreadedExecutor m_executor;
    m_executor.add_node(dialogComponent.getNode());
    m_executor.spin();
    
    // dialogComponent.spin();

    dialogComponent.close();
    return 0;
}