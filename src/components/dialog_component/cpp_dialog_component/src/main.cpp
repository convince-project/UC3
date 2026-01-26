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
    std::cout << "YARP configured successfully in main" << std::endl;
    if (!dialogComponent.start(argc, argv)) {
        yError() << "[main] Unable to start DialogComponent";
        return 1;
    }
    std::cout << "DialogComponent started successfully in main" << std::endl;
    // blocking call
    rclcpp::executors::MultiThreadedExecutor m_executor;
    std::cout << "Spinning DialogComponent in main" << std::endl;
    m_executor.add_node(dialogComponent.getNode());
    std::cout << "Added DialogComponent node to executor in main" << std::endl;
    m_executor.spin();
    std::cout << "DialogComponent spun successfully in main" << std::endl;
    
    // dialogComponent.spin();
    // std::cout << "DialogComponent spinning in main" << std::endl;

    // dialogComponent.close();
    return 0;
}