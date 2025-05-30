#include <iostream>
#include "DummyCondition.h"
#include <yarp/os/ResourceFinder.h>

#include <thread>
#include <chrono>



int main(int argc, char *argv[])
{
    yarp::os::Network yarp(yarp::os::YARP_CLOCK_SYSTEM);
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);
    if(!rf.check("skill_name"))
    {
        std::cout << "ERROR: Missing skill_name parameter\n";
        RCLCPP_ERROR(rclcpp::get_logger("DummyConditionLogger"), "Missing skill_name parameter");
        return 1;
    }
    if(!rf.check("default_status"))
    {
        std::cout << "ERROR: Missing default_status parameter\n";
        RCLCPP_ERROR(rclcpp::get_logger("DummyConditionLogger"), "Missing default_status parameter");
        return 1;
    }
    rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    std::shared_ptr<DummyCondition> dummyCondition = std::make_shared<DummyCondition>(rf.find("skill_name").asString(), rf.find("default_status").asString());
    rclcpp::spin(dummyCondition);
    rclcpp::shutdown();

    return 0;
}

