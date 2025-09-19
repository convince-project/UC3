#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <builtin_interfaces/msg/time.hpp>  

class ClockRos2Publisher : public rclcpp::Node
{
public:
    ClockRos2Publisher()
        : Node("monitoring_clock_ros2_publisher")
    {
        publisher_ = this->create_publisher<builtin_interfaces::msg::Time>("monitoring_clock", 10);
        yarp::os::Network yarp;
        if (!yarp.checkNetwork())
        {
            RCLCPP_ERROR(this->get_logger(), "YARP server not available!");
            return;
        }
        // yarp_port_.open("/clock_ros2_publisher");
        // yarp::os::Network::connect("/clock", "/clock_ros2_publisher");
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200), std::bind(&ClockRos2Publisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // yarp::os::Bottle *input = yarp_port_.read(false);
        auto input = yarp::os::Time::now();
        auto message = builtin_interfaces::msg::Time();
        auto time_in_string = std::to_string(input);
        // split second and nanosecond
        auto second = time_in_string.substr(0, time_in_string.find(" "));
        auto nanosecond = time_in_string.substr(time_in_string.find(" ") + 1, time_in_string.size());
        message.sec = std::stoi(second);
        message.nanosec = std::stoi(nanosecond);
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
        
    }

    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    yarp::os::Network yarp;
    auto node = std::make_shared<ClockRos2Publisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
