/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/


#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <drain_interfaces/srv/drain.hpp>
#include <set>

class BatteryDrainerComponent
{
public:
    BatteryDrainerComponent() = default;

    bool start(int argc, char*argv[])
    {

        if(!rclcpp::ok())
        {
            rclcpp::init(/*argc*/ argc, /*argv*/ argv);
        }


        m_node = rclcpp::Node::make_shared("BatteryDrainerComponentNode");
        m_drainService = m_node->create_service<drain_interfaces::srv::Drain>("/BatteryDrainerComponent/Drain",  
                                                                                    std::bind(&BatteryDrainerComponent::drain,
                                                                                    this,
                                                                                    std::placeholders::_1,
                                                                                    std::placeholders::_2));
        // RCLCPP_DEBUG(m_node->get_logger(), "BatteryDrainerComponent::start");
        std::cout << "BatteryDrainerComponent::start" << std::endl;        
        return true;


    }

    bool close()
    {
        rclcpp::shutdown();  
        return true;
    }

    void spin()
    {
        rclcpp::spin(m_node);  
    }

    void drain([[maybe_unused]] const std::shared_ptr<drain_interfaces::srv::Drain::Request> request,
               [[maybe_unused]] std::shared_ptr<drain_interfaces::srv::Drain::Response>      response) 
    {
        //m_ibattery->getBatteryCharge(m_level);
        
        // std::cout <<"draining" << std::endl;
        response->is_ok = true;
        std::this_thread::sleep_for (std::chrono::milliseconds(500));

    }

private:
    //double m_level { 100.0 };
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<drain_interfaces::srv::Drain>::SharedPtr m_drainService;

};
