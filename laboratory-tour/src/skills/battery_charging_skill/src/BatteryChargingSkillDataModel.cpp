#include "BatteryChargingSkillDataModel.h"
#include <QDebug>


bool BatteryChargingSkillDataModel::setup(const QVariantMap& initialDataValues)
{

    
    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
    }

    m_node = rclcpp::Node::make_shared("BatteryChargingSkillDataModelNode");
    m_threadSpin = std::make_shared<std::thread>(spin, m_node);
    m_subscription = m_node->create_subscription<sensor_msgs::msg::BatteryState>(
     "/battery_status", 10, std::bind(&BatteryChargingSkillDataModel::topic_battery_callback, this, std::placeholders::_1));
    return true;
}


void BatteryChargingSkillDataModel::spin(std::shared_ptr<rclcpp::Node> node)
{
    rclcpp::spin(node);  
    rclcpp::shutdown();  
}



void BatteryChargingSkillDataModel::log(std::string to_log) {
    qInfo(to_log.c_str());
}


void BatteryChargingSkillDataModel::topic_battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
    // m_status = msg->power_supply_status;
    m_old_voltage = m_voltage;
    m_voltage = msg->voltage;
    if(m_voltage > m_old_voltage) //da considerare caso in cui alimentatore connesso e tensione passa da es. 28.5 a 28.3
    {
        log("Battery is charging");
        m_charging = true;
    }
    else
    {
        log("Battery is not charging");
        m_charging = false;
    }
}