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
    m_status = msg->power_supply_status;
}