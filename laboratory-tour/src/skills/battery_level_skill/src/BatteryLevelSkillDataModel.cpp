#include "BatteryLevelSkillDataModel.h"
#include <QDebug>


bool BatteryLevelSkillDataModel::setup(const QVariantMap& initialDataValues)
{

    
    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
    }

    m_node = rclcpp::Node::make_shared("BatteryLevelSkillDataModelNode");
    m_threadSpin = std::make_shared<std::thread>(spin, m_node);
    m_subscription = m_node->create_subscription<sensor_msgs::msg::BatteryState>(
     "/battery_status", 10, std::bind(&BatteryLevelSkillDataModel::topic_battery_callback, this, std::placeholders::_1));
    return true;
}


void BatteryLevelSkillDataModel::spin(std::shared_ptr<rclcpp::Node> node)
{
    rclcpp::spin(node);  
    rclcpp::shutdown();  
}



void BatteryLevelSkillDataModel::log(std::string to_log) {
    qInfo(to_log.c_str());
}


void BatteryLevelSkillDataModel::topic_battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
    percentage = msg->percentage;
    // RCLCPP_INFO(m_node->get_logger(), "BatteryLevelSkill::topic_callback");
    // QVariantMap data;
    // data.insert("percentage", msg->percentage);
    // foreach (auto key, data.keys())
    // {
    //     qInfo()<<"-- key:"<<key<<" value:"<<data.value(key);
    // }
    // std::shared_ptr<QScxmlEvent> event = std::make_shared<QScxmlEvent>();
    // event->setName("BatteryDriverCmpInterface.readLevel");
    // event->setData(data);
    // event->setEventType(QScxmlEvent::EventType::ExternalEvent);
    // m_stateMachine.submitEvent(event.get());
    //m_stateMachine.submitEvent("BatteryDriverCmpInterface.readLevel", data);
}