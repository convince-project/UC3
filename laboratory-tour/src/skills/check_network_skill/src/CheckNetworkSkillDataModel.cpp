#include "CheckNetworkSkillDataModel.h"
#include <QDebug>


bool CheckNetworkSkillDataModel::setup(const QVariantMap& initialDataValues)
{

    
    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
    }

    m_node = rclcpp::Node::make_shared("CheckNetworkSkillDataModelNode");
    m_threadSpin = std::make_shared<std::thread>(spin, m_node);
    m_subscription = m_node->create_subscription<std_msgs::msg::Bool>(
     "/CheckNetworkComponent/NetworkStatus", 10, std::bind(&CheckNetworkSkillDataModel::topic_battery_callback, this, std::placeholders::_1));
    return true;
}


void CheckNetworkSkillDataModel::spin(std::shared_ptr<rclcpp::Node> node)
{
    rclcpp::spin(node);  
    rclcpp::shutdown();  
}



void CheckNetworkSkillDataModel::log(std::string to_log) {
    qInfo(to_log.c_str());
}


void CheckNetworkSkillDataModel::topic_battery_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    network_status = msg->data;
    // RCLCPP_INFO(m_node->get_logger(), "CheckNetworkSkill::topic_callback");
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