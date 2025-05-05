#include "$className$DataModel.h"
#include <QDebug>

//#include <.hpp>

void $className$DataModel::spin(std::shared_ptr<rclcpp::Node> node)
{
	rclcpp::spin(node);
	rclcpp::shutdown();
}

void $className$DataModel::log(std::string to_log)
{
	qInfo(to_log.c_str());
}

bool $className$DataModel::setup(const QVariantMap& initialDataValues)
{
	if(!rclcpp::ok())
	{
		rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
	}

	m_node = rclcpp::Node::make_shared("$className$DataModelNode");
	m_threadSpin = std::make_shared<std::thread>(spin, m_node);
	//m_subscription = m_node->create_subscription<>(
	//	"/", 10, std::bind(&::topic_callback, this, std::placeholders::_1));

	return true;
}

//void $className$Skill::topic_callback(const ::SharedPtr msg) {
//}

