/******************************************************************************
 *                                                                            *
 * Copyright (C) 2023 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
/**
 * @file main.cpp
 * @authors: Stefano Bernagozzi <stefano.bernagozzi@iit.it>
 */

#ifdef ZMQ_FOUND
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#endif

#include <iostream>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <ROS2Condition.h>
#include <ROS2Action.h>
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/actions/always_failure_node.h>
#include <behaviortree_cpp_v3/actions/always_success_node.h>

#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_minitrace_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>

#include <rclcpp/rclcpp.hpp>
#include <bt_interfaces_dummy/srv/reload_tree.hpp>


using namespace std;
using namespace BT;



class FlipFlopCondition : public ConditionNode
{

public:
    FlipFlopCondition(const std::string& name) :
        ConditionNode(name, {} )
    {
        setRegistrationID("FlipFlopCondition");
    }

private:
    int n = 0;
    virtual BT::NodeStatus tick() override
    {
        if(n++ < 30)
        {
            cout << "condition true" << endl;
            return NodeStatus::SUCCESS;
        }
        else
        {
            n = 0;
            cout << "condition false" << endl;
            return NodeStatus::FAILURE;
        }
    }
};


class AlwaysRunning : public ActionNodeBase
{

public:
    AlwaysRunning(const std::string& name) :
        ActionNodeBase(name, {} )
    {
        setRegistrationID("AlwaysRunning");
    }

private:
    virtual BT::NodeStatus tick() override
    {
        cout << "Action Ticked" << endl;
            return NodeStatus::RUNNING;
    }

    virtual void halt() override
    {
        cout << "Action halted" << endl;
    }
};

void ReloadTree(const std::shared_ptr<bt_interfaces_dummy::srv::ReloadTree::Request> request,
                std::shared_ptr<bt_interfaces_dummy::srv::ReloadTree::Response> response,
                std::unique_ptr<BT::Tree>& tree, const std::string file_path, bool halt, BehaviorTreeFactory bt_factory, std::unique_ptr<PublisherZMQ>& publisher_zmq)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reloading the behavior tree...");
    publisher_zmq.reset();
    tree = std::make_unique<BT::Tree>( bt_factory.createTreeFromFile( file_path ) );
    publisher_zmq = std::make_unique<PublisherZMQ>(*tree);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Done reloading the behavior tree...");
    response->is_ok = true;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    if (argc < 2)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Not enough arguments");
        return 1;
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "count " << argc << argv[0] << argv[1] << argv[2]);

    BehaviorTreeFactory bt_factory;
    bt_factory.registerNodeType<ROS2Action>("ROS2Action");
    bt_factory.registerNodeType<ROS2Condition>("ROS2Condition");
    bt_factory.registerNodeType<AlwaysRunning>("AlwaysRunning");

    rclcpp::Node::SharedPtr m_node = rclcpp::Node::make_shared("BtExecutableNode");
    

    // Create the tree
    // BT::Tree tree = bt_factory.createTreeFromFile(argv[1]);
    auto tree = std::make_unique<BT::Tree>( bt_factory.createTreeFromFile( argv[1] ) );
    const std::string path = argv[1];
    bool halt = false;

    // Create loggers
    StdCoutLogger logger_cout(*tree);
    MinitraceLogger logger_minitrace(*tree, "/tmp/bt_trace.json");
    FileLogger logger_file(*tree, "/tmp/bt_trace.fbl");

#ifdef ZMQ_FOUND
    // PublisherZMQ publisher_zmq(tree);
    auto publisher_zmq = std::make_unique<PublisherZMQ>(*tree);
#endif
    printTreeRecursively((*tree).rootNode());
    // Create the reload service and pass the tree by reference
    rclcpp::Service<bt_interfaces_dummy::srv::ReloadTree>::SharedPtr m_reloadTreeService = 
        m_node->create_service<bt_interfaces_dummy::srv::ReloadTree>(
            "/BtExecutable/ReloadTree",
            [&tree, &path, &halt, &bt_factory, &publisher_zmq](const std::shared_ptr<bt_interfaces_dummy::srv::ReloadTree::Request> request,
                    std::shared_ptr<bt_interfaces_dummy::srv::ReloadTree::Response> response)
            {
                ReloadTree(request, response, tree, path, halt, bt_factory, publisher_zmq);
            });
    while (rclcpp::ok())
    {
        (*tree).tickRoot();
        rclcpp::spin_some(m_node);

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    rclcpp::shutdown();
    return 0;
}