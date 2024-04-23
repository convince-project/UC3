/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/


#include "NarrateComponent.h"




bool NarrateComponent::start(int argc, char*argv[])
{

    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }
    m_node = rclcpp::Node::make_shared("NarrateComponentNode");
    m_narrateService = m_node->create_service<narrate_interfaces::srv::Narrate>("/NarrateComponent/Narrate",  
                                                                                std::bind(&NarrateComponent::Narrate,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_isDoneService = m_node->create_service<narrate_interfaces::srv::IsDone>("/NarrateComponent/IsDone",  
                                                                                std::bind(&NarrateComponent::IsDone,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));

    //setup the subscription to the DoneSpeaking message
    m_doneSpeakingSub = m_node->create_subscription<text_to_speech_interfaces::msg::DoneSpeaking>("/TextToSpeech/DoneSpeaking", 
                                                                                                    10, 
                                                                                                    [this](const text_to_speech_interfaces::msg::DoneSpeaking::SharedPtr msg) {
                                                                                                        m_isDoneSpeaking = msg->is_done;
                                                                                                    });

    RCLCPP_DEBUG(m_node->get_logger(), "NarrateComponent::start");
    std::cout << "NarrateComponent::start";        
    return true;

}



bool NarrateComponent::close()
{
    rclcpp::shutdown();  
    return true;
}

void NarrateComponent::spin()
{
    rclcpp::spin(m_node);  
}


void NarrateComponent::IsDone([[maybe_unused]] const std::shared_ptr<narrate_interfaces::srv::IsDone::Request> request,
             std::shared_ptr<narrate_interfaces::srv::IsDone::Response>      response) 
{
    response->is_done = m_doneWithPoi;
    response->is_ok = true;
}


void NarrateComponent::Narrate(const std::shared_ptr<narrate_interfaces::srv::Narrate::Request> request,
             std::shared_ptr<narrate_interfaces::srv::Narrate::Response>      response) 
{
    //create new thread with lambda function
    std::thread threadLocal([this, request](){
        m_doneWithPoi = false;
        do {
            //calls the GetCurrentAction service 
            auto getCurrentActionClientNode = rclcpp::Node::make_shared("NarrateComponentGetCurrentActionNode");
            std::shared_ptr<rclcpp::Client<scheduler_interfaces::srv::GetCurrentAction>> getCurrentActionClient = 
            getCurrentActionClientNode->create_client<scheduler_interfaces::srv::GetCurrentAction>("/SchedulerComponent/GetCurrentAction");
            auto getCurrentActionRequest = std::make_shared<scheduler_interfaces::srv::GetCurrentAction::Request>();

            while (!getCurrentActionClient->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'getCurrentActionClient'. Exiting.");
                }
            }
            auto getCurrentActionResult = getCurrentActionClient->async_send_request(getCurrentActionRequest);
            auto futureGetCurrentActionResult = rclcpp::spin_until_future_complete(getCurrentActionClientNode, getCurrentActionResult);
            auto currentAction = getCurrentActionResult.get();
            if (currentAction->type == "0") // 0 is speak
    
            {
                // // calls the Speak service
                // auto speakRequest = std::make_shared<text_to_speech_interfaces::srv::Speak::Request>();
                // speakRequest->text = currentAction->param;
                // auto speakResult = m_speakClient->async_send_request(speakRequest);
                // while (rclcpp::ok() && rclcpp::spin_until_future_complete(m_node, speakResult) != rclcpp::FutureReturnCode::SUCCESS)
                // {
                //     if (!rclcpp::ok())
                //     {
                //         RCLCPP_ERROR(m_node->get_logger(), "Interrupted while waiting for the result.");
                //         return;
                //     } else {
                //         RCLCPP_INFO(m_node->get_logger(), "Waiting for the result.");
                //     }
                //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
                // }
                // prints the text to the console
                std::cout << "NarrateComponent::Narratetype: " << currentAction->type << std::endl;
                std::cout << "NarrateComponent::Narrate: " << currentAction->param << std::endl;
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "NarrateComponent::Narrate: " << currentAction->param );
                std::this_thread::sleep_for(std::chrono::milliseconds(3000));
            }

            // //waits for the DoneSpeaking message
            // bool m_isDoneSpeaking = false;
            // while (!m_isDoneSpeaking)
            // {
            //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
            // }

            
            // calls the UpdateAction service
            auto updateActionClientNode = rclcpp::Node::make_shared("NarrateComponentUpdateActionNode");
            std::shared_ptr<rclcpp::Client<scheduler_interfaces::srv::UpdateAction>> updateActionClient = 
            updateActionClientNode->create_client<scheduler_interfaces::srv::UpdateAction>("/SchedulerComponent/UpdateAction");
            auto updateActionRequest = std::make_shared<scheduler_interfaces::srv::UpdateAction::Request>();

            while (!updateActionClient->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'updateActionClient'. Exiting.");
                }
            }
            auto updateActionResult = updateActionClient->async_send_request(updateActionRequest);

            auto futureUpdateActionResult = rclcpp::spin_until_future_complete(updateActionClientNode, updateActionResult); 
            
            auto updateActionResponse = updateActionResult.get();
            m_doneWithPoi = updateActionResponse->done_with_poi;
        } while (!m_doneWithPoi);  
    });
    m_threadNarration = std::move(threadLocal);
    response->is_ok = true;
}
