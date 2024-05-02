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
    m_stopService = m_node->create_service<narrate_interfaces::srv::Stop>("/NarrateComponent/Stop",  
                                                                                std::bind(&NarrateComponent::Stop,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));

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
    if (m_threadNarration.joinable()) {
        m_threadNarration.join();
    }
    m_stopped = false;
    std::thread threadLocal([this, request](){
        // calls the SetCommand service
	    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "NEW_THREAD");
        auto setCommandClientNode = rclcpp::Node::make_shared("NarrateComponentSetCommandNode");
        std::shared_ptr<rclcpp::Client<scheduler_interfaces::srv::SetCommand>> setCommandClient =  
        setCommandClientNode->create_client<scheduler_interfaces::srv::SetCommand>("/SchedulerComponent/SetCommand");
        auto setCommandRequest = std::make_shared<scheduler_interfaces::srv::SetCommand::Request>();
        setCommandRequest->command = request->command;
        while (!setCommandClient->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'setCommandClient'. Exiting.");
            }
        }
        auto setCommandResult = setCommandClient->async_send_request(setCommandRequest);
        auto futureSetCommandResult = rclcpp::spin_until_future_complete(setCommandClientNode, setCommandResult);
        auto setCommandResponse = setCommandResult.get();
        if (setCommandResponse->is_ok == false) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Error in SetCommand service" << setCommandResponse->error_msg);
            
        }
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
            if (currentAction->type == "speak") // 0 is speak
    
            {
                std::cout << "Speak Action" << std::endl;
                auto speakClientNode = rclcpp::Node::make_shared("NarrateComponentSpeakNode");
                std::shared_ptr<rclcpp::Client<text_to_speech_interfaces::srv::Speak>> speakClient = 
                speakClientNode->create_client<text_to_speech_interfaces::srv::Speak>("/TextToSpeechComponent/Speak");
                auto speakRequest = std::make_shared<text_to_speech_interfaces::srv::Speak::Request>();
                speakRequest->text = currentAction->param;

                while (!speakClient->wait_for_service(std::chrono::seconds(1))) {
                    if (!rclcpp::ok()) {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'speakClient'. Exiting.");
                    }
                }
                auto speakResult = speakClient->async_send_request(speakRequest);
                auto futureSpeakResult = rclcpp::spin_until_future_complete(speakClientNode, speakResult);
                auto speakFutureResult = speakResult.get();
                if (speakFutureResult->is_ok == true) { // 0 is speak
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
            }
            else{
                std::cout << " Other Action " << currentAction->type<< std::endl;
            }

            // waits until the robot has finished speaking
            bool isSpeaking = false;
            do {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                // calls the isSpeaking service
                auto isSpeakingClientNode = rclcpp::Node::make_shared("NarrateComponentIsSpeakingNode");
                std::shared_ptr<rclcpp::Client<text_to_speech_interfaces::srv::IsSpeaking>> isSpeakingClient =
                isSpeakingClientNode->create_client<text_to_speech_interfaces::srv::IsSpeaking>("/TextToSpeechComponent/IsSpeaking");
                auto isSpeakingRequest = std::make_shared<text_to_speech_interfaces::srv::IsSpeaking::Request>();
                while (!isSpeakingClient->wait_for_service(std::chrono::seconds(1))) {
                    if (!rclcpp::ok()) {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'isSpeakingClient'. Exiting.");
                    }
                }
                auto isSpeakingResult = isSpeakingClient->async_send_request(isSpeakingRequest);
                auto futureIsSpeakingResult = rclcpp::spin_until_future_complete(isSpeakingClientNode, isSpeakingResult);
                auto isSpeakingResponse = isSpeakingResult.get();
                isSpeaking = isSpeakingResponse->is_speaking;
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "is_speaking" << isSpeakingResponse->is_speaking); 
            } while (isSpeaking);
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
            if(m_stopped)
            {
                break;   
            }
	    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "donee_with_poi" << updateActionResponse->done_with_poi);
        } while (!m_doneWithPoi);  
        std::cout << " Done with Poi " << std::endl;
    });
    m_threadNarration = std::move(threadLocal);
    response->is_ok = true;
}


void NarrateComponent::Stop([[maybe_unused]] const std::shared_ptr<narrate_interfaces::srv::Stop::Request> request,
             std::shared_ptr<narrate_interfaces::srv::Stop::Response>      response) 
{
    m_stopped = true;
    if (m_threadNarration.joinable()) {
        m_threadNarration.join();
    }
    response->is_ok = true;
}