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
    // response->is_done = (!m_speakTask && !m_danceTask);
    response->is_done = !(m_speakTask);
    response->is_ok = true;
}


void NarrateComponent::speakTask() {
    RCLCPP_INFO_STREAM(m_node->get_logger(), "New Speak Thread ");
    m_speakMutex.lock();
    int size = m_speakBuffer.size();
    m_speakMutex.unlock();
    do{
        m_speakMutex.lock();
        std::string text = m_speakBuffer.front();
        m_speakBuffer.pop();
        m_speakMutex.unlock();
        RCLCPP_INFO_STREAM(m_node->get_logger(), "Speak Action " << text);
        auto speakClientNode = rclcpp::Node::make_shared("NarrateComponentSpeakNode");
        std::shared_ptr<rclcpp::Client<text_to_speech_interfaces::srv::Speak>> speakClient = 
        speakClientNode->create_client<text_to_speech_interfaces::srv::Speak>("/TextToSpeechComponent/Speak");
        auto speakRequest = std::make_shared<text_to_speech_interfaces::srv::Speak::Request>();
        speakRequest->text = text;
        bool wait_succeded{true};
        int retries = 0;
        while (!speakClient->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service '/TextToSpeechComponent/Speak'. Exiting.");
                wait_succeded = false;
                break;
            }
            retries++;
            if(retries == SERVICE_TIMEOUT) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service '/TextToSpeechComponent/Speak'.");
                wait_succeded = false;
                break;
            }
        }
        if (!wait_succeded) {
            break;
        }
        auto speakResult = speakClient->async_send_request(speakRequest);
        auto futureSpeakResult = rclcpp::spin_until_future_complete(speakClientNode, speakResult);
        auto speakFutureResult = speakResult.get();
        if (speakFutureResult->is_ok == true) { // 0 is speak
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        // std::this_thread::sleep_for(std::chrono::seconds(30));
        // waits until the robot has finished speaking
        bool isSpeaking = false;
        do {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            // calls the isSpeaking service
            auto isSpeakingClientNode = rclcpp::Node::make_shared("NarrateComponentIsSpeakingNode");
            std::shared_ptr<rclcpp::Client<text_to_speech_interfaces::srv::IsSpeaking>> isSpeakingClient =
            isSpeakingClientNode->create_client<text_to_speech_interfaces::srv::IsSpeaking>("/TextToSpeechComponent/IsSpeaking");
            auto isSpeakingRequest = std::make_shared<text_to_speech_interfaces::srv::IsSpeaking::Request>();
            retries = 0;
            while (!isSpeakingClient->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service '/TextToSpeechComponent/IsSpeaking'. Exiting.");
                }
                retries++;
                if(retries == SERVICE_TIMEOUT) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service '/TextToSpeechComponent/IsSpeaking'.");
                wait_succeded = false;
                break;
                }
            }
            if (wait_succeded) {
                auto isSpeakingResult = isSpeakingClient->async_send_request(isSpeakingRequest);
                auto futureIsSpeakingResult = rclcpp::spin_until_future_complete(isSpeakingClientNode, isSpeakingResult);
                auto isSpeakingResponse = isSpeakingResult.get();
                isSpeaking = isSpeakingResponse->is_speaking;
                m_timeMutex.lock();
                m_seconds_left = isSpeakingResponse->seconds_left;
                m_timeMutex.unlock(); 
            }
        } while (isSpeaking);
        if(m_stopped)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Stop Recieved Speak Task" ); 
            break;   
        }
        m_speakMutex.lock();
        size = m_speakBuffer.size();
        m_speakMutex.unlock();
    }
    while(size);
    m_speakTask = false;
}

void NarrateComponent::danceTask() {
    RCLCPP_INFO_STREAM(m_node->get_logger(), "New Dance Thread ");
    // m_danceMutex.lock();
    // int size = m_danceBuffer.size();
    // m_danceMutex.unlock();
    // int l_seconds_left;
    // int old_randomNumber;
    // int randomNumber;
    // do{
    //     std::string text;
    //     m_timeMutex.lock();
    //     l_seconds_left = m_seconds_left;
    //     m_timeMutex.unlock();
    //     if(l_seconds_left < 3){
    //         // m_danceMutex.lock();
    //         // if(m_danceBuffer.size() != 0)
    //         // {
    //         //     m_danceBuffer.pop();
    //         // }
    //         // m_danceMutex.unlock();
    //         text = "idleMove";
    //     }
    //     else{
    //         m_danceMutex.lock();
    //         if(m_danceBuffer.size() != 0){
    //             text = m_danceBuffer.front();
    //             if(text == "argue"){
    //                 srand (time(NULL));
    //                 randomNumber = rand() % 3;
    //                 if(randomNumber == old_randomNumber)
    //                 {
    //                     randomNumber = (randomNumber + 1) % 3;
    //                 }
    //                 old_randomNumber = randomNumber;
    //                 text += std::to_string(randomNumber);
    //             }
    //         }
    //         m_danceMutex.unlock();
    //     }
    //     // m_danceMutex.lock();
    //     // text = m_danceBuffer.front();
    //     // m_danceBuffer.pop();
    //     // m_danceMutex.unlock();
    //     RCLCPP_INFO_STREAM(m_node->get_logger(), "Dance Action " << text); 
    //     auto danceClientNode = rclcpp::Node::make_shared("NarrateComponentDanceNode");
    //     std::shared_ptr<rclcpp::Client<execute_dance_interfaces::srv::ExecuteDance>> danceClient = 
    //     danceClientNode->create_client<execute_dance_interfaces::srv::ExecuteDance>("/ExecuteDanceComponent/ExecuteDance");
    //     auto danceRequest = std::make_shared<execute_dance_interfaces::srv::ExecuteDance::Request>();
    //     danceRequest->dance_name = text;
    //     bool wait_succeded{true};
    //     int retries = 0;
    //     while (!danceClient->wait_for_service(std::chrono::seconds(1))) {
    //         if (!rclcpp::ok()) {
    //             RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'ExecuteDance'. Exiting.");
    //             wait_succeded = false;
    //             break;
    //         }
    //         retries++;
    //         if(retries == SERVICE_TIMEOUT) {
    //            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service 'ExecuteDance'.");
    //            wait_succeded = false;
    //            break;
    //         }
    //     }
    //     if (wait_succeded) {
    //         auto danceResult = danceClient->async_send_request(danceRequest);
    //         const std::chrono::seconds timeout_duration(SERVICE_TIMEOUT);
    //         auto futureDanceResult = rclcpp::spin_until_future_complete(danceClientNode, danceResult, timeout_duration);
    //         if(futureDanceResult == rclcpp::FutureReturnCode::SUCCESS)
    //         {
    //             auto danceFutureResult = danceResult.get();
    //             if (danceFutureResult->is_ok == true) { // 0 is dance
    //                 std::this_thread::sleep_for(std::chrono::milliseconds(500));
    //             }
    //             std::this_thread::sleep_for(std::chrono::seconds(3));
    //         }
    //         else if(futureDanceResult == rclcpp::FutureReturnCode::TIMEOUT){
    //            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while future complete for the service 'ExecuteDance'.");
    //        }
    //     }
    //     if(m_stopped)
    //     {
    //         RCLCPP_INFO_STREAM(m_node->get_logger(), "Stop Recieved Dance Task" ); 
    //         break;   
    //     }
        
    //     m_danceMutex.lock();
    //     size = m_danceBuffer.size();
    //     m_danceMutex.unlock();
    //     m_timeMutex.lock();
    //     l_seconds_left = m_seconds_left;
    //     m_timeMutex.unlock();
    // }
    // while(l_seconds_left > 0 || size > 0);
    // // while(size);
    // m_danceTask = false;
}
        
void NarrateComponent::NarrateTask(const std::shared_ptr<narrate_interfaces::srv::Narrate::Request> request) {
        // calls the SetCommand service
	    RCLCPP_INFO_STREAM(m_node->get_logger(), "New Narrate Thread ");
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
        bool doneWithPoi = false;
        std::thread speakThread;
        std::thread danceThread;

        // bool m_doneWithPoi = false;
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
                if (currentAction->type == "speak")
                {
                    RCLCPP_INFO_STREAM(m_node->get_logger(), "Got speak action " ); 
                    m_speakMutex.lock();
                    m_speakBuffer.push(currentAction->param);
                    m_speakMutex.unlock();
                    if (!m_speakTask) {
                        m_speakTask = true;
                        // std::thread threadSpeak([=]() { speakTask();});
                        if (speakThread.joinable()) {
                            speakThread.join();
                        }
                        speakThread = std::thread(&NarrateComponent::speakTask, this);
                        // threadSpeak.detach();
                    } else {
                        RCLCPP_INFO_STREAM(m_node->get_logger(), "Speak Task is already running");
                    }
                }
                // else if (currentAction->type == "dance")
                // {
                //     RCLCPP_INFO_STREAM(m_node->get_logger(), "Got dance action " ); 
                //     m_danceMutex.lock();
                //     m_danceBuffer.push(currentAction->param);
                //     m_danceMutex.unlock();
                //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
                //     if (!m_danceTask) {
                //         m_danceTask = true;
                //         if (danceThread.joinable()) {
                //             danceThread.join();
                //         }
                //         danceThread = std::thread(&NarrateComponent::danceTask, this);
                //     } else {
                //         RCLCPP_INFO_STREAM(m_node->get_logger(), "Dance Task is already running");
                //     }
                // }
                if(currentAction->is_blocking)
                {
                    RCLCPP_INFO_STREAM(m_node->get_logger(), "Blocking action " ); 
                    if (speakThread.joinable()) {
                        speakThread.join();
                        // m_danceMutex.lock();
                        // while(m_danceBuffer.size() != 0)
                        // {
                        //     m_danceBuffer.pop();
                        // }
                        // m_danceMutex.unlock();
                        RCLCPP_INFO_STREAM(m_node->get_logger(), "Speak Thread Joined");
                    }
                    // if (danceThread.joinable() ) {
                    //     danceThread.join();
                    //     RCLCPP_INFO_STREAM(m_node->get_logger(), "Dance Thread Joined");
                    // }
                    RCLCPP_INFO_STREAM(m_node->get_logger(), "Both Threads Joined");
                }

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
            doneWithPoi = updateActionResponse->done_with_poi;
            if(m_stopped)
            {
                RCLCPP_INFO_STREAM(m_node->get_logger(), "Stop Command received");
                if (speakThread.joinable()) {
                    speakThread.join();
                    RCLCPP_INFO_STREAM(m_node->get_logger(), "Speak Thread Joined");
                }
                // if (danceThread.joinable() ) {
                //     danceThread.join();
                //     RCLCPP_INFO_STREAM(m_node->get_logger(), "Dance Thread Joined");
                // }
                RCLCPP_INFO_STREAM(m_node->get_logger(), "Both Thread Joined");
                break;   
            }
        } while (!doneWithPoi); 
        m_doneWithPoi = true; 
        RCLCPP_INFO_STREAM(m_node->get_logger(), "Done with PoI");
        // } while (!m_doneWithPoi);  
}

void NarrateComponent::Narrate(const std::shared_ptr<narrate_interfaces::srv::Narrate::Request> request,
             std::shared_ptr<narrate_interfaces::srv::Narrate::Response>      response) 
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "NarrateComponent::Narrate ");
    if (m_threadNarration.joinable()) {
        m_threadNarration.join();
    }
    m_stopped = false;
    m_threadNarration = std::thread([this, request]() { NarrateTask(request); });
    response->is_ok = true;
}

void NarrateComponent::Stop([[maybe_unused]] const std::shared_ptr<narrate_interfaces::srv::Stop::Request> request,
             std::shared_ptr<narrate_interfaces::srv::Stop::Response>      response) 
{
    RCLCPP_INFO_STREAM(m_node->get_logger(), "NarrateComponent::Stop ");
    m_stopped = true;
    if (m_threadNarration.joinable()) {
        m_threadNarration.join();
    }
    response->is_ok = true;
}
