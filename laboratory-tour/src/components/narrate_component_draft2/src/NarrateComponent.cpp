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


void NarrateComponent::speakTask() {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "NEW_SPEAK_THREAD");
    m_speakMutex.lock();
    int size = m_speakBuffer.size();
    m_speakMutex.unlock();
    do{
        m_speakMutex.lock();
        std::string text = m_speakBuffer.front();
        m_speakBuffer.pop();
        m_speakMutex.unlock();
        std::cout << "Speak Action" << std::endl;
        auto speakClientNode = rclcpp::Node::make_shared("NarrateComponentSpeakNode");
        std::shared_ptr<rclcpp::Client<text_to_speech_interfaces::srv::Speak>> speakClient = 
        speakClientNode->create_client<text_to_speech_interfaces::srv::Speak>("/TextToSpeechComponent/Speak");
        auto speakRequest = std::make_shared<text_to_speech_interfaces::srv::Speak::Request>();
        speakRequest->text = text;
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
            while (!isSpeakingClient->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'isSpeakingClient'. Exiting.");
                }
            }
            auto isSpeakingResult = isSpeakingClient->async_send_request(isSpeakingRequest);
            auto futureIsSpeakingResult = rclcpp::spin_until_future_complete(isSpeakingClientNode, isSpeakingResult);
            auto isSpeakingResponse = isSpeakingResult.get();
            isSpeaking = isSpeakingResponse->is_speaking;
            m_timeMutex.lock();
            m_seconds_left = isSpeakingResponse->seconds_left;
            m_timeMutex.unlock();
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "is_speaking " << isSpeakingResponse->is_speaking); 
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "seconds_left " << isSpeakingResponse->seconds_left); 
        } while (isSpeaking);
        std::cout << "Value stop "<< m_stopped << std::endl;
        if(m_stopped)
        {
            std::cout << "stop recieved speak" << std::endl;
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
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "NEW_DANCE_THREAD");
    m_danceMutex.lock();
    int size = m_danceBuffer.size();
    m_danceMutex.unlock();
    int l_seconds_left;
    do{
        
        std::cout << "qui do" << std::endl;
        std::string text;
        m_timeMutex.lock();
        l_seconds_left = m_seconds_left;
        m_timeMutex.unlock();
        if(l_seconds_left < 3){
            std::cout << "qui if" << std::endl;
            // m_danceMutex.lock();
            // if(m_danceBuffer.size() != 0)
            // {
            //     m_danceBuffer.pop();
            // }
            // m_danceMutex.unlock();
            text = "idleMove";
            std::cout << "qui if out" << std::endl;
        }
        else{
            std::cout << "qui else" << std::endl;
            m_danceMutex.lock();
            if(m_danceBuffer.size() != 0){
                text = m_danceBuffer.front();
            }
            m_danceMutex.unlock();
            std::cout << "qui else out" << std::endl;
        }
        // m_danceMutex.lock();
        // text = m_danceBuffer.front();
        // m_danceBuffer.pop();
        // m_danceMutex.unlock();
        std::cout << "Dance Action " << text << std::endl;
        auto danceClientNode = rclcpp::Node::make_shared("NarrateComponentDanceNode");
        std::shared_ptr<rclcpp::Client<execute_dance_interfaces::srv::ExecuteDance>> danceClient = 
        danceClientNode->create_client<execute_dance_interfaces::srv::ExecuteDance>("/ExecuteDanceComponent/ExecuteDance");
        auto danceRequest = std::make_shared<execute_dance_interfaces::srv::ExecuteDance::Request>();
        danceRequest->dance_name = text;
        while (!danceClient->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'danceClient'. Exiting.");
            }
        }
        auto danceResult = danceClient->async_send_request(danceRequest);
        auto futureDanceResult = rclcpp::spin_until_future_complete(danceClientNode, danceResult);
        auto danceFutureResult = danceResult.get();
        if (danceFutureResult->is_ok == true) { // 0 is dance
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        // std::this_thread::sleep_for(std::chrono::seconds(30));
        // waits until the robot has finished dancing
        bool isDancing = false;
        do {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            // calls the isDancing service
            auto isDancingClientNode = rclcpp::Node::make_shared("NarrateComponentIsDancingNode");
            std::shared_ptr<rclcpp::Client<execute_dance_interfaces::srv::IsDancing>> isDancingClient =
            isDancingClientNode->create_client<execute_dance_interfaces::srv::IsDancing>("/ExecuteDanceComponent/IsDancing");
            auto isDancingRequest = std::make_shared<execute_dance_interfaces::srv::IsDancing::Request>();
            while (!isDancingClient->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service 'isDancingClient'. Exiting.");
                }
            }
            auto isDancingResult = isDancingClient->async_send_request(isDancingRequest);
            auto futureIsDancingResult = rclcpp::spin_until_future_complete(isDancingClientNode, isDancingResult);
            auto isDancingResponse = isDancingResult.get();
            isDancing = isDancingResponse->is_dancing;
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "is_dancing " << isDancingResponse->is_dancing); 
        } while (isDancing);
        if(m_stopped)
        {
            std::cout << "stop recieved" << std::endl;
            break;   
        }
        
        m_danceMutex.lock();
        size = m_danceBuffer.size();
        m_danceMutex.unlock();
        m_timeMutex.lock();
        l_seconds_left = m_seconds_left;
        m_timeMutex.unlock();
        std::cout << "size " << size << std::endl; 
        std::cout << "l_second_left " << l_seconds_left << std::endl; 
    }
    while(l_seconds_left > 0 || size > 0);
    // while(size);
    std::cout << "qui" << std::endl;
    m_danceTask = false;
}
        
void NarrateComponent::NarrateTask(const std::shared_ptr<narrate_interfaces::srv::Narrate::Request> request) {
        // calls the SetCommand service
	    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "NEW_NARRATE_THREAD");
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
                        std::cout << "Speak Task started." << std::endl;
                    } else {
                        std::cout << "Speak Task is already running." << std::endl;
                    }
                }
                else if (currentAction->type == "dance")
                {
                    std::cout << "dance" << std::endl;
                    m_danceMutex.lock();
                    m_danceBuffer.push(currentAction->param);
                    m_danceMutex.unlock();
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    if (!m_danceTask) {
                        m_danceTask = true;
                        if (danceThread.joinable()) {
                            danceThread.join();
                        }
                        danceThread = std::thread(&NarrateComponent::danceTask, this);
                        std::cout << "Dance Task started." << std::endl;
                    } else {
                        std::cout << "Dance Task is already running." << std::endl;
                    }
                }
                if(currentAction->is_blocking)
                {
                    std::cout << "is blocking"<< std::endl;
                    if (speakThread.joinable()) {
                        speakThread.join();
                        m_danceMutex.lock();
                        while(m_danceBuffer.size() != 0)
                        {
                            m_danceBuffer.pop();
                        }
                        m_danceMutex.unlock();
                        std::cout << "speak joined"<< std::endl;
                    }
                    if (danceThread.joinable() ) {
                        danceThread.join();
                        std::cout << "dance joined"<< std::endl;
                    }
                    std::cout << "both joined"<< std::endl;
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
            std::cout << " Done with Poi " << doneWithPoi << std::endl;
            if(m_stopped)
            {
                std::cout << "stop recieved" << std::endl;
                if (speakThread.joinable()) {
                    speakThread.join();
                    std::cout << "speak joined"<< std::endl;
                }
                if (danceThread.joinable() ) {
                    danceThread.join();
                    std::cout << "dance joined"<< std::endl;
                }
                std::cout << "both joined"<< std::endl;
                break;   
            }
        } while (!doneWithPoi); 
        m_doneWithPoi = true; 
        // } while (!m_doneWithPoi);  
}

void NarrateComponent::Narrate(const std::shared_ptr<narrate_interfaces::srv::Narrate::Request> request,
             std::shared_ptr<narrate_interfaces::srv::Narrate::Response>      response) 
{
    if (m_threadNarration.joinable()) {
        m_threadNarration.join();
    }
    m_stopped = false;
    // std::thread mainThread([this, request]() { NarrateTask(request); });
    m_threadNarration = std::thread([this, request]() { NarrateTask(request); });
    // mainThread.join();
    // m_threadNarration([this, request]() { NarrateTask(request); });
    // std::thread mainThread(&NarrateComponent::NarrateTask, request);
    // m_threadNarration.join();
    // std::thread threadLocal([this, request](){
    //     m_doneWithPoi = false;
    //         // calls the UpdateAction service
    //         m_doneWithPoi = updateActionResponse->done_with_poi;
	//     RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "donee_with_poi" << updateActionResponse->done_with_poi);
    //     } while (!m_doneWithPoi);  
    //     std::cout << " Done with Poi " << std::endl;
    // });
    // m_threadNarration = std::move(threadLocal);
    response->is_ok = true;
}

void NarrateComponent::Stop([[maybe_unused]] const std::shared_ptr<narrate_interfaces::srv::Stop::Request> request,
             std::shared_ptr<narrate_interfaces::srv::Stop::Response>      response) 
{
    std::cout << "STOP" << std::endl;
    m_stopped = true;
    std::cout << "STOP" << std::endl;
    if (m_threadNarration.joinable()) {
        m_threadNarration.join();
    }
    response->is_ok = true;
}