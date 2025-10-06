/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "yarp/sig/Sound.h"

#include "VerbalOutputBatchReader.hpp"

using namespace std::chrono_literals;

bool VerbalOutputBatchReader::ConfigureYARP(yarp::os::ResourceFinder &rf)
{
    // --------------------- TEXT TO SPEECH NWC----------------------------
    bool okCheck = rf.check("TEXT_TO_SPEECH_COMPONENT");

    m_audioInputPort.useCallback(*this);
    if (!m_audioInputPort.open("/VerbalOutputBatchReader/batch:i"))
    {
        yError() << "[VerbalOutputBatchReader::ConfigureYARP] Unable to open port: " << "/VerbalOutputBatchReader/batch:i";
        return false;
    }
    
    // yInfo() << "[VerbalOutputBatchReader::ConfigureYARP] Trying to connect to /TextToSpeechComponent/batch:o";
    // yarp::os::Network::connect("/TextToSpeechComponent/batch:o", m_audioInputPort.getName());

    yInfo() << "[VerbalOutputBatchReader::ConfigureYARP] Successfully configured component";

    return true;

}

void VerbalOutputBatchReader::resetQueue()
{
    m_audioQueue = {};
    yInfo() << "[VerbalOutputBatchReader::resetQueue] Audio queue has been reset.";
}


std::unique_ptr<yarp::sig::Sound> VerbalOutputBatchReader::GetVerbalOutput()
{
    std::cout << "[VerbalOutputBatchReader::GetVerbalOutput] Checking audio queue. Current size is: " << m_audioQueue.size() << std::endl;
    if (!m_audioQueue.empty())
    {
        std::cout << "[VerbalOutputBatchReader::GetVerbalOutput] Assigning audio from queue. Queue size is: " << m_audioQueue.size() << std::endl;
        
        auto sound = std::move(m_audioQueue.front());
        std::cout << "[VerbalOutputBatchReader::GetVerbalOutput] Audio queue size before popping: " << m_audioQueue.size() << std::endl;
        m_audioQueue.pop();
        std::cout << "[VerbalOutputBatchReader::GetVerbalOutput] Returning audio from queue. Queue size is now: " << m_audioQueue.size() << std::endl;
        return sound;
    }
    return nullptr;
}


void VerbalOutputBatchReader::onRead(yarp::sig::Sound &msg)
{
    m_audioQueue.push(std::make_unique<yarp::sig::Sound>(msg));
    yInfo() << "[VerbalOutputBatchReader::onRead] Received audio message. Queue size is now: " << m_audioQueue.size();
}
