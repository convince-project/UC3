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
    

    textToSpeechClientPortName = "/VerbalOutputBatchReader/TextToSpeechComponent/batch:i";
    bool b = m_textToSpeechClientPort.open(textToSpeechClientPortName);
    if (!b)
    {
        yError() << "Cannot open textToSpeech client port";
        return false;
    }

    yInfo() << "[VerbalOutputBatchReader::ConfigureYARP] Trying to connect to /TextToSpeechComponent/batch:o";
    yarp::os::Network::connect(textToSpeechClientPortName, "/TextToSpeechComponent/batch:o");
    
}

void VerbalOutputBatchReader::resetQueue()
{
    std::queue<yarp::sig::Sound> empty;
    std::swap(m_audioQueue, empty);
    yInfo() << "[VerbalOutputBatchReader::resetQueue] Audio queue has been reset.";
}

void VerbalOutputBatchReader::GetVerbalOutput(yarp::sig::Sound *output)
{
    if (!m_audioQueue.empty())
    {
        *output = m_audioQueue.front();
        m_audioQueue.pop();
        yInfo() << "[VerbalOutputBatchReader::GetVerbalOutput] Returning audio from queue. Queue size is now: " << m_audioQueue.size();
        return;
    }
}

void VerbalOutputBatchReader::onRead(yarp::sig::Sound &msg)
{
    m_audioQueue.push(msg);
    yInfo() << "[VerbalOutputBatchReader::onRead] Received audio message. Queue size is now: " << m_audioQueue.size();
}
