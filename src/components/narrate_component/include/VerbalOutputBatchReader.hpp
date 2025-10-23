/******************************************************************************
 *                                                                            *
 * Copyright (C) 2023 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
#pragma once

#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
// #include <yarp/os/LogComponent.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ISpeechTranscription.h>
#include <yarp/sig/AudioPlayerStatus.h>
#include <yarp/sig/Sound.h>
#include <yarp/dev/IAudioGrabberSound.h>
#include <text_to_speech_interfaces/action/batch_generation.hpp>

#include "SafeQueue.h"

#include <queue>

class VerbalOutputBatchReader : public yarp::os::TypedReaderCallback<yarp::sig::Sound>
{
public:
    VerbalOutputBatchReader() = default;

    void onRead(yarp::sig::Sound &msg) override;
    void setSoundQueue(SoundSafeQueue* soundQueue) { m_audioQueue = soundQueue; }

private:
    SoundSafeQueue* m_audioQueue;
};
