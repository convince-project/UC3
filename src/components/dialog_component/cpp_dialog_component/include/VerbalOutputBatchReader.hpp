/******************************************************************************
 *                                                                            *
 * Copyright (C) 2023 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
#ifndef VERBAL_OUTPUT_BATCH_READER__HPP
#define VERBAL_OUTPUT_BATCH_READER__HPP

#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ISpeechTranscription.h>
#include <yarp/sig/AudioPlayerStatus.h>
#include <yarp/dev/IAudioGrabberSound.h>
#include <text_to_speech_interfaces/action/batch_generation.hpp>

#include <queue>

class VerbalOutputBatchReader : public yarp::os::TypedReaderCallback<yarp::sig::Sound>
{
public:
    VerbalOutputBatchReader() = default;

    // bool start(int argc, char*argv[]);
    // bool close();
    // void spin();
    bool ConfigureYARP(yarp::os::ResourceFinder &rf);

    std::unique_ptr<yarp::sig::Sound> GetVerbalOutput();

    void resetQueue();

    void onRead(yarp::sig::Sound &msg) override;

private:


    std::queue<std::unique_ptr<yarp::sig::Sound>> m_audioQueue;

    yarp::os::BufferedPort<yarp::sig::Sound> m_audioInputPort;
};

#endif // VERBAL_OUTPUT_BATCH_READER__HPP
