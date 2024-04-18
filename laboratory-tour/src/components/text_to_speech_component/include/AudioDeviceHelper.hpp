/******************************************************************************
 *                                                                            *
 * Copyright (C) 2023 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#ifndef AUDIO_DEVICE_HELPER__HPP
#define AUDIO_DEVICE_HELPER__HPP

#include <iostream>
#include <vector>
#include <mutex>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/dev/IAudioRender.h>
#include <yarp/dev/PolyDriver.h>

class AudioDeviceHelper
{
private:
    yarp::dev::PolyDriver m_audioDevicePoly;
    yarp::dev::IAudioRender *m_iAudioRender{nullptr};

public:
    AudioDeviceHelper() = default;
    ~AudioDeviceHelper() = default;

    bool ConfigureYARP(yarp::os::ResourceFinder &rf);

    bool isAudioPlaying(bool &isPlaying);
    bool playSound(yarp::sig::Sound &sound);
    bool stopPlaying();
};

#endif