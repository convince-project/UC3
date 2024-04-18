/******************************************************************************
 *                                                                            *
 * Copyright (C) 2023 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "AudioDeviceHelper.hpp"

bool AudioDeviceHelper::ConfigureYARP(yarp::os::ResourceFinder &rf)
{
    bool okCheck = rf.check("AUDIODEVICEHELPER-CLIENT");
    std::string device = "AudioPlayerWrapper";
    int period = 10;
    std::string name = "/audioPlayerWrapper";

    if (okCheck)
    {
        yarp::os::Searchable &speech_config = rf.findGroup("AUDIODEVICEHELPER-CLIENT");
        if (speech_config.check("device"))
        {
            device = speech_config.find("device").asString();
        }
        if (speech_config.check("local-suffix"))
        {
            name = "/AudioDeviceHelper" + speech_config.find("local-suffix").asString();
        }
        if (speech_config.check("period"))
        {
            period = speech_config.find("remote").asInt64();
        }
    }

    yarp::os::Property prop;
    prop.put("device", device);
    prop.put("name", name);
    prop.put("period", period);

    m_audioDevicePoly.open(prop);
    if (!m_audioDevicePoly.isValid())
    {
        yError() << "[AudioDeviceHelper::ConfigureYARP] Error opening Audio Player Wrapper PolyDriver. Check parameters";
        return false;
    }
    m_audioDevicePoly.view(m_iAudioRender);
    if (!m_iAudioRender)
    {
        yError() << "[AudioDeviceHelper::ConfigureYARP] Error opening m_iAudioRender interface. Device not available";
        return false;
    }
    return true;
}

bool AudioDeviceHelper::isAudioPlaying(bool &isPlaying)
{
    if(! m_iAudioRender->isPlaying(isPlaying))
    {
        yError() << "[AudioDeviceHelper::isAudioPlaying] Unable to get if the audio is playing";
        return false;
    }
    return true;
}

bool AudioDeviceHelper::stopPlaying()
{
    bool audioPlaying = false;
    if(! m_iAudioRender->isPlaying(audioPlaying))
    {
        yError() << "[AudioDeviceHelper::stopPlaying] Unable to get if the audio is playing";
    }
    if (audioPlaying)
    {
        if (! m_iAudioRender->stopPlayback())
        {
            yError() << "[AudioDeviceHelper::stopPlaying] Unable to stop the playback";
            return false;
        }
        yInfo() << "[AudioDeviceHelper::stopPlaying] Stopping playback";
    }

    return true;
}

bool AudioDeviceHelper::playSound(yarp::sig::Sound &sound)
{
    // TODO - maybe do check if robot is already playing (maybe should be made at an higher level)
    if (! m_iAudioRender->renderSound(sound))
    {
        yError() << "[AudioDeviceHelper::stopPlaying] Unable to send the sound to the speakers";
        return false;
    }
    return true;
}