#include "SoundConsumerThread.h"
#include <yard/os/LogStream.h>

YARP_LOG_COMPONENT(SOUND_CONSUMER_THREAD, "convince.narrate_component.SoundConsumerThread")

bool SoundConsumerThread::setPorts(yarp::os::BufferedPort<yarp::sig::Sound>* outSoundPort,
                                    yarp::os::Port* playerStatusPort)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (outSoundPort == nullptr || playerStatusPort == nullptr)
    {
        yCError(SOUND_CONSUMER_THREAD) << "setPorts: Null pointer provided";
        return false;
    }
    m_outSoundPort = outSoundPort;
    m_playerStatusPort = playerStatusPort;
    return true;
}

bool SoundConsumerThread::initialize(int counterIn)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_toSend = counterIn;
    m_soundQueue->flush();
    return true;
}

void SoundConsumerThread::setSoundQueue(SoundSafeQueue* soundQueue)
{
    m_soundQueue = soundQueue;
}

void SoundConsumerThread::_sendSound(const yarp::sig::Sound& sound)
{
    if (m_outSoundPort != nullptr)
    {
        yarp::os::Sound& toSend = m_outSoundPort->prepare();
        toSend = sound;
        m_outSoundPort->write();
        yCDebug(SOUND_CONSUMER_THREAD) << "Sent sound with" << sound.getSamples() << "samples";
    }
    else
    {
        yCError(SOUND_CONSUMER_THREAD) << "Output sound port is not set";
    }
}

void SoundConsumerThread::_waitForPlayerStatus(bool discriminator)
{
    bool keepOnWaiting;
    do{
        yarp::sig::AudioPlayerStatus* player_status = m_playerStatusInPort->read();
        if (player_status != nullptr && player_status->current_buffer_size > 0)
        {
            keepOnWaiting = !discriminator;
        }
        else
        {
            keepOnWaiting = discriminator;
        }
    } while (keepOnWaiting);
}

void SoundConsumerThread::run()
{
    while (!isStopping() && !m_soundQueue->isEmpty() && m_toSend > 0)
    {
        if(!m_soundQueue->isEmpty())
        {
            _waitForPlayerStatus(false);
            yarp::sig::Sound sound;
            if (m_soundQueue->pop(sound))
            {
                _sendSound(sound);
                m_toSend--;
            }
            else
            {
                yCError(SOUND_CONSUMER_THREAD) << "Failed to pop sound from the queue";
            }
            _waitForPlayerStatus(true);
        }
        else
        {
            yarp::os::Time::delay(0.1); // Sleep for a short while before checking again
            continue;
        }
    }
}