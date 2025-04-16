#ifndef SPEAKER_STATUS_CALLBACK__HPP
#define SPEAKER_STATUS_CALLBACK__HPP

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/AudioPlayerStatus.h>

// Is a mutex needed?
#include <mutex>

class SpeakerStatusCallback : public yarp::os::BufferedPort<yarp::sig::AudioPlayerStatus>
{
public:
    using yarp::os::BufferedPort<yarp::sig::AudioPlayerStatus>::onRead;
    void onRead(yarp::sig::AudioPlayerStatus &msg) override
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (msg.current_buffer_size > 0)
        {
            m_isPlaying = true;
        }
        else
        {
            m_isPlaying = false;
        }
        m_bufferSize = msg.current_buffer_size;
        m_isEnabled = msg.enabled;
    };

    // Returns whether the speaker status port has something on its audio buffer
    bool isPlaying()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_isPlaying;
    };

    // Returns if the device is enabled
    bool isEnabled()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_isEnabled;
    };

    // Returns the current size of the audio buffer for the pending sound on execution
    size_t get_bufferSize()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_bufferSize;
    }
private:
    std::mutex m_mutex;
    bool m_isPlaying = false;
    bool m_isEnabled = false;
    size_t m_bufferSize = 0;
};

#endif