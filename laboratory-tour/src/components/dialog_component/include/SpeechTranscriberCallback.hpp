#ifndef SPEECH_TRANSCRIBER_CALLBACK__HPP
#define SPEECH_TRANSCRIBER_CALLBACK__HPP

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>

// Is a mutex needed?
#include <mutex>

class SpeechTranscriberCallback : public yarp::os::BufferedPort<yarp::os::Bottle>
{
public:
    using yarp::os::BufferedPort<yarp::os::Bottle>::onRead;
    void onRead(yarp::os::Bottle &msg) override
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_listenedText = msg.get(0).asString();
        m_hasNewMessage = true;
    };

    bool getText(std::string &text)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_listenedText == "")
            return false;
        
        text = m_listenedText;
        m_hasNewMessage = false;
        return true;
    };

    bool hasNewMessage()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_hasNewMessage;
    };
private:
    std::mutex m_mutex;
    std::string m_listenedText = "";
    bool m_hasNewMessage = false;
};

#endif