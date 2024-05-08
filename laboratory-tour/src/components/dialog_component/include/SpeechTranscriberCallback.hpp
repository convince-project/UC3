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
	yInfo() << "onRead" << __LINE__;
        if(m_hasNewMessage)
        {
            yWarning() << "New message received before the previous one was consumed";
            return;
        }
	if (msg.get(0).asString() == "") 
		return;
        m_listenedText = msg.get(0).asString();
        yWarning() << "Got message: " << m_listenedText;
        m_hasNewMessage = true;
    };

    bool getText(std::string &text)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_listenedText == "")
            return false;

        text = m_listenedText;
        // m_listenedText = "";
        //m_hasNewMessage = false;
        return true;
    };

    bool setMessageConsumed()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_listenedText == "")
        {
            yWarning() << "No message to consume";
            return false;
        }
        m_hasNewMessage = false;
        m_listenedText = "";
        yWarning() << "Message consumed";
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
