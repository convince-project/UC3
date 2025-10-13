/******************************************************************************
 *                                                                            *
 * Copyright (C) 2025 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

# pragma once

#include <mutex>
#include <thread>
#include <queue>
#include <stdlib.h>
#include <time.h>

#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/os/ResourceFinder.h>
#include <yard/os/BufferedPort.h>
#include <yard/sig/Sound.h>

#include "SafeQueue.h"


class SoundConsumerThread : public yarp::os::Thread
{
public:
    SoundConsumerThread();

    // yarp::os::Thread methods
    void run() override;

    // Inner methods
    bool setPorts(yarp::os::BufferedPort<yarp::sig::Sound>* outSoundPort,
                  yarp::os::Port* playerStatusPort);
    bool initialize(int counterIn); // flushes the queue and resets the counter
    void setSoundQueue(SoundSafeQueue* soundQueue);

private:
    std::mutex m_mutex;
    SoundSafeQueue* m_soundQueue;
    yarp::os::BufferedPort<yarp::sig::Sound>* m_outSoundPort; // Port to send the sound to the player
    yarp::os::Port* m_playerStatusInPort; // Port to receive the status from the player
    u_int m_toSend{0};

    void _sendSound(const yarp::sig::Sound& sound);

    // Wait until the player is not speaking if discriminator is false
    // Wait until the player is speaking if discriminator is true
    void _waitForPlayerStatus(bool discriminator);
};


