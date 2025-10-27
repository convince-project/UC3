/******************************************************************************
 *                                                                            *
 * Copyright (C) 2025 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#pragma once

#include <mutex>
#include <queue>
#include <string>
#include <cstdlib>
#include <ctime>

#include <yarp/sig/Sound.h>

template <class T>
class SafeQueue
{
private:
    mutable std::mutex m_mutex;
    std::queue<T> m_queue;

public:
    SafeQueue() = default;
    ~SafeQueue() = default;

    void push(const T& item) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_queue.push(item);
    }

    void push(T&& item) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_queue.push(std::move(item));
    }

    bool pop(T& item) {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_queue.empty()) return false;
        item = std::move(m_queue.front());
        m_queue.pop();
        return true;
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_queue.size();
    }

    bool isEmpty() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_queue.empty();
    }

    bool flush() {
        std::lock_guard<std::mutex> lock(m_mutex);
        std::queue<T> empty;
        std::swap(m_queue, empty);
        return true;
    }
};

class SoundSafeQueue : public SafeQueue<yarp::sig::Sound> {};
class StringSafeQueue : public SafeQueue<std::string> {};