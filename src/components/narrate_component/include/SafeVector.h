/******************************************************************************
 *                                                                            *
 * Copyright (C) 2025 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#pragma once

#include <mutex>
#include <vector>
#include <string>
#include <cstdlib>
#include <ctime>


template <class T>
class SafeVector
{
private:
    mutable std::mutex m_mutex;
    std::vector<T> m_vector;

public:
    SafeVector() = default;
    ~SafeVector() = default;

    void push_back(const T& item) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_vector.push_back(item);
    }

    void push_back(T&& item) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_vector.push_back(std::move(item));
    }

    bool pop_back(T& item) {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_vector.empty()) return false;
        item = std::move(m_vector.back());
        m_vector.pop_back();
        return true;
    }

    bool at(size_t index, T& item) const {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (index >= m_vector.size()) return false;
        item = m_vector[index];
        return true;
    }

    bool set(size_t index, const T& item) {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (index >= m_vector.size()) return false;
        m_vector[index] = item;
        return true;
    }

    bool set(size_t index, T&& item) {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (index >= m_vector.size()) return false;
        m_vector[index] = std::move(item);
        return true;
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_vector.size();
    }

    bool isEmpty() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_vector.empty();
    }

    bool clear() {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_vector.clear();
        return true;
    }

    void reserve(size_t capacity) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_vector.reserve(capacity);
    }

    std::vector<T> copy() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_vector;
    }

    T operator[](size_t index) const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_vector[index];
    }
};

class StringSafeVector : public SafeVector<std::string> {};
