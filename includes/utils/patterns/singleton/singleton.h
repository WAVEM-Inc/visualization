//
// Created by antique on 24. 2. 20.
//

#ifndef NODE_EDITOR_SINGLETON_H
#define NODE_EDITOR_SINGLETON_H

#include <iostream>
#include <mutex>
#include <memory>

template<class T>
class Singleton {
public:
    Singleton(const Singleton &) = delete;

    Singleton &operator=(const Singleton &) = delete;

    static T &Instance() {
        std::lock_guard<std::mutex> lock(mutex);

        if (!m_instance) {
            m_instance.reset(new T());
        }

        return *m_instance;
    }

protected:
    Singleton() {};

private:
    static std::unique_ptr<T> m_instance;
    static std::mutex mutex;
};

template <typename T>
std::unique_ptr<T> Singleton<T>::m_instance = nullptr;

template <typename T>
std::mutex Singleton<T>::mutex;


#endif //NODE_EDITOR_SINGLETON_H
