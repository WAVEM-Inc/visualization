//
// Created by antique on 24. 2. 20.
//

#ifndef NODE_EDITOR_SUBJECT_H
#define NODE_EDITOR_SUBJECT_H

#include <iostream>
#include <vector>
#include <list>
#include "observer.h"
#include "singleton.h"

template <class T>
class Subject {
public:
    using ObserverPtr = std::shared_ptr<Observer<T>>;
    using ObserverList = std::list<ObserverPtr>;

    Subject<T>() :m_value{} {

    };

    virtual ~Subject() {

    };

    void attach(ObserverPtr observer) {
        observers.push_back(observer);
    };

    void detach(ObserverPtr observer) {
        observers.remove_if([&observer](const ObserverPtr& o) {
            return o == observer;
        });
    };

    void notify(T data) {
        m_value = data;

        for (ObserverPtr observer : observers) {
            observer->update(data);
        }
    };

    T value() const {
        return m_value;
    }

private:
    ObserverList observers;
    T m_value;
};

#endif //NODE_EDITOR_SUBJECT_H
