//
// Created by antique on 24. 2. 20.
//

#ifndef NODE_EDITOR_OBSERVER_H
#define NODE_EDITOR_OBSERVER_H

template <class T>
class Observer {
public:
    explicit Observer<T>() {};
    virtual ~Observer() {};
    virtual void update(T data) = 0;
};


#endif //NODE_EDITOR_OBSERVER_H
