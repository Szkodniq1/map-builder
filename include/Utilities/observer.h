
#ifndef OBSERVER_H_
#define OBSERVER_H_

#include "../include/Defs/defs.h"
#include <vector>
#include <list>

class Observer
{
public:
    virtual void update(const Mat34& newPose) = 0;
};

class Subject
{
    std::vector<Observer*> list;

public:
    void attach(Observer *observer);
    void detach(Observer *observer);
    void notify(const Mat34& newPose);
};

#endif // OBSERVER_H_
