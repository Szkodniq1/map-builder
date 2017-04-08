
#ifndef OBSERVER_H_
#define OBSERVER_H_

#include "../include/Defs/defs.h"
#include <vector>
#include <list>

class Observer
{
public:
    virtual void update(const mapping::PointCloud& newCloud, bool isLast) = 0;
};

class Subject
{
    std::vector<Observer*> list;

public:
    void attach(Observer *observer);
    void detach(Observer *observer);
    void notify(const mapping::PointCloud& newCloud, bool isLast);
};

#endif // OBSERVER_H_
