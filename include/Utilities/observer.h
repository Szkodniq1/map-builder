
#ifndef OBSERVER_H_
#define OBSERVER_H_

#include "../include/Defs/defs.h"
#include "../include/Defs/voxel.h"
#include <vector>
#include <list>
#include "3rdParty/octree/octree.h"


class Observer
{
public:
    virtual void update(const mapping::PointCloud& newCloud, bool isLast) = 0;
    virtual void update(Octree<mapping::Voxel>& map, double res) = 0;
};

class Subject
{
    std::vector<Observer*> list;

public:
    void attach(Observer *observer);
    void detach(Observer *observer);
    void notify(const mapping::PointCloud& newCloud, bool isLast);
    void notify(Octree<mapping::Voxel>& map, double res);
};

#endif // OBSERVER_H_
