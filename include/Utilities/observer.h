
#ifndef OBSERVER_H_
#define OBSERVER_H_

#include "../include/Defs/defs.h"
#include "../include/Voxel/voxel.h"
#include <vector>
#include <list>
#include "3rdParty/octree/octree.h"
#include <unordered_map>


class Observer
{
public:
    virtual void update(const mapping::PointCloud& newCloud,std::vector<mapping::Mat33> uncertinatyErrors, bool isLast) = 0;
    virtual void update(Octree<mapping::Voxel>& map, double res, std::unordered_map<std::string, Eigen::Vector3i> indexes , bool isLast) = 0;
};

class Subject
{
    std::vector<Observer*> list;

public:
    void attach(Observer *observer);
    void detach(Observer *observer);
    void notify(const mapping::PointCloud& newCloud,std::vector<mapping::Mat33> uncertinatyErrors, bool isLast);
    void notify(Octree<mapping::Voxel>& map, double res, std::unordered_map<std::string, Eigen::Vector3i> indexes , bool isLast);
};

#endif // OBSERVER_H_
