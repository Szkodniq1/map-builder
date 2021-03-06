//CPP File
#include "../include/Utilities/observer.h"
#include <algorithm>
#include <iostream>

using namespace std;

void Subject::attach(Observer *observer){
    list.push_back(observer);
}
void Subject::detach(Observer *observer){
    list.erase(std::remove(list.begin(), list.end(), observer), list.end());
}

void Subject::notify(const mapping::PointCloud& newCloud,std::vector<mapping::Mat33> uncertinatyErrors,  bool isLast){
    for(vector<Observer*>::const_iterator iter = list.begin(); iter != list.end(); ++iter)
    {
        if(*iter != 0) {
           (*iter)->update(newCloud,uncertinatyErrors,isLast);
        }
    }
}

void Subject::notify(Octree<mapping::Voxel>& map, double res, std::unordered_map<std::string, Eigen::Vector3i> indexes , bool isLast){
    for(vector<Observer*>::const_iterator iter = list.begin(); iter != list.end(); ++iter)
    {
        if(*iter != 0) {
           (*iter)->update(map, res, indexes, isLast);
        }
    }
}

void Subject::notify(mapping::Quaternion orientation, mapping::Vec3 translation){
    for(vector<Observer*>::const_iterator iter = list.begin(); iter != list.end(); ++iter)
    {
        if(*iter != 0) {
           (*iter)->update(orientation, translation);
        }
    }
}
