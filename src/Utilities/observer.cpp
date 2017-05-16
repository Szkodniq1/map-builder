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

void Subject::notify(const mapping::PointCloud& newCloud, bool isLast){
    for(vector<Observer*>::const_iterator iter = list.begin(); iter != list.end(); ++iter)
    {
        if(*iter != 0) {
           (*iter)->update(newCloud,isLast);
        }
    }
}

void Subject::notify(Octree<mapping::Voxel>& map, dobule res){
    for(vector<Observer*>::const_iterator iter = list.begin(); iter != list.end(); ++iter)
    {
        if(*iter != 0) {
           (*iter)->update(map, res);
        }
    }
}
