#include "../include/Map/octomap.h"
#include <memory>
#include <stdexcept>
#include <chrono>
#include <thread>

/// A single instance of Octomap
OctoMap::Ptr octoMap;

OctoMap::OctoMap(void){
    std::cout << "OctoMap created\n";
    for (int i=0;i<20;i++){
        for (int j=0;j<20;j++){
            mapping::Point3D point(i*0.1, j*0.1,0.0);
            cloud.push_back(point);
        }
    }
}

/// Insert point cloud into map
void OctoMap::insertCloud(octomap::Pointcloud& pcl){
    notify(cloud);
}

/// save map in file
void OctoMap::saveMap(){

}

///Attach visualizer
void OctoMap::attachVisualizer(QGLVisualizer* visualizer){
    attach(visualizer);
}

/// print map
void OctoMap::printMap(){

}

mapping::Map* mapping::createMapOcto(void) {
    octoMap.reset(new OctoMap());
    return octoMap.get();
}
