#include "../include/Map/octomap.h"
#include <memory>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <iostream>

/// A single instance of Octomap
OctoMap::Ptr octoMap;

OctoMap::OctoMap(void) : map(0.1) {
    std::cout << "OctoMap created\n";
    for (int i=0;i<20;i++){
        for (int j=0;j<20;j++){
            mapping::Point3D point(i*0.1, j*0.1,0.0);
            cloud.push_back(point);
        }
    }
}

OctoMap::OctoMap(mapping::PointCloud PC) : map(0.1) {
    cloud = PC;
}

/// Insert point cloud into map
void OctoMap::insertCloud(octomap::Pointcloud& pcl){
    notify(cloud);
}

void OctoMap::insertCloud(mapping::PointCloud& PC){
    for(mapping::Point3D point : PC) {
        octoCloud.push_back(point.position.x(), point.position.y(), point.position.z());
    }
    octomap::point3d origin (0,0,0);
    this->map.insertPointCloud(octoCloud, origin);
    notify(PC);
}

/// save map in file
void OctoMap::saveMap(){
    std::stringstream stream;
    stream <<"tree.bt";
    this->map.writeBinary(stream.str());
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

mapping::Map* mapping::createMapOcto(PointCloud PC) {
    octoMap.reset(new OctoMap(PC));
    return octoMap.get();
}
