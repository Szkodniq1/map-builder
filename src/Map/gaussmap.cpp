#include "../include/Map/gaussmap.h"

/// A single instance of Octomap
Gaussmap::Ptr gaussMap;

Gaussmap::Gaussmap(void) : map(256) {

}

Gaussmap::Gaussmap(mapping::PointCloud PC) : map(256) {

}

/// Insert point cloud into map
void Gaussmap::insertCloud(octomap::Pointcloud& pcl){

}

void Gaussmap::insertCloud(mapping::PointCloud& PC, octomap::pose6d pose){

}

/// save map in file
void Gaussmap::saveMap(){

}

///Attach visualizer
void Gaussmap::attachVisualizer(QGLVisualizer* visualizer){
    attach(visualizer);
}

/// print map
void Gaussmap::printMap(){

}

mapping::Map* mapping::createMapGauss(void) {
    gaussMap.reset(new Gaussmap());
    return gaussMap.get();
}

mapping::Map* mapping::createMapGauss(PointCloud PC) {
    gaussMap.reset(new Gaussmap(PC));
    return gaussMap.get();
}
