#include "../include/Map/octomap.h"
#include <memory>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <iostream>
#include <opencv2/core/core.hpp>

/// A single instance of Octomap
OctoMap::Ptr octoMap;

OctoMap::OctoMap(void) : map(res) {

}

OctoMap::OctoMap(mapping::PointCloud PC) : map(res) {

}

void OctoMap::mapLoaded() {

}

/// Insert point cloud into map
void OctoMap::insertCloud(mapping::GrabbedImage grab, bool isLast) {
    mapping::PointCloud newPC = grab.transformedPointCloud();
    octoCloud.reserve(newPC.size());
    for(mapping::Point3D point : newPC) {
        octoCloud.push_back(
                    point.position.x(),
                    point.position.y(),
                    point.position.z());
    }
    int64 e1 = cv::getTickCount();
    //First cloud 2,7s, second cloud 5,2s
    this->map.insertPointCloud(octoCloud, octomap::point3d(grab.translation.x(), grab.translation.y(), grab.translation.z()));
    this->map.updateInnerOccupancy();
    int64 e2 = cv::getTickCount();
    double time = ((e2 - e1)/ cv::getTickFrequency());
    std::cout<<"Octomap insert time and update: "<<time<<std::endl;

    notify(newPC, grab.uncertinatyErrors, isLast);
}

/// save map in file
void OctoMap::saveMap(){
    std::stringstream stream;
    stream <<currentDateTime()<<".bt";
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

std::string OctoMap::currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d-%X", &tstruct);

    return buf;
}
