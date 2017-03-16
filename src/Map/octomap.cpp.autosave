#include "../include/Map/octomap.h"
#include <memory>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <iostream>

/// A single instance of Octomap
OctoMap::Ptr octoMap;

OctoMap::OctoMap(void) : map(this->MAP_RES) {
    std::cout << "OctoMap created\n";
    for (int i=0;i<20;i++){
        for (int j=0;j<20;j++){
            mapping::Point3D point(i*0.1, j*0.1,0.0);
            cloud.push_back(point);
        }
    }
}

OctoMap::OctoMap(mapping::PointCloud PC) : map(this->MAP_RES) {
    cloud = PC;
}

/// Insert point cloud into map
void OctoMap::insertCloud(octomap::Pointcloud& pcl){
    notify(cloud);
}

void OctoMap::insertCloud(mapping::PointCloud& PC, octomap::pose6d pose){

    Eigen::Translation<float, 3> translation (pose.trans().x(), pose.trans().y(), pose.trans().z());
    // Assemble an Eigen Transform
    Eigen::Quaternion<float> pos(pose.rot().u(),pose.rot().x(), pose.rot().y(), pose.rot().z());
    Eigen::Transform<float, 3, Eigen::Affine> transform (translation * pos);
    octoCloud.reserve(PC.size());
    for(mapping::Point3D point : PC) {
        Eigen::Matrix<float, 3, 1> pt (point.position.x(), point.position.y(), point.position.z());
        octoCloud.push_back(
                    static_cast<float> (transform (0, 0) * pt.coeffRef (0) + transform (0, 1) * pt.coeffRef (1) + transform (0, 2) * pt.coeffRef (2) + transform (0, 3)),
                    static_cast<float> (transform (1, 0) * pt.coeffRef (0) + transform (1, 1) * pt.coeffRef (1) + transform (1, 2) * pt.coeffRef (2) + transform (1, 3)),
                    static_cast<float> (transform (2, 0) * pt.coeffRef (0) + transform (2, 1) * pt.coeffRef (1) + transform (2, 2) * pt.coeffRef (2) + transform (2, 3)));
    }
    this->map.insertPointCloud(octoCloud, pose.trans());
    this->map.updateInnerOccupancy();
    notify(PC);
}

/// save map in file
void OctoMap::saveMap(){
    std::stringstream stream;
    stream <<"tree10.bt";
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
