#include "../include/Map/octomap.h"
#include <memory>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <iostream>
#include <opencv2/core/core.hpp>

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


    mapping::PointCloud newPC;
    newPC.reserve(PC.size());
    std::cout<<"Vector before normalisation :"<<pos.norm()<<std::endl;
    pos.normalize();
    std::cout<<"Vector normalised :"<<pos.norm()<<std::endl;
    int64 e1 = cv::getTickCount();
    for(mapping::Point3D point : PC) {
        /// Performance +/- 0,6s left as it more readable solution
        Eigen::Matrix<float, 3, 1> pt (point.position.x(), point.position.y(), point.position.z());
                newPC.push_back(mapping::Point3D(
                            static_cast<float> (transform (0, 0) * pt.coeffRef (0) + transform (0, 1) * pt.coeffRef (1) + transform (0, 2) * pt.coeffRef (2) + transform (0, 3)),
                            static_cast<float> (transform (1, 0) * pt.coeffRef (0) + transform (1, 1) * pt.coeffRef (1) + transform (1, 2) * pt.coeffRef (2) + transform (1, 3)),
                            static_cast<float> (transform (2, 0) * pt.coeffRef (0) + transform (2, 1) * pt.coeffRef (1) + transform (2, 2) * pt.coeffRef (2) + transform (2, 3)),point.color.r,
                            point.color.g,
                            point.color.b,
                            point.color.a));

        /// Performance +/- 1,5s
//        Eigen::Vector3f pt (point.position.x(), point.position.y(), point.position.z());
//        Eigen::Quaternion<float> p;
//        p.w() = 0;
//        p.vec() = pt;
//        Eigen::Quaternion<float> rotatedP = pos * p * pos.inverse();
//        Eigen::Vector3f rotatedPt = rotatedP.vec();
//        rotatedPt[0] += translation.x();
//        rotatedPt[1] += translation.y();
//        rotatedPt[2] += translation.z();

//        newPC.push_back(mapping::Point3D(
//                            rotatedPt[0],
//                            rotatedPt[1],
//                            rotatedPt[2],
//                            point.color.r,
//                            point.color.g,
//                            point.color.b,
//                            point.color.a));

        /// Performance +/- 0,6s
//        Eigen::Vector3f pt (point.position.x(), point.position.y(), point.position.z());
//        newPC.push_back(mapping::Point3D(
//                            (1-2*pos.y()*pos.y()-2*pos.z()*pos.z())*point.position.x() + 2*(pos.x()*pos.y() - pos.w()*pos.z())*point.position.y() + 2*(pos.x()*pos.z() + pos.w()*pos.y())*point.position.z() + translation.x(),
//                            2*(pos.x()*pos.y() + pos.w()*pos.z())*point.position.x() + (1-2*pos.x()*pos.x()-2*pos.z()*pos.z())*point.position.y() + 2*(pos.y()*pos.z() + pos.w()*pos.x())*point.position.z() + translation.y(),
//                            2*(pos.x()*pos.z() - pos.w()*pos.y())*point.position.x() + 2*(pos.y()*pos.z() - pos.w()*pos.x())*point.position.y() + (1-2*pos.x()*pos.x()-2*pos.y()*pos.y())*point.position.z() + translation.z(),
//                            point.color.r,
//                            point.color.g,
//                            point.color.b,
//                            point.color.a));
    }
    int64 e2 = cv::getTickCount();
    double time = ((e2 - e1)/ cv::getTickFrequency());
    std::cout<<"Calc time: "<<time<<std::endl;
    octoCloud.reserve(newPC.size());
    for(mapping::Point3D point : newPC) {
        octoCloud.push_back(
                    point.position.x(),
                    point.position.y(),
                    point.position.z());
    }
    e1 = cv::getTickCount();
    //First cloud 2,7s, second cloud 5,2s
    this->map.insertPointCloud(octoCloud, pose.trans());
    this->map.updateInnerOccupancy();
    e2 = cv::getTickCount();
    time = ((e2 - e1)/ cv::getTickFrequency());
    std::cout<<"Octomap insert time and update: "<<time<<std::endl;

    notify(newPC);
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
