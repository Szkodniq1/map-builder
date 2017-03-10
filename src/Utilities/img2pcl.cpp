//CPP File
#include "../include/Utilities/img2pcl.h"

namespace mapping {

/// Convert disparity image to point cloud
PointCloud depth2cloud(cv::Mat& depthImage) {
    Eigen::Vector3d point;
    PointCloud cloud;
    cloud.clear();
    for (unsigned int i=0;i<depthImage.rows;i++) {
        for (unsigned int j=0;j<depthImage.cols;j++) {
            if(depthImage.at<uint16_t>(i,j)>800&&depthImage.at<uint16_t>(i,j)<8500){
                float_type depthM = float_type(depthImage.at<uint16_t>(i,j))*0.001;
                getPoint(j,i,depthM,point);
                Point3D pointPCL;
                pointPCL.position.x() = point(0);
                pointPCL.position.y() = point(1);
                pointPCL.position.z() = point(2);
                pointPCL.color.r = 255;
                pointPCL.color.g = 255;
                pointPCL.color.b = 0;
                pointPCL.color.a = 0;
                //std::cout << "depth: " << depthM << " u: " << i << " v: " <<  j << " x y z " << pointPCL.position.position.x() << ", " << pointPCL.position.position.y() << "," << pointPCL.position.position.z() << "\n";
                cloud.push_back(pointPCL);
            }
        }
    }
    return cloud;
}

/// Convert disparity image to point cloud
PointCloud depth2cloud(cv::Mat& depthImage, cv::Mat& colorImage) {
    Eigen::Vector3d point;
    PointCloud cloud;
    cloud.clear();
    for (unsigned int i=0;i<depthImage.rows;i++){
        for (unsigned int j=0;j<depthImage.cols;j++){
            if(depthImage.at<uint16_t>(i,j)>800&&depthImage.at<uint16_t>(i,j)<8500){
                float_type depthM = float_type(depthImage.at<uint16_t>(i,j))*0.001;
                getPoint(j,i,depthM,point);
                Point3D pointPCL;
                pointPCL.position.x() = point(0);
                pointPCL.position.y() = point(1);
                pointPCL.position.z() = point(2);
                cv::Vec3i colorBGR = colorImage.at<cv::Vec3b>(cv::Point2f((float)j,(float)i));
                pointPCL.color.r = colorBGR.val[2];
                pointPCL.color.g = colorBGR.val[1];
                pointPCL.color.b = colorBGR.val[0];
                pointPCL.color.a = 0;
                //std::cout << "depth: " << depthM << " u: " << i << " v: " <<  j << " x y z " << pointPCL.position.position.x() << ", " << pointPCL.position.position.y() << "," << pointPCL.position.position.z() << "\n";
                cloud.push_back(pointPCL);
            }
        }
    }
    return cloud;
}

void getPoint(unsigned int u, unsigned int v, float_type depth, Eigen::Vector3d& point3D) {
    Eigen::Vector3d point(u, v, 1);
    Eigen::Matrix<float_type,3,3> PHCPModel;
    PHCPModel << 1/582.64,0,-320.17/582.64, 0,1/586.97, -260.0/586.97, 0,0,1;
    std::cout << PHCPModel << std::endl;
    point3D = depth*PHCPModel*point;
}
}

void octopointToPointcloud(octomap::Pointcloud& fromCloud, mapping::PointCloud toCloud) {
    octomap::point3d fromPoint;
    for(int  i = 0; i<fromCloud.size(); i++ ) {
        fromPoint = fromCloud.getPoint(i);
        mapping::Point3D toPoint = mapping::Point3D(fromPoint.x(), fromPoint.y(), fromPoint.z());
        toCloud.push_back(toPoint);
    }
}

void pointcloudToOctopoint(mapping::PointCloud& fromCloud, octomap::Pointcloud& toCloud) {
    for(mapping::Point3D point : fromCloud) {
        toCloud.push_back(point.position.x(), point.position.y(), point.position.z());
    }
}

/*
void getPoint(unsigned int u, unsigned int v, float_type depth, Eigen::Vector3d& point3D){
    Eigen::Vector3d point(u, v, 1);
    point3D = depth*PHCPModel*point;
}

/// Construction
UncertaintyModel(std::string configFile) : config(configFile){
    PHCPModel << 1/config.focalLength[0],0,-config.focalAxis[0]/config.focalLength[0],0,1/config.focalLength[1], -config.focalAxis[1]/config.focalLength[1], 0,0,1;
    Ruvd << config.varU, 0, 0, 0, config.varV, 0, 0, 0, 0;
}

Config() :
    focalLength{582.64, 586.97},
    focalAxis{320.17, 260.0},
    varU(1.1046), varV(0.64160),
    distVarCoefs{-8.9997e-06, 3.069e-003, 3.6512e-006, -0.0017512e-3}{}
}*/
