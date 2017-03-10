#ifndef IMG2PCL_H
#define IMG2PCL_H

#include "../include/Defs/defs.h"
#include "opencv2/opencv.hpp"
#include "../../3rdParty/Eigen/Dense"
#include "octomap/Pointcloud.h"

namespace mapping {

    PointCloud depth2cloud(cv::Mat& depthImage);
    PointCloud depth2cloud(cv::Mat& depthImage, cv::Mat& colorImage);
    void getPoint(unsigned int u, unsigned int v, float_type depth, Eigen::Vector3d& point3D);
    void octopointToPointcloud(octomap::Pointcloud& fromCloud, mapping::PointCloud toCloud);
    void pointcloudToOctopoint(mapping::PointCloud& fromCloud, octomap::Pointcloud& toCloud);
}

#endif // IMG2PCL_H
