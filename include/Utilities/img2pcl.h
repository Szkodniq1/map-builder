#ifndef IMG2PCL_H
#define IMG2PCL_H

#include "../include/Defs/defs.h"
#include "opencv2/opencv.hpp"
#include "../../3rdParty/Eigen/Dense"

namespace mapping {

    PointCloud depth2cloud(cv::Mat& depthImage);
    PointCloud depth2cloud(cv::Mat& depthImage, cv::Mat& colorImage);
    void getPoint(unsigned int u, unsigned int v, float_type depth, Eigen::Vector3d& point3D);
}

#endif // IMG2PCL_H
