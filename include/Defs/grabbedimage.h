#ifndef GRABBEDIMAGE_H
#define GRABBEDIMAGE_H

#include "Defs/defs.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include "../../3rdParty/Eigen/Geometry"

namespace mapping {

class GrabbedImage {
public:
    PointCloud pointCloud;
    Vec3 translation;
    Quaternion orientation;
    std::vector<Mat33> uncertinatyErrors;
    Eigen::Vector3d cameraPos;


    GrabbedImage();
    GrabbedImage(PointCloud pc, Vec3 translation, Quaternion orientation, std::vector<Mat33> uncertinatyErrors, Eigen::Vector3d cameraPos);
    PointCloud transformedPointCloud();
};

}

#endif // GRABBEDIMAGE_H
