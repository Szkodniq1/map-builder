#ifndef GRABBEDIMAGE_H
#define GRABBEDIMAGE_H

#include "Defs/defs.h"
#include <iostream>
#include <opencv2/core/core.hpp>

namespace mapping {

class GrabbedImage {
public:
    PointCloud pointCloud;
    Vec3 translation;
    Quaternion orientation;

    GrabbedImage();
    GrabbedImage(PointCloud pc, Vec3 translation, Quaternion orientation);
    PointCloud transformedPointCloud();
};

}

#endif // GRABBEDIMAGE_H
