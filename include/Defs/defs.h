#ifndef DEFS_H_INCLUDED
#define DEFS_H_INCLUDED

#include <vector>
#include "../../3rdParty/Eigen/Geometry"
#include "octomap/Pointcloud.h"


/// mapping namespace
namespace mapping {


/// default floating point
typedef double float_type;

/// 3 element vector class
typedef Eigen::Translation<float_type,3> Vec3;

/// RGBA color space
class RGBA {
public:
    /// Color representation
    union {
        struct {
            std::uint8_t r, g, b, a;
        };
        std::uint8_t rgba_color[4];
    };
    /// Default constructor
    inline RGBA() {
        r = 255; g = 255; b = 255; a = 255;
    }
    /// Default constructor
    inline RGBA(int _r, int _g, int _b, int _a = 255) {
        r = _r;
        g = _g;
        b = _b;
        a = _a;
    }
};

/// 3D point representation
class Point3D {
public:
    /// Position
    Vec3 position;
    /// Colour
    RGBA color;

    /// Default constructor
    inline Point3D() {
    }


    inline Point3D(float x, float y, float z) {
        position = Vec3 (x, y, z);
    }

    inline Point3D(float x, float y, float z, int _r, int _g, int _b, int _a = 255) {
        position = Vec3 (x, y, z);
        color = RGBA(_r, _g, _b, _a);
    }
};

/// 3D point cloud representation
typedef std::vector<Point3D> PointCloud;

class GrabbedImage {
public:
    PointCloud pointCloud;
    std::string framePos;
    octomap::pose6d octoPose;

    inline GrabbedImage() {

    }

    inline GrabbedImage(PointCloud pc, std::string fP) {
       this->pointCloud = pc;
       this->framePos = fP;
    }

    inline GrabbedImage(PointCloud pc, octomap::pose6d fP) {
       this->pointCloud = pc;
       this->octoPose = fP;
    }
};

/// Quaternion representation of SO(3) group of rotations
typedef Eigen::Quaternion<float_type> Quaternion;

/// Homogeneous representation of SE(3) rigid body transformations
typedef Eigen::Transform<double, 3, Eigen::Affine> Mat34;

/// Voxel class definition
class Voxel {
public:
    struct NormalDist{
        double mean;
        double stdDev;
    };

    double probability;
    unsigned int sampNumber;
    NormalDist xAxis, yAxis, zAxis;

    ///default constructor
    inline Voxel(){
        probability = 0;
        sampNumber = 0;
        xAxis.mean = 0;
        xAxis.stdDev = 0;
        yAxis = xAxis = zAxis;
    }
    ///constructor
    inline Voxel(double prob, unsigned int samps, double xMean = 0, double xDev = 0, double yMean = 0, double yDev = 0, double zMean = 0, double zDev = 0) {
        probability = prob;
        sampNumber = samps;
        xAxis.mean = xMean;
        xAxis.stdDev = xDev;
        yAxis.mean = yMean;
        yAxis.stdDev = yDev;
        zAxis.mean = zMean;
        zAxis.stdDev = zDev;
    }
};

}

#endif
