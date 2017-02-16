#ifndef DEFS_H_INCLUDED
#define DEFS_H_INCLUDED

#include <vector>
#include "../../3rdParty/Eigen/Geometry"

/// mapping namespace
namespace mapping {


    /// default floating point
    typedef double float_type;

    /// 3 element vector class
    typedef Eigen::Translation<float_type,3> Vec3;

    /// 3D point representation
    typedef Vec3 Point3D;

    /// 3D point cloud representation
    typedef std::vector<Point3D> PointCloud;

    /// Quaternion representation of SO(3) group of rotations
    typedef Eigen::Quaternion<float_type> Quaternion;

    /// Homogeneous representation of SE(3) rigid body transformations
    typedef Eigen::Transform<double, 3, Eigen::Affine> Mat34;
}

#endif
