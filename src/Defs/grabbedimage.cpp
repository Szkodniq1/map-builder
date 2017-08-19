#include "grabbedimage.h"

namespace mapping {

GrabbedImage::GrabbedImage () {

}

GrabbedImage::GrabbedImage(PointCloud pc, Vec3 translation, Quaternion orientation, std::vector<Mat33> uncertinatyErrors, Eigen::Vector3d cameraPos) {
    this->pointCloud = pc;
    this->translation = translation;
    this->orientation = orientation;
    this->uncertinatyErrors = uncertinatyErrors;
    this->cameraPos = cameraPos;
}


PointCloud GrabbedImage::transformedPointCloud() {
    PointCloud newPC;
    Eigen::Transform<double, 3, Eigen::Affine> transform (translation * orientation);
    newPC.reserve(this->pointCloud.size());
    this->orientation.normalize();

    int64 e1 = cv::getTickCount();
    for(mapping::Point3D point : this->pointCloud) {
        /// Performance +/- 0,6s left as it more readable solution
        Eigen::Matrix<double, 3, 1> pt (point.position.x(), point.position.y(), point.position.z());
                newPC.push_back(mapping::Point3D(
                            static_cast<double> (transform (0, 0) * pt.coeffRef (0) + transform (0, 1) * pt.coeffRef (1) + transform (0, 2) * pt.coeffRef (2) + transform (0, 3)),
                            static_cast<double> (transform (1, 0) * pt.coeffRef (0) + transform (1, 1) * pt.coeffRef (1) + transform (1, 2) * pt.coeffRef (2) + transform (1, 3)),
                            static_cast<double> (transform (2, 0) * pt.coeffRef (0) + transform (2, 1) * pt.coeffRef (1) + transform (2, 2) * pt.coeffRef (2) + transform (2, 3)),point.color.r,
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
    return newPC;
}


}
