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

    int64 e1 = cv::getTickCount();
    for(mapping::Point3D point : this->pointCloud) {
        /*Eigen::Matrix4d T;
        Mat33 R = orientation.normalized().toRotationMatrix();
        T << R(0,0), R(0,1), R(0,2), translation.x(),
             R(1,0), R(1,1), R(1,2), translation.y(),
             R(2,0), R(2,1), R(2,2), translation.z(),
             0, 0, 0, 1;

        Eigen::Matrix4d Tp;
        Tp << 1, 0, 0, point.position.x(),
              0, 1, 0, point.position.y(),
              0, 0, 1, point.position.z(),
              0, 0, 0, 1;

        Eigen::Matrix4d wynik = T.inverse() * Tp;

        /*Eigen::Vector3d q = Eigen::Vector3d(point.position.x(), point.position.y(), point.position.z());
        q = transform.inverse() * q;
        Vec3(q(0), q(1), q(2));

        newPC.push_back(mapping::Point3D(
                    wynik(0,3),
                    wynik(1,3),
                    wynik(2,3),
                    point.color.r,
                    point.color.g,
                    point.color.b,
                    point.color.a));*/

        /// Performance +/- 0,6s left as it more readable solution
        Eigen::Matrix<double, 3, 1> pt (point.position.x(), point.position.y(), point.position.z());
                newPC.push_back(mapping::Point3D(
                            static_cast<double> (transform (0, 0) * pt.coeffRef (0) + transform (0, 1) * pt.coeffRef (1) + transform (0, 2) * pt.coeffRef (2) + transform (0, 3)),
                            static_cast<double> (transform (1, 0) * pt.coeffRef (0) + transform (1, 1) * pt.coeffRef (1) + transform (1, 2) * pt.coeffRef (2) + transform (1, 3)),
                            static_cast<double> (transform (2, 0) * pt.coeffRef (0) + transform (2, 1) * pt.coeffRef (1) + transform (2, 2) * pt.coeffRef (2) + transform (2, 3)),
                            point.color.r,
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
//                            -rotatedPt[2],
//                            point.color.r,
//                            point.color.g,
//                            point.color.b,
//                            point.color.a));

        /// Performance +/- 0,87s
//        Eigen::Matrix<double, 3, 1> pt (point.position.x(), point.position.y(), point.position.z());
//        Quaternion pos = orientation;
//        Eigen::Matrix4d transform;
//        transform<<     (1-2*pos.y()*pos.y()-2*pos.z()*pos.z()), 2*(pos.x()*pos.y() - pos.w()*pos.z()), 2*(pos.x()*pos.z() + pos.w()*pos.y()), translation.x(),
//                2*(pos.x()*pos.y() + pos.w()*pos.z()), (1-2*pos.x()*pos.x()-2*pos.z()*pos.z()), 2*(pos.y()*pos.z() + pos.w()*pos.x()), translation.y(),
//                2*(pos.x()*pos.z() - pos.w()*pos.y()), 2*(pos.y()*pos.z() - pos.w()*pos.x()), (1-2*pos.x()*pos.x()-2*pos.y()*pos.y()), translation.z(),
//                0, 0, 0, 1;

//        newPC.push_back(mapping::Point3D(
//                            static_cast<double> (transform (0, 0) * pt.coeffRef (0) + transform (0, 1) * pt.coeffRef (1) + transform (0, 2) * pt.coeffRef (2) + transform (0, 3)),
//                            static_cast<double> (transform (1, 0) * pt.coeffRef (0) + transform (1, 1) * pt.coeffRef (1) + transform (1, 2) * pt.coeffRef (2) + transform (1, 3)),
//                            static_cast<double> (transform (2, 0) * pt.coeffRef (0) + transform (2, 1) * pt.coeffRef (1) + transform (2, 2) * pt.coeffRef (2) + transform (2, 3)),
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
