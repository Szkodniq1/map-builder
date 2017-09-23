#include "grabbedimage.h"

namespace mapping {

GrabbedImage::GrabbedImage () {

}

GrabbedImage::GrabbedImage(PointCloud pc, Vec3 translation, Quaternion orientation, std::vector<Mat33> uncertinatyErrors, Eigen::Vector3d cameraPos) {
    this->pointCloud = pc;
    this->translation = translation;
    this->orientation = orientation;
    this->uncertinatyErrors = uncertinatyErrors;

    int matrixScale;
    tinyxml2::XMLDocument xmlDoc;
    xmlDoc.LoadFile("../../resources/config.xml");
    xmlDoc.FirstChildElement("TransformCameraScale")->QueryIntText(&matrixScale);
    this->cameraPos = matrixScale * cameraPos;
}

PointCloud GrabbedImage::transformedPointCloud() {
    PointCloud newPC;

    Eigen::Matrix4d A;
    tinyxml2::XMLDocument xmlDoc;
    xmlDoc.LoadFile("../../resources/config.xml");

    A = Eigen::Matrix4d::Identity();
    int shouldLoadMatrix, matrixScale;
    xmlDoc.FirstChildElement("TransformCamera")->QueryIntText(&shouldLoadMatrix);
    xmlDoc.FirstChildElement("TransformCameraScale")->QueryIntText(&matrixScale);
    if(shouldLoadMatrix != 0) {
        xmlDoc.FirstChildElement( "TransformCameraMatrix" )->QueryDoubleAttribute("a00",&A(0,0));
        xmlDoc.FirstChildElement( "TransformCameraMatrix" )->QueryDoubleAttribute("a01",&A(0,1));
        xmlDoc.FirstChildElement( "TransformCameraMatrix" )->QueryDoubleAttribute("a02",&A(0,2));
        xmlDoc.FirstChildElement( "TransformCameraMatrix" )->QueryDoubleAttribute("a03",&A(0,3));
        xmlDoc.FirstChildElement( "TransformCameraMatrix" )->QueryDoubleAttribute("a10",&A(1,0));
        xmlDoc.FirstChildElement( "TransformCameraMatrix" )->QueryDoubleAttribute("a11",&A(1,1));
        xmlDoc.FirstChildElement( "TransformCameraMatrix" )->QueryDoubleAttribute("a12",&A(1,2));
        xmlDoc.FirstChildElement( "TransformCameraMatrix" )->QueryDoubleAttribute("a13",&A(1,3));
        xmlDoc.FirstChildElement( "TransformCameraMatrix" )->QueryDoubleAttribute("a20",&A(2,0));
        xmlDoc.FirstChildElement( "TransformCameraMatrix" )->QueryDoubleAttribute("a21",&A(2,1));
        xmlDoc.FirstChildElement( "TransformCameraMatrix" )->QueryDoubleAttribute("a22",&A(2,2));
        xmlDoc.FirstChildElement( "TransformCameraMatrix" )->QueryDoubleAttribute("a23",&A(2,3));
    }

    newPC.reserve(this->pointCloud.size());

    int64 e1 = cv::getTickCount();
    for(mapping::Point3D point : this->pointCloud) {
        Eigen::Matrix4d T;
        Mat33 R = orientation.normalized().toRotationMatrix();
        T << R(0,0), R(0,1), R(0,2), translation.x(),
                R(1,0), R(1,1), R(1,2), translation.y(),
                R(2,0), R(2,1), R(2,2), translation.z(),
                0, 0, 0, 1;
        T = matrixScale * T;
        if(shouldLoadMatrix != 0) {
            T = T * A;
        }
        Eigen::Matrix4d Tp;
        Tp << 1, 0, 0, point.position.x(),
                0, 1, 0, point.position.y(),
                0, 0, 1, point.position.z(),
                0, 0, 0, 1;

        Eigen::Matrix4d wynik = T * Tp;

        newPC.push_back(mapping::Point3D(
                            wynik(0,3),
                            wynik(1,3),
                            wynik(2,3),
                            point.color.r,
                            point.color.g,
                            point.color.b,
                            point.color.a));


    }
    int64 e2 = cv::getTickCount();
    double time = ((e2 - e1)/ cv::getTickFrequency());
    std::cout<<"Calc time: "<<time<<std::endl;
    return newPC;
}


}
