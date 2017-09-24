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
        /*A << std::atof(xmlDoc.FirstChildElement( "TransformCameraMatrix" )->FirstChildElement("a00")->GetText()),
                std::atof(xmlDoc.FirstChildElement( "TransformCameraMatrix" )->FirstChildElement("a01")->GetText()),
                std::atof(xmlDoc.FirstChildElement( "TransformCameraMatrix" )->FirstChildElement("a02")->GetText()),
                std::atof(xmlDoc.FirstChildElement( "TransformCameraMatrix" )->FirstChildElement("a03")->GetText()),
                std::atof(xmlDoc.FirstChildElement( "TransformCameraMatrix" )->FirstChildElement("a10")->GetText()),
                std::atof(xmlDoc.FirstChildElement( "TransformCameraMatrix" )->FirstChildElement("a11")->GetText()),
                std::atof(xmlDoc.FirstChildElement( "TransformCameraMatrix" )->FirstChildElement("a12")->GetText()),
                std::atof(xmlDoc.FirstChildElement( "TransformCameraMatrix" )->FirstChildElement("a13")->GetText()),
                std::atof(xmlDoc.FirstChildElement( "TransformCameraMatrix" )->FirstChildElement("a20")->GetText()),
                std::atof(xmlDoc.FirstChildElement( "TransformCameraMatrix" )->FirstChildElement("a21")->GetText()),
                std::atof(xmlDoc.FirstChildElement( "TransformCameraMatrix" )->FirstChildElement("a22")->GetText()),
                std::atof(xmlDoc.FirstChildElement( "TransformCameraMatrix" )->FirstChildElement("a23")->GetText()),
                0, 0, 0, 1;*/
        A << 0.241761029007299, -0.156482265388292, 0.957635058606503, 0.078542236056188,
                -0.159587283912986, -0.979884124280077, -0.119829052391907, -0.078052590612562,
                0.957122512360151, -0.123856382971628, -0.261870373907737, -0.061252007077847,
                0, 0, 0, 1;
    }
    newPC.reserve(this->pointCloud.size());
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
    int64 e1 = cv::getTickCount();
    for(mapping::Point3D point : this->pointCloud) {

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
