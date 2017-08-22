#ifndef IMG2PCL_H
#define IMG2PCL_H

#include "../include/Defs/defs.h"
#include "../include/Defs/grabbedimage.h"
#include "Defs/occmethodtype.h"
#include "opencv2/opencv.hpp"
#include "../../3rdParty/Eigen/Dense"
#include "octomap/Pointcloud.h"
#include "octomap/math/Vector3.h"
#include "octomap/math/Quaternion.h"

#include "3rdParty/tinyXML/tinyxml2.h"
#include <fstream>
#include <string>
#include <math.h>

namespace mapping {

class img2pcl {
private:
    /// Method type
    OccMethodType methodType;
    //Path of images catalog and format of their naming
    std::string path;
    std::string form;

    //File containing camera parameters
    std::string fileName;
    std::ifstream stateFile;
    std::ifstream association_RGB;
    std::ifstream association_D;

    //Grabbed images of scene
    std::string Timestamp;
    int color;

    cv::Mat depth;
    cv::Mat bgr;

    //Camera parameters
    float focalLength[2];
    float focalAxis[2];
    float factor;
    float distVarCoefs[4];
    Mat33 Ruvd;


    //Position and orientation of camera
    float t[3];
    float q[4];
    Eigen::Vector3d cam;


    //Point Cloude generated from images
    PointCloud Cloud;
    //Position for each frame
    std::string pos;
    //uncertinaty errors list
    std::vector<Mat33> errors;


    Eigen::Translation<double,3> xyz0(int u, int v, double d);
    Eigen::Matrix<double,3,3> Rot();
    Eigen::Translation<double,3> Trans();
    octomap::pose6d FramePose();

    int depth2cloud();
    int depth2colorcloud();

public:
    img2pcl();
    img2pcl(std::string xmlPath);
    ~img2pcl();
    int grabFrame();
    int grabFrame2();
    int calcPCL();
    void computeCov(int u, int v, double depth, Mat33& cov);

    mapping::GrabbedImage returnPC();

};


    PointCloud depth2cloud(cv::Mat& depthImage);
    PointCloud depth2cloud(cv::Mat& depthImage, cv::Mat& colorImage);
    void octopointToPointcloud(octomap::Pointcloud& fromCloud, mapping::PointCloud toCloud);
    void pointcloudToOctopoint(mapping::PointCloud& fromCloud, octomap::Pointcloud& toCloud);
}

#endif // IMG2PCL_H
