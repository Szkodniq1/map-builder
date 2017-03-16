//CPP File
#include "../include/Utilities/img2pcl.h"

namespace mapping {

img2pcl::img2pcl() {
    img2pcl("../resources");
}

img2pcl::img2pcl(std::string xmlPath) {
    tinyxml2::XMLDocument xmlDoc;
    tinyxml2::XMLError Result = xmlDoc.LoadFile(xmlPath.c_str());

    std::cout << Result <<std::endl;


    if(Result == tinyxml2::XML_SUCCESS) {
        xmlDoc.FirstChildElement("Color")->QueryIntText(&color);
        xmlDoc.FirstChildElement("fx")->QueryFloatText(&focalLength[0]);
        xmlDoc.FirstChildElement("fy")->QueryFloatText(&focalLength[1]);
        xmlDoc.FirstChildElement("cx")->QueryFloatText(&focalAxis[0]);
        xmlDoc.FirstChildElement("cy")->QueryFloatText(&focalAxis[1]);
        xmlDoc.FirstChildElement("factor")->QueryFloatText(&factor);

        path = xmlDoc.FirstChildElement( "Path" )->GetText();
        form = xmlDoc.FirstChildElement( "Form" )->GetText();
        fileName = xmlDoc.FirstChildElement( "stateFile" )->GetText();;
        stateFile.open(path + fileName, std::ios::in);

        if(!stateFile.is_open()) {
            path = "Loading Camera state file failed!\n";
            std::cout << path;
            exit(414);
        }
    }
    else {
        path = "Loading Frame Grabber configuration file failed!\n";
        std::cout << path << std::endl << xmlPath;
        exit(404);
    }
}

img2pcl::~img2pcl() {
    if(stateFile.is_open()) {
        std::cout << "State File closed!\n";
        stateFile.close();
    }


}

int img2pcl::grabFrame() {

    if(!stateFile.eof()) {
        std::string data;
        std::getline(stateFile, data);

        std::cout << data << std::endl;
        pos = data;

        std::stringstream dataStr(data);
        dataStr >> Timestamp;


        for(int i=0; i<3;i++)
            dataStr >> t[i];

        for(int i=0; i<4;i++)
            dataStr >> q[i];

        char buff[100];
        std::sprintf(buff, form.c_str(), Timestamp);
        depth = cv::imread(path + "depth/" + buff, CV_LOAD_IMAGE_ANYDEPTH);
        bgr = cv::imread(path + "rgb/" + buff);
//        cv::imshow("troll", depth);
//        cv::imshow("troll2", bgr);
//        cv::waitKey(30);

        return 1;

    }
    else {
        std::cout << "Koniec nagrania!" <<std::endl;
        return 0;
    }


}

int img2pcl::calcPCL() {
    if(color == 0) {
        depth2cloud();
    }
    else {
        depth2colorcloud();
    }
}

float img2pcl::z0(int u, int v, float_type d) {
    float dx = u-focalAxis[0], dy=v-focalAxis[1];
    return factor*sqrt(d*d/(dx*dx+factor*factor+dy*dy));
}

float img2pcl::x0(int u, float z) {
    float dx = u-focalAxis[0];
    return dx*z/focalLength[0];
}

float img2pcl::y0(int v, float z) {
    float dy = v-focalAxis[1];
    return -1*dy*z/focalLength[1];
}

Eigen::Vector3d img2pcl::xyz0(int u, int v, float_type d) {
    float z = z0(u,v,d);
    Eigen::Vector3d xyz(x0(u,z), y0(v,z), z);
    return xyz;
}

Eigen::Matrix<float_type,3,3> img2pcl::Rot() {
    float r11 = 1 - 2*q[1]*q[1] - 2*q[2]*q[2], r12 = 2*q[0]*q[1] - 2*q[2]*q[3],     r13 = 2*q[0]*q[2] + 2*q[1]*q[3],
          r21 = 2*q[0]*q[1] + 2*q[2]*q[3],     r22 = 1 - 2*q[0]*q[0] - 2*q[2]*q[2], r23 = 2*q[1]*q[2] - 2*q[0]*q[3],
          r31 = 2*q[0]*q[2] - 2*q[1]*q[3],     r32 = 2*q[1]*q[2] + 2*q[0]*q[3],     r33 = 1 - 2*q[0]*q[0] - 2*q[1]*q[1];

    Eigen::Matrix<float_type,3,3> Rotation;
    Rotation <<   r11, r12, r13,
                  r21, r22, r23,
                  r31, r32, r33;
    return Rotation;
}

Eigen::Vector3d img2pcl::Trans() {
    Eigen::Vector3d xyz(t[0],t[1],t[2]);
    return xyz;
}

octomap::pose6d img2pcl::FramePose() {
    return octomap::pose6d(octomath::Vector3(t[0],t[1],t[2]), octomath::Quaternion(q[3], q[0], q[1], q[2]));
}

int img2pcl::depth2cloud() {
    Eigen::Vector3d point;
    PointCloud tempCloud;
    tempCloud.clear();
    tempCloud.resize(depth.cols*depth.rows);

    Eigen::Matrix<float_type,3,3> R = Rot();
    Eigen::Vector3d T = Trans();

    uint16_t tmp;

    for (unsigned int i=0;i<depth.rows;i++) {
        for (unsigned int j=0;j<depth.cols;j++) {
            tmp = (depth.at<uint16_t>(i,j)>>3);
            if(tmp>800 && tmp<8500){
                float_type depthM = float_type(tmp)*0.001;

                point = xyz0(j,i,depthM);
                //getPoint(j,i,depthM,point);
                Point3D pointPCL;
                pointPCL.position.x() = point(0);
                pointPCL.position.y() = point(1);
                pointPCL.position.z() = point(2);
                pointPCL.color.r = 255;
                pointPCL.color.g = 255;
                pointPCL.color.b = 0;
                pointPCL.color.a = 0;
                //std::cout << "depth: " << depthM << " u: " << i << " v: " <<  j << " x y z " << pointPCL.position.position.x() << ", " << pointPCL.position.position.y() << "," << pointPCL.position.position.z() << "\n";
                tempCloud[depth.cols*i + j] = pointPCL;
            }
        }
    }
    Cloud = tempCloud;
    return 1;
}

/// Convert disparity image to point cloud
int img2pcl::depth2colorcloud() {
    Eigen::Vector3d point;
    PointCloud tempCloud;
    tempCloud.clear();
    tempCloud.resize(depth.cols*depth.rows);

    Eigen::Matrix<float_type,3,3> R = Rot();
    Eigen::Vector3d T = Trans();

    uint16_t tmp;

    for (unsigned int i=0;i<depth.rows;i++) {
        for (unsigned int j=0;j<depth.cols;j++) {
            tmp = (depth.at<uint16_t>(i,j)>>3);
            if(tmp>800 && tmp<8500){
                float_type depthM = float_type(tmp)*0.001;

                point = xyz0(j,i,depthM);
                //getPoint(j,i,depthM,point);
                Point3D pointPCL;
                pointPCL.position.x() = point(0);
                pointPCL.position.y() = point(1);
                pointPCL.position.z() = point(2);
                pointPCL.color.r = bgr.at<uint8_t>(i,3*j+2);
                pointPCL.color.g = bgr.at<uint8_t>(i,3*j+1);
                pointPCL.color.b = bgr.at<uint8_t>(i,3*j);
                pointPCL.color.a = 255;
                //std::cout << "depth: " << depthM << " u: " << i << " v: " <<  j << " x y z " << pointPCL.position.position.x() << ", " << pointPCL.position.position.y() << "," << pointPCL.position.position.z() << "\n";
                tempCloud[depth.cols*i + j] = pointPCL;


            }
        }
    }
    Cloud = tempCloud;
    return 1;
}

mapping::GrabbedImage img2pcl::returnPC()
{
    return mapping::GrabbedImage(Cloud, FramePose());
}

void getPoint(unsigned int u, unsigned int v, float_type depth, Eigen::Vector3d& point3D) {
    Eigen::Vector3d point(u, v, 1);
    Eigen::Matrix<float_type,3,3> PHCPModel;
    PHCPModel <<    1/582.64,   0,          -320.17/582.64,
                    0,          1/586.97,   -260.0/586.97,
                    0,          0,          1;
//    std::cout << PHCPModel << std::endl;
    point3D = depth*PHCPModel*point;

    std::cout<<point3D <<std::endl;
}
}

void octopointToPointcloud(octomap::Pointcloud& fromCloud, mapping::PointCloud toCloud) {
    octomap::point3d fromPoint;
    for(int  i = 0; i<fromCloud.size(); i++ ) {
        fromPoint = fromCloud.getPoint(i);
        mapping::Point3D toPoint = mapping::Point3D(fromPoint.x(), fromPoint.y(), fromPoint.z());
        toCloud.push_back(toPoint);
    }
}

void pointcloudToOctopoint(mapping::PointCloud& fromCloud, octomap::Pointcloud& toCloud) {
    for(mapping::Point3D point : fromCloud) {
        toCloud.push_back(point.position.x(), point.position.y(), point.position.z());
    }
}

/*
void getPoint(unsigned int u, unsigned int v, float_type depth, Eigen::Vector3d& point3D){
    Eigen::Vector3d point(u, v, 1);
    point3D = depth*PHCPModel*point;
}

/// Construction
UncertaintyModel(std::string configFile) : config(configFile){
    PHCPModel << 1/config.focalLength[0],0,-config.focalAxis[0]/config.focalLength[0],0,1/config.focalLength[1], -config.focalAxis[1]/config.focalLength[1], 0,0,1;
    Ruvd << config.varU, 0, 0, 0, config.varV, 0, 0, 0, 0;
}

Config() :
    focalLength{582.64, 586.97},
    focalAxis{320.17, 260.0},
    varU(1.1046), varV(0.64160),
    distVarCoefs{-8.9997e-06, 3.069e-003, 3.6512e-006, -0.0017512e-3}{}
}*/
