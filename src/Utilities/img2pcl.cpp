//CPP File
#include "../include/Utilities/img2pcl.h"

namespace mapping {

img2pcl::img2pcl() {
    img2pcl("../resources");
}

img2pcl::img2pcl(std::string xmlPath) {
    tinyxml2::XMLDocument xmlDoc;
    tinyxml2::XMLError Result = xmlDoc.LoadFile(xmlPath.c_str());
    int asso;

    std::cout << Result <<std::endl;


    if(Result == tinyxml2::XML_SUCCESS) {
        xmlDoc.FirstChildElement("Color")->QueryIntText(&color);
        xmlDoc.FirstChildElement("Association")->QueryIntText(&asso);
        xmlDoc.FirstChildElement("fx")->QueryFloatText(&focalLength[0]);
        xmlDoc.FirstChildElement("fy")->QueryFloatText(&focalLength[1]);
        xmlDoc.FirstChildElement("cx")->QueryFloatText(&focalAxis[0]);
        xmlDoc.FirstChildElement("cy")->QueryFloatText(&focalAxis[1]);
        xmlDoc.FirstChildElement("factor")->QueryFloatText(&factor);
        xmlDoc.FirstChildElement( "varianceDepth" )->QueryFloatAttribute("c3", &distVarCoefs[0]);
        xmlDoc.FirstChildElement( "varianceDepth" )->QueryFloatAttribute("c2",&distVarCoefs[1]);
        xmlDoc.FirstChildElement( "varianceDepth" )->QueryFloatAttribute("c1",&distVarCoefs[2]);
        xmlDoc.FirstChildElement( "varianceDepth" )->QueryFloatAttribute("c0",&distVarCoefs[3]);
        Ruvd << 0, 0, 0, 0, 0, 0, 0, 0, 0;
        xmlDoc.FirstChildElement( "variance" )->QueryDoubleAttribute("sigmaU",&Ruvd(0,0));
        xmlDoc.FirstChildElement( "variance" )->QueryDoubleAttribute("sigmaV",&Ruvd(1,1));



        path = xmlDoc.FirstChildElement( "Path" )->GetText();
        form = xmlDoc.FirstChildElement( "Form" )->GetText();
        fileName = xmlDoc.FirstChildElement( "stateFile" )->GetText();;
        stateFile.open(path + fileName, std::ios::in);

        if(!stateFile.is_open()) {
            path = "Loading Camera state file failed!\n";
            std::cout << path;
            exit(414);
        }

        if(asso==1){
            association_D.open(path+"depth.txt", std::ios::in);
            if(!association_D.is_open()) {
                path = "Loading Camera association file failed!\n";
                std::cout << path;
                exit(424);
            }

            association_RGB.open(path+"rgb.txt", std::ios::in);
            if(!association_RGB.is_open()) {
                path = "Loading Camera association file failed!\n";
                std::cout << path;
                exit(424);
            }
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

    if(association_D.is_open()) {
        std::cout << "Association File closed!\n";
        association_D.close();
    }

    if(association_RGB.is_open()) {
        std::cout << "Association File closed!\n";
        association_RGB.close();
    }
}

int img2pcl::grabFrame() {

    if(!stateFile.eof()) {
        std::string data = "#";

        while(data[0] == '#')
            std::getline(stateFile, data);


        std::cout << data << std::endl;
        pos = data;

        std::stringstream dataStr(data);

        std::getline(dataStr, Timestamp,' ');


        for(int i=0; i<3;i++)
            dataStr >> t[i];

        for(int i=0; i<4;i++)
            dataStr >> q[i];

        char buff[100];
        std::sprintf(buff, form.c_str(), Timestamp.c_str());
        depth = cv::imread(path + "depth/" + buff, CV_LOAD_IMAGE_ANYDEPTH);
        bgr = cv::imread(path + "rgb/" + buff);
        //        cv::imshow("troll", depth);
        //        cv::imshow("troll2", bgr);
        //        cv::waitKey(0);

        return 1;

    }
    else {
        std::cout << "Koniec nagrania!" <<std::endl;
        return 0;
    }


}

int img2pcl::grabFrame2() {

    if(!association_D.eof()) {
        std::string params = "#", time;

        std::string depthName = "#";
        while(depthName[0] == '#')
        {
            std::getline(association_D, depthName);
            std::cout << depthName << std::endl;
        }
        std::string rgbName = "#";
        while(rgbName[0] == '#')
            std::getline(association_RGB, rgbName);

        std::stringstream dataStr(depthName);
        std::getline(dataStr, Timestamp,' ');

        char buff[100];
        std::sprintf(buff, form.c_str(), Timestamp.c_str());
        depth = cv::imread(path + "depth/" + buff, CV_LOAD_IMAGE_ANYDEPTH);

        dataStr.str(rgbName);
        std::getline(dataStr, rgbName,' ');
        std::sprintf(buff, form.c_str(), rgbName.c_str());
        bgr = cv::imread(path + "rgb/" + buff);
        //        cv::imshow("troll", depth);
        //        cv::imshow("troll2", bgr);
        //        cv::waitKey(30);

        Timestamp.resize(Timestamp.size() - 4);

        while(Timestamp != time) {
            params = "#";
            while(params[0] == '#')
                std::getline(stateFile, params);

            std::cout << params << std::endl;
            pos = params;


            dataStr.str(params);
            std::getline(dataStr, time,' ');
            time.resize(time.size() - 2);

            std::cout << time << std::endl;
        }

        for(int i=0; i<3;i++)
            dataStr >> t[i];

        for(int i=0; i<4;i++)
            dataStr >> q[i];

        return 1;

    }
    else {
        std::cout << "Koniec nagrania!" <<std::endl;
        return 0;
    }


}

/// u,v [px], depth [m]
void img2pcl::computeCov(int u, int v, double depth, Mat33& cov) {
    Mat33 J;
    J << depth/focalLength[0], 0, ((u/focalLength[0])-(focalAxis[0]/focalLength[0])),
            0, depth/focalLength[1], ((v/focalLength[1])-(focalAxis[1]/focalLength[1])),
            0, 0, 1;
    Ruvd(2,2) = distVarCoefs[0]*pow(depth,3.0) + distVarCoefs[1]*pow(depth,2.0) + distVarCoefs[2]*depth + distVarCoefs[3];
    cov=J*Ruvd*J.transpose();
}

int img2pcl::calcPCL() {
    if(color == 0) {
        depth2cloud();
    }
    else {
        depth2colorcloud();
    }
}

Eigen::Translation<double,3> img2pcl::xyz0(int u, int v, double d) {
    Eigen::Vector3d point(u, v, 1);
    Eigen::Matrix<double,3,3> PHCPModel;
    PHCPModel <<1/focalLength[0],   0,                  -focalAxis[0]/focalLength[0],
            0,                  1/focalLength[1],   -focalAxis[1]/focalLength[1],
            0,                  0,                  1;
    Eigen::Translation<double,3> xyz(d*PHCPModel*point);
    return xyz;
}

Eigen::Matrix<double,3,3> img2pcl::Rot() {
    float r11 = 1 - 2*q[1]*q[1] - 2*q[2]*q[2], r12 = 2*q[0]*q[1] - 2*q[2]*q[3],     r13 = 2*q[0]*q[2] + 2*q[1]*q[3],
            r21 = 2*q[0]*q[1] + 2*q[2]*q[3],     r22 = 1 - 2*q[0]*q[0] - 2*q[2]*q[2], r23 = 2*q[1]*q[2] - 2*q[0]*q[3],
            r31 = 2*q[0]*q[2] - 2*q[1]*q[3],     r32 = 2*q[1]*q[2] + 2*q[0]*q[3],     r33 = 1 - 2*q[0]*q[0] - 2*q[1]*q[1];

    Eigen::Matrix<double,3,3> Rotation;
    Rotation <<   r11, r12, r13,
            r21, r22, r23,
            r31, r32, r33;
    return Rotation;
}

Eigen::Translation<double,3> img2pcl::Trans() {
    Eigen::Translation<double,3> xyz(t[0],t[1],t[2]);
    return xyz;
}

octomap::pose6d img2pcl::FramePose() {
    return octomap::pose6d(octomath::Vector3(t[0],t[1],t[2]), octomath::Quaternion(q[3], q[0], q[1], q[2]));
}

/// Convert disparity image to point cloud
int img2pcl::depth2cloud() {
    Eigen::Translation<double,3> point;
    PointCloud tempCloud;
    tempCloud.clear();
    std::vector<Mat33> uncertinatyErrors;

    Eigen::Matrix<double,3,3> R = Rot();
    Eigen::Translation<double,3> T = Trans();

    uint16_t tmp;

    for (unsigned int i=0;i<depth.rows;i++) {
        for (unsigned int j=0;j<depth.cols;j++) {
            tmp = (depth.at<uint16_t>(i,j));
            if(tmp>800 && tmp<60000){
                double depthM = double(tmp)/factor;
                Mat33 uncError;
                if (methodType.type != MethodType::TYPE_SIMPLE) {
                    computeCov(j,i,depthM,uncError);
                }
                uncertinatyErrors.push_back(uncError);
                point = xyz0(j,i,depthM);
                Point3D pointPCL;
                pointPCL.position.x() = point.x();
                pointPCL.position.y() = point.y();
                pointPCL.position.z() = point.z();
                pointPCL.color.r = 255;
                pointPCL.color.g = 255;
                pointPCL.color.b = 0;
                pointPCL.color.a = 0;
                //std::cout << "depth: " << depthM << " u: " << i << " v: " <<  j << " x y z " << pointPCL.position.position.x() << ", " << pointPCL.position.position.y() << "," << pointPCL.position.position.z() << "\n";
                tempCloud.push_back(pointPCL);
            }
        }
    }
    errors = uncertinatyErrors;
    Cloud = tempCloud;
    return 1;
}

int img2pcl::depth2colorcloud() {
    Eigen::Translation<double,3> point;
    PointCloud tempCloud;
    tempCloud.clear();
    std::vector<Mat33> uncertinatyErrors;

    //    Eigen::Matrix<float_type,3,3> R = Rot();
    //    Eigen::Translation<float_type,3> T = Trans();

    uint16_t tmp;

    for (unsigned int i=0;i<depth.rows;i++) {
        for (unsigned int j=0;j<depth.cols;j++) {
            tmp = (depth.at<uint16_t>(i,j));
            if(tmp>800 && tmp<60000){
                double depthM = double(tmp)/factor;
                Mat33 uncError;
                if (methodType.type != MethodType::TYPE_SIMPLE) {
                    computeCov(j,i,depthM,uncError);
                }
                uncertinatyErrors.push_back(uncError);
                point = xyz0(j,i,depthM);
                Point3D pointPCL;
                pointPCL.position.x() = point.x();
                pointPCL.position.y() = point.y();
                pointPCL.position.z() = point.z();
                pointPCL.color.r = bgr.at<uint8_t>(i,3*j+2);
                pointPCL.color.g = bgr.at<uint8_t>(i,3*j+1);
                pointPCL.color.b = bgr.at<uint8_t>(i,3*j);
                pointPCL.color.a = 255;
                //std::cout << "depth: " << depthM << " u: " << i << " v: " <<  j << " x y z " << pointPCL.position.position.x() << ", " << pointPCL.position.position.y() << "," << pointPCL.position.position.z() << "\n";
                tempCloud.push_back(pointPCL);


            }
        }
    }
    errors = uncertinatyErrors;
    Cloud = tempCloud;
    return 1;
}

mapping::GrabbedImage img2pcl::returnPC() {
    return mapping::GrabbedImage(Cloud, Vec3(t[0],t[1],t[2]), Quaternion(q[3], q[0], q[1], q[2]), errors, Eigen::Vector3d(t[0], t[1], t[2]));
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
