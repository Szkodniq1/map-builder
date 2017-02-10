#include <iostream>
#include "3rdParty/tinyXML/tinyxml2.h"
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace cv;
using namespace std;


int main()
{
    tinyxml2::XMLDocument config;
    config.LoadFile("../map-builder/resources/config.xml");
    if (config.ErrorID())
        cout << "unable to load config file.\n";
    string imagePath(config.FirstChildElement( "TestSource" )->FirstChildElement( "path" )->GetText());

    Mat rgb, depth;
    rgb = imread( imagePath + "/rgb/0.png", 1 );
    depth = imread(imagePath + "/depth/0.png", 1);
    cvtColor( depth, depth, CV_BGR2GRAY );

    namedWindow("rgb", 1);
    imshow("rgb", rgb);
    namedWindow("depth", 1);
    imshow("depth", depth);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PCLPointCloud2 cloud_blob;
    pcl::io::loadPCDFile (imagePath + "/pcd/0.pcd", cloud_blob);
    pcl::fromPCLPointCloud2 (cloud_blob, *cloud);

    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud(cloud);

    while(1) {
        if(waitKey(30) == 0) {
            break;
        }
    }

    return 0;
}
