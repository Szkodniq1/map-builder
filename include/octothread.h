#ifndef OCTOTHREAD_H
#define OCTOTHREAD_H

//#include <QThread>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
//#include <pcl/io/pcd_io.h>

using namespace std;
//using namespace pcl;
using namespace octomap;

//class OctoThread : public QThread
//{

//public:
//    void setPointCloudData(vector<PointXYZRGBA, Eigen::aligned_allocator<PointXYZRGBA> > points, Eigen::Vector4f sensor_origin);
//    void setTree(OcTree* tree);
//    Q_OBJECT void run() Q_DECL_OVERRIDE;
//private:
//    vector<PointXYZRGBA, Eigen::aligned_allocator<PointXYZRGBA> > points;
//    Eigen::Vector4f sensor_origin;
//    Pointcloud pc;
//    OcTree *tree;

//};

#endif // OCTOTHREAD_H
