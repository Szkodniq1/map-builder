#include "octothread.h"

void OctoThread::setPointCloudData(vector<PointXYZRGBA, Eigen::aligned_allocator<PointXYZRGBA> > points, Eigen::Vector4f sensor_origin) {
    this->points = points;
    this->sensor_origin = sensor_origin;
}

void OctoThread::run() {

    for(int i=0; i<points.size(); i++) {
        pc.push_back(((PointXYZRGBA) points.at(i)).x, ((PointXYZRGBA) points.at(i)).y, ((PointXYZRGBA) points.at(i)).z);
    }
    point3d origin (sensor_origin(0), sensor_origin(1), sensor_origin(2));
    this->tree->insertPointCloud(pc, origin);
}

void OctoThread::setTree(OcTree* tree) {
    this->tree = tree;
}
