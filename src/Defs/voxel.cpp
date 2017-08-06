#include "Defs/voxel.h"


/*
 * Voxel methods
 */


namespace mapping{

Voxel::Voxel(){
    probability = 0;
    sampNumber = 0;
    mean = Eigen::Vector3d(0, 0, 0);
    var << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    color = RGBA(255, 255, 255);
}

Voxel::Voxel(int res){
    probability = 0;
    sampNumber = 0;
    mean = Eigen::Vector3d(0, 0, 0);
    var << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    color = RGBA(255, 255, 255);
}

Voxel::Voxel(double prob, unsigned int samps, Eigen::Vector3d mean, Mat33 dev, RGBA color) {
    probability = prob;
    sampNumber = samps;
    this->mean = mean;
    this->var = dev;
    this->color = color;

}

void Voxel::insertPoint(Point3D point) {
    this->points.push_back(point);
    updateOccupancy();
}

void Voxel::update(int x, int y, int z) {
    mean = Eigen::Vector3d(0, 0, 0);
    var << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    color = RGBA(255, 255, 255);
    updateDistribution(x, y, z);
    updateColor();
}

void Voxel::updateDistribution(int x, int y, int z) {
    for(mapping::Point3D &point : points) {
        Eigen::Vector3d newPoint = Eigen::Vector3d(point.position.x(), point.position.y(), point.position.z());
        mean += newPoint;
    }
    mean = mean / points.size();
    if(points.size() > 1) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                var(i,j) = 0.0;
                for(mapping::Point3D &point : points){
                    Eigen::Vector3d newPoint = Eigen::Vector3d(point.position.x(), point.position.y(), point.position.z());
                    var(i,j) += (mean(i) - newPoint(i)) * (mean(j) - newPoint(j));
                }
                var(i,j) /= points.size() - 1;
            }
        }
    }
}

void Voxel::updateColor() {
    long r,g,b,a;
    r=g=b=a=0;
    for(mapping::Point3D &point : points) {
        r += point.color.r;
        g += point.color.g;
        b += point.color.b;
        a += point.color.a;
    }

    this->color.r = (int) r/points.size();
    this->color.g = (int) g/points.size();
    this->color.b = (int) b/points.size();
    this->color.a = (int) a/points.size();
}

void Voxel::updateOccupancy() {
    if(probability<100)
        ++probability;
}

void Voxel::updateNullOccupancy() {
    if(probability>0)
        --probability;
}
}












