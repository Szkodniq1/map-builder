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

void Voxel::update() {
    updateDistribution();
    updateColor();
}

void Voxel::updateDistribution() {

    for(mapping::Point3D &point : points) {
        Eigen::Vector3d newPoint = Eigen::Vector3d(point.position.x(), point.position.y(), point.position.z());
        mean += newPoint;
    }
    mean = mean / points.size();

    for(mapping::Point3D &point : points) {
        var(0,0) += pow(point.position.x() - mean(0), 2.0);
        var(1,1) += pow(point.position.y() - mean(1), 2.0);
        var(2,2) += pow(point.position.z() - mean(2), 2.0);

        var(0,1) += (point.position.x() - mean(0)) * (point.position.y() - mean(1));
        var(0,2) += (point.position.x() - mean(0)) * (point.position.z() - mean(2));

        var(1,0) += (point.position.y() - mean(1)) * (point.position.x() - mean(0));
        var(1,2) += (point.position.y() - mean(1)) * (point.position.z() - mean(2));

        var(2,0) += (point.position.z() - mean(2)) * (point.position.x() - mean(0));
        var(2,1) += (point.position.z() - mean(2)) * (point.position.y() - mean(1));
    }

    var = var / points.size();
}

void Voxel::updateColor() {
    for(mapping::Point3D &point : points) {
        this->color.r += point.color.r;
        this->color.g += point.color.g;
        this->color.b += point.color.b;
        this->color.a += point.color.a;
    }

    this->color.r = this->color.r/points.size();
    this->color.g = this->color.g/points.size();
    this->color.b = this->color.b/points.size();
    this->color.a = this->color.a/points.size();
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












