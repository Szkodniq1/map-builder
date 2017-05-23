#include "Defs/voxel.h"


/*
 * Voxel methods
 */


namespace mapping{

Voxel::Voxel(){
    probability = 0;
    sampNumber = 0;
    mean = Eigen::Vector3d(0, 0, 0);
    var << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    color = RGBA(255, 255, 255);
}

Voxel::Voxel(int res){
    probability = 0;
    sampNumber = 0;
    mean = Eigen::Vector3d(0, 0, 0);
    var << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    color = RGBA(255, 255, 255);
}

Voxel::Voxel(double prob, unsigned int samps, Eigen::Vector3d mean, Mat33 dev, RGBA color) {
    probability = prob;
    sampNumber = samps;
    this->mean = mean;
    this->var = dev;
    this->color = color;

}

void Voxel::update(Point3D point, Mat33 uncertaintyError) {

    updateDistribution(point, uncertaintyError);
    updateOccupancy();

}

void Voxel::updateDistribution(Point3D point, Mat33 uncertaintyError) {

    Mat33 priorVar;
    priorVar = var;

    Eigen::Vector3d sampleMean = Eigen::Vector3d(point.position.x(), point.position.y(), point.position.z());

    Mat33 Inv = uncertaintyError.inverse();
    double det = Inv.determinant();

    if(det == 0)
        return;

    ++sampNumber;

    Mat33 temp;
    temp = var.inverse();
    temp += (sampNumber * Inv);
    var = temp.inverse();
    mean = var * (sampNumber * Inv*sampleMean + priorVar.inverse() * mean);

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












