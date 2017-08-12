#include "Voxel/voxel.h"


/*
 * Voxel methods
 */

namespace mapping {

Voxel::Voxel(){
    probability = 0;
    sampNumber = 0;
    mean = Eigen::Vector3d(0, 0, 0);
    var << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    color = RGBA(255, 255, 255);
    this->type = TYPE_BAYES;
}

Voxel::Voxel(int res){
    probability = 0;
    sampNumber = 0;
    mean = Eigen::Vector3d(0, 0, 0);
    var << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    color = RGBA(255, 255, 255);
    this->type = TYPE_BAYES;
}

Voxel::Voxel(double prob, unsigned int samps, Eigen::Vector3d mean, Mat33 dev, RGBA color) {
    probability = prob;
    sampNumber = samps;
    this->mean = mean;
    this->var = dev;
    this->color = color;
    this->type = TYPE_BAYES;
}

void Voxel::insertPoint(Point3D point, Mat33 uncertaintyError) {

    updateDistribution(point, uncertaintyError);
    updateColor(point.color);
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

void Voxel::updateColor(RGBA color) {
    if(sampNumber == 1) {
        this->color = color;
    } else {
        this->color.r = ((this->color.r*(sampNumber-1)) + color.r)/sampNumber;
        this->color.g = ((this->color.g*(sampNumber-1)) + color.g)/sampNumber;
        this->color.b = ((this->color.b*(sampNumber-1)) + color.b)/sampNumber;
        this->color.a = ((this->color.a*(sampNumber-1)) + color.a)/sampNumber;
    }
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












