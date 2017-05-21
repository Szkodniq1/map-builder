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

void Voxel::update(Point3D point, Mat33 uncertaintyError) {

    updateDistribution(point, uncertaintyError);
    updateOccupancy();

}

void Voxel::updateDistribution(Point3D point, Mat33 uncertaintyError) {

    Mat33 sampleVar;
    sampleVar << 0, 0, 0, 0, 0, 0, 0, 0, 0;

    Mat33 priorVar;
    if(var.sum() == 0) {
        priorVar<< 1,0,0,0,1,0,0,0,1;
    } else {
        priorVar = var;
    }

    ++sampNumber;

    Eigen::Vector3d sampleMean = Eigen::Vector3d(point.position.x(), point.position.y(), point.position.z());


    sampleVar(0, 0) += ((point.position.x() - sampleMean.x()) * (point.position.x() - sampleMean.x()));
    sampleVar(0, 1) += ((point.position.x() - sampleMean.x()) * (point.position.y() - sampleMean.y()));
    sampleVar(0, 2) += ((point.position.x() - sampleMean.x()) * (point.position.z() - sampleMean.z()));

    sampleVar(1, 1) += ((point.position.y() - sampleMean.y()) * (point.position.y() - sampleMean.y()));
    sampleVar(1, 2) += ((point.position.y() - sampleMean.y()) * (point.position.z() - sampleMean.z()));

    sampleVar(2, 2) += ((point.position.z() - sampleMean.z()) * (point.position.z() - sampleMean.z()));

    sampleVar(1, 0) = sampleVar(0, 1);
    sampleVar(2, 0) = sampleVar(0, 2);
    sampleVar(2, 1) = sampleVar(1, 2);

    Mat33 temp;
    temp = priorVar.inverse();
    temp += (sampNumber * sampleVar.inverse());
    var = temp.inverse();
    mean = var * (sampNumber * sampleVar.inverse()*sampleMean + priorVar.inverse() * mean);

}

void Voxel::updateOccupancy() {
    probability = 1;
}



}












