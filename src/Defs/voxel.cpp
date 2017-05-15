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

void Voxel::update(std::vector<Vec3> measurements, double distance) {

    updateDistribution(measurements);


}

void Voxel::updateDistribution(std::vector<Vec3> measurements){

    Mat33 sampleVar;
    sampleVar << 0, 0, 0, 0, 0, 0, 0, 0, 0;

    Mat33 priorVar = var;

    int sampleNumber = measurements.size();
    double x=0,y=0,z=0;

    for(Vec3 &point : measurements) {
        x += point.x();
        y += point.y();
        z += point.z();

    }

    Eigen::Vector3d sampleMean = Eigen::Vector3d(x/sampleNumber, y/sampleNumber, z/sampleNumber);

    for(Vec3 &point : measurements){
        sampleVar(0, 0) += ((point.x() - sampleMean.x()) * (point.x() - sampleMean.x()));
        sampleVar(0, 1) += ((point.x() - sampleMean.x()) * (point.y() - sampleMean.y()));
        sampleVar(0, 2) += ((point.x() - sampleMean.x()) * (point.z() - sampleMean.z()));

        sampleVar(1, 1) += ((point.y() - sampleMean.y()) * (point.y() - sampleMean.y()));
        sampleVar(1, 2) += ((point.y() - sampleMean.y()) * (point.z() - sampleMean.z()));

        sampleVar(2, 2) += ((point.z() - sampleMean.z()) * (point.z() - sampleMean.z()));
    }
    sampleVar(1, 0) = sampleVar(0, 1);
    sampleVar(2, 0) = sampleVar(0, 2);
    sampleVar(2, 1) = sampleVar(1, 2);

    sampleVar /= sampleNumber;

    //var = Eigen::inverse((Eigen::inverse(priorVar) + sampleNumber * Eigen::inverse(sampleVar)));
    Mat33 temp = priorVar.inverse() + (sampleNumber * sampleVar.inverse());
    var = temp.inverse();
    mean = var * (sampleNumber * sampleVar.inverse()*sampleMean + priorVar.inverse() * mean);

}









}












