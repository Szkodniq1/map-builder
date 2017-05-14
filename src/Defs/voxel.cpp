#include "voxel.h"

/*
 * Voxel methods
 */

namespace mapping{

void Voxel::update(std::vector<Vec3> measurements, double distance) {

    updateDistribution(measurements, distance);


}

void Voxel::updateDistribution(std::vector<Vec3> measurements){

    Vec3 sampleMean = Vec3(0, 0, 0);
    Mat33 sampleVar = (Mat33() << 0, 0, 0, 0, 0, 0, 0, 0, 0);

    Mat33 priorVar = var;

    int sampleNumber = measurements.size();

    for(Vec3 &point : measurements)
        sampleMean += point;

    sampleMean /= sampleNumber;

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

    var = Eigen::inverse((Eigen::inverse(priorVar) + sampleNumber * Eigen::inverse(sampleVar)));
    mean = var * (sampleNumber * Eigen::inverse(sampleVar)*sampleMean + Eigen::inverse(priorVar) * mean);

}









}











