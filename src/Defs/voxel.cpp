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

void Voxel::update(Point3D point, Mat33 uncertaintyError, bool printlog) {

    updateDistribution(point, uncertaintyError, printlog);
    updateOccupancy();

}

void Voxel::updateDistribution(Point3D point, Mat33 uncertaintyError, bool printlog) {

    Mat33 priorVar;
    priorVar = var;

    sampNumber++;

//    for(int i = 0; i<9 ;i++) {
//        uncertaintyError(i) = ((int) (uncertaintyError(i) * 100000000.0))  / 100000000.0;
//    }

    Eigen::Vector3d sampleMean = Eigen::Vector3d(point.position.x(), point.position.y(), point.position.z());
    Eigen::FullPivLU<Mat33> lu(var);
    Mat33 invertedVar = lu.inverse();
    Eigen::FullPivLU<Mat33> lu2(uncertaintyError);
    Mat33 invertedError = lu2.inverse();
    Mat33 invertedPriori = priorVar.inverse();

    if(printlog) {
        std::cout<<"ITERATION "<<sampNumber<<std::endl;
        std::cout<<"INCVERSE VAR"<<std::endl;
        std::cout<<invertedVar<<std::endl;
        std::cout<<"INCVERSE ERROR"<<std::endl;
        std::cout<<invertedError<<std::endl;
        std::cout<<"VAR"<<std::endl;
        std::cout<<var<<std::endl;
        std::cout<<"ERROR"<<std::endl;
        std::cout<<uncertaintyError<<std::endl<<std::endl;
    }

    Mat33 temp = invertedVar + (sampNumber * invertedError);
    var = temp.inverse();
    mean = var * (sampNumber * invertedError*sampleMean + invertedPriori * mean);

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












