#include "Voxel/voxel.h"


/*
 * Voxel methods
 */

namespace mapping {

Voxel::Voxel() {
    probability = 0;
    sampNumber = 0;
    mean = Eigen::Vector3d(0, 0, 0);
    var << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    color = RGBA(100, 100, 100, 40);
}

Voxel::Voxel(int res) {
    probability = 0;
    sampNumber = 0;
    mean = Eigen::Vector3d(0, 0, 0);
    var << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    color = RGBA(100, 100, 100, 40);
}

void Voxel::preinitParameters(double res, Eigen::Vector3d center) {
    this->mean = center;
    var << res/10, 0, 0, 0, res/10, 0, 0, 0, res/10;
}

void Voxel::insertPoint(Point3D point, Mat33 uncertaintyError) {
    switch (this->methodType.type) {
    case MethodType::TYPE_SIMPLE:
        this->points.push_back(point);
        break;
    case MethodType::TYPE_BAYES:
        updateBayesDistribution(point, uncertaintyError);
        updateColor(point.color);
        break;
    case MethodType::TYPE_KALMAN:
        updateKalmanDistribution(point, uncertaintyError);
        updateColor(point.color);
        break;
    default:
        this->points.push_back(point);
        break;
    }
    updateOccupancy();
}

void Voxel::updateWithSimpleMethod() {
    mean = Eigen::Vector3d(0, 0, 0);
    var << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    color = RGBA(255, 255, 255, 255);
    updateSimpleDistribution();
    updateSimpleColor();
}

void Voxel::updateSimpleDistribution() {
    for(mapping::Point3D &point : points) {
        Eigen::Vector3d newPoint = Eigen::Vector3d(point.position.x(), point.position.y(), point.position.z());
        mean += newPoint;
    }
    mean = mean / points.size();
    if(points.size() > 1) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                var(i,j) = 0.0;
                for(mapping::Point3D &point : points) {
                    Eigen::Vector3d newPoint = Eigen::Vector3d(point.position.x(), point.position.y(), point.position.z());
                    var(i,j) += (mean(i) - newPoint(i)) * (mean(j) - newPoint(j));
                }
                var(i,j) /= points.size() - 1;
            }
        }
    }
}

void Voxel::updateSimpleColor() {
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


void Voxel::updateBayesDistribution(Point3D point, Mat33 uncertaintyError) {
    Mat33 priorVar;
    priorVar = var;

    if (sampNumber == 0) {
        sampMean = Eigen::Vector3d(point.position.x(), point.position.y(), point.position.z());
    } else {
        sampMean = Eigen::Vector3d(
                    ((sampMean(0) * sampNumber) + point.position.x())/(sampNumber+1),
                    ((sampMean(1) * sampNumber) + point.position.y())/(sampNumber+1),
                    ((sampMean(2) * sampNumber) + point.position.z())/(sampNumber+1));
    }

    Mat33 Inv = uncertaintyError.inverse();
    double det = Inv.determinant();

    if(det == 0)
        return;

    ++sampNumber;

    Mat33 temp;
    temp = var.inverse();
    temp += (sampNumber * Inv);
    var = temp.inverse();
    mean = var * (sampNumber * Inv*sampMean + priorVar.inverse() * mean);
}

void Voxel::updateKalmanDistribution(Point3D point, Mat33 uncertaintyError) {
    /*if (sampNumber == 0) {
        mean = Eigen::Vector3d(point.position.x(), point.position.y(), point.position.z());
        var  = Mat33::Identity();
    } else {
        //Mat33 A = Mat33::Identity();
        //Mat33 At = A.transpose();
        Eigen::Vector3d sampleMean = Eigen::Vector3d(point.position.x(), point.position.y(), point.position.z());

        //Eigen::Vector3d px = A*mean;
        //Mat33 P = A*var*At;

        Mat33 H = Mat33::Identity();
        Mat33 Ht = H.transpose();

        Eigen::Vector3d e = sampleMean - mean;
        Mat33 R = uncertaintyError;
        Mat33 S = var + R;
        Mat33 Sinv = S.inverse();
        Mat33 K = var*Ht*Sinv;
        Mat33 Kt = K.transpose();
        mean = mean + K*e;
        var  = var - K*S*Kt;
    }*/
    Eigen::Vector3d sampleMean = Eigen::Vector3d(point.position.x(), point.position.y(), point.position.z());
    Mat33 temp = (var + uncertaintyError);
    Mat33 tempInv = temp.inverse();
    Mat33 K = var * tempInv;
    mean = mean - K * (mean  - sampleMean);
    var = (Mat33::Identity()-K)*var;
    sampNumber++;
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












