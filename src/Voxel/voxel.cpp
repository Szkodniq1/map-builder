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
    sampMean = Eigen::Vector3d(0, 0, 0);
    color = RGBA(80, 80, 80, 80);
    points = {};
    for(int i =0; i<9;i++) {
        for(int j =0; j<9;j++) {
            if(i==j) {
                P_values[9*i+j] = 1;
            }
        }
    }
}

Voxel::Voxel(int res) {
    probability = 0;
    sampNumber = 0;
    mean = Eigen::Vector3d(0, 0, 0);
    var << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    sampMean = Eigen::Vector3d(0, 0, 0);
    color = RGBA(80, 80, 80, 80);
    points = {};
    for(int i =0; i<9;i++) {
        for(int j =0; j<9;j++) {
            if(i==j) {
                P_values[9*i+j] = 1;
            }
        }
    }
}

void Voxel::preinitParameters(double res, Eigen::Vector3d center) {
    this->mean = center;
    var << res/10, 0, 0, 0, res/10, 0, 0, 0, res/10;
}

void Voxel::insertPoint(Point3D point, Mat33 uncertaintyError) {
    OccMethodType methodType;
    switch (methodType.type) {
    case MethodType::TYPE_SIMPLE:
        this->points.push_back(point);
        break;
    case MethodType::TYPE_BAYES:
        this->points.push_back(point);
        //this->uncertaintyErrors.push_back(uncertaintyError);
        break;
    case MethodType::TYPE_KALMAN:
        this->points.push_back(point);
        //this->uncertaintyErrors.push_back(uncertaintyError);
        break;
    case MethodType::TYPE_NDTOM:
        this->points.push_back(point);
        //this->uncertaintyErrors.push_back(uncertaintyError);
        break;
    default:
        this->points.push_back(point);
        break;
    }
    updateOccupancy();
}

void Voxel::updateWithSimpleMethod() {
    OccMethodType methodType;
    if (methodType.type == MethodType::TYPE_SIMPLE) {
        mean = Eigen::Vector3d(0, 0, 0);
        var << 0, 0, 0, 0, 0, 0, 0, 0, 0;
        color = RGBA(255, 255, 255, 255);
        updateSimpleDistribution();
        updateSimpleColor();
    } else if (methodType.type == MethodType::TYPE_BAYES) {
        updateBayesDistribution();
        updateNaiveColor();
        points.clear();
        //uncertaintyErrors.clear();
    } else if (methodType.type == MethodType::TYPE_KALMAN) {
        updateKalmanDistribution();
        updateNaiveColor();
        points.clear();
        //uncertaintyErrors.clear();
    } else if (methodType.type == MethodType::TYPE_NDTOM) {
        updateNDTOM();
        updateNaiveColor();
        points.clear();
    }
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

Mat33 Voxel::prostuj(Mat33 R) {
    Eigen::Vector3d tmp = R.col(2).cross(R.col(0));
    double n = tmp.cross(R.col(1)).norm();
    double d = tmp.dot(R.col(1));
    double a = std::atan2(n, d) * 180/M_PI;
    double absNorm = std::abs(a);

    if(absNorm > 90) {
        R.col(1) = -R.col(1);
    }

    if (R(0,0) < 0 && R(1,1) < 0 && R(2,2) < 0)
        R = -R;

    if (R(0,0) < 0 && R(1,1) < 0) {
        R.col(0) = -R.col(0);
        R.col(1) = -R.col(1);
    }

    if (R(0,0) < 0 && R(2,2) < 0) {
        R.col(0) = -R.col(0);
        R.col(2) = -R.col(2);
    }

    if (R(2,2) < 0 && R(1,1) < 0) {
        R.col(1) = -R.col(1);
        R.col(2) = -R.col(2);
    }

    return R;
}

std::tuple<Mat33, Eigen::Vector3d> Voxel::changeOrder(Mat33 Rot, Eigen::Vector3d S) {
    Mat33 I = Mat33::Identity();
    Mat33 newRot = Mat33::Zero();
    Eigen::Vector3d newS(0,0,0);
    Eigen::Vector3i index(-1,-1,-1);

    for(int i = 0; i < 3; i++) {
        double angle = 190;
        for(int j = 0; j < 3; j++) {
            double n = I.col(j).cross(Rot.col(i)).norm();
            double d = I.col(j).dot(Rot.col(i));
            double a = std::atan2(n, d) * 180/M_PI;

            if(std::norm(a) < angle || std::norm(a-180) < angle) {

                if (index(0) != j && index(1) != j &&  index(2) != j ) {

                    if (std::norm(a) < std::norm(a-180))
                        angle = std::norm(a);
                    else
                        angle = std::norm(a-180);
                    index(i) = j;
                    newRot.col(j) = Rot.col(i);
                    newS(j) = S(i);
                }

            }
        }
    }

    Eigen::Vector3d tmp = newRot.col(2).cross(newRot.col(0));
    double n = tmp.cross(newRot.col(1)).norm();
    double d = tmp.dot(newRot.col(1));
    double a = std::atan2(n, d) * 180/M_PI;

    if(std::norm(a) > 90)
        newRot.col(1) = -newRot.col(1);

    return  std::make_tuple(newRot, newS);
}

Eigen::Vector3d Voxel::castVector(Mat33 Rot, Eigen::Vector3d S) {
    Eigen::Vector3d newS;
    newS << 0,0,0;
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            if (std::norm(newS(i)) < std::norm(Rot(i,j)*S(j)))
                newS(i) = std::norm(Rot(i,j)*S(j));
        }
    }

    return newS;
}

Mat33 Voxel::expmap(const Vec3& omega) {
    double theta =sqrt(pow(omega.x(),2.0)+pow(omega.y(),2.0)+pow(omega.z(),2.0));
    Mat33 omegaRot(skewSymetric(omega));
    if (theta<1e-5)
        return Mat33::Identity()+(1+(pow(theta,2.0)/6.0)+(pow(theta,4.0)/120.0))*omegaRot+(0.5-(pow(theta,2.0)/24.0)+(pow(theta,4.0)/720.0))*(omegaRot*omegaRot);
    else
        return Mat33::Identity()+(sin(theta)/theta)*omegaRot+((1-cos(theta))/pow(theta,2.0))*(omegaRot*omegaRot);
}

Eigen::Vector3d Voxel::logmap(const Mat33& R) {
    double trace = (R.trace()-1)/2;
    if(trace > 1.0) {
        trace = 1.0;
    }
    if(trace < -1.0) {
        trace = -1.0;
    }
    double theta = acos(trace);
    double coeff=1;
    if (theta<1e-5)
        coeff=1.0;
    else
        coeff = (theta/(2*sin(theta)));
    Mat33 lnR = coeff*(R-R.transpose());
    return invSkewSymetric(lnR);
}

Mat33 Voxel::skewSymetric(const Vec3& omega) {
    Mat33 R;
    R<< 0, -omega.z(), omega.y(),
            omega.z(), 0, -omega.x(),
            -omega.y(), omega.x(), 0;
    return R;
}

Eigen::Vector3d Voxel::invSkewSymetric(const Mat33& R) {
    return Eigen::Vector3d(R(2,1), R(0,2), R(1,0));
}

void Voxel::updateNaiveColor() {
    long r,g,b,a;
    int previousSampNumber = (sampNumber - points.size());
    r = color.r * previousSampNumber;
    g = color.g * previousSampNumber;
    b = color.b * previousSampNumber;
    a = color.a * previousSampNumber;
    for(mapping::Point3D &point : points) {
        r += point.color.r;
        g += point.color.g;
        b += point.color.b;
        a += point.color.a;
    }

    this->color.r = (int) r/sampNumber;
    this->color.g = (int) g/sampNumber;
    this->color.b = (int) b/sampNumber;
    this->color.a = (int) a/sampNumber;
}


void Voxel::updateBayesDistribution() {

    if (sampNumber == 0) {
        mean = Eigen::Vector3d(0, 0, 0);
        var << 0, 0, 0, 0, 0, 0, 0, 0, 0;
        color = RGBA(255, 255, 255, 255);
        sampNumber = points.size();
        if (sampNumber == 1) {
            mean = Eigen::Vector3d(points[0].position.x(), points[0].position.y(), points[0].position.z());
            //var = uncertaintyErrors[0];
        } else {
            updateSimpleDistribution();
        }
    } else {
        Eigen::Vector3d newMean = Eigen::Vector3d(0, 0, 0);
        for(mapping::Point3D &point : points) {
            Eigen::Vector3d newPoint = Eigen::Vector3d(point.position.x(), point.position.y(), point.position.z());
            newMean += newPoint;
        }
        newMean = newMean / points.size();

        Mat33 newVar;
        newVar << 0, 0, 0, 0, 0, 0, 0, 0, 0;
        if(points.size() > 1) {
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    newVar(i,j) = 0.0;
                    for(mapping::Point3D &point : points) {
                        Eigen::Vector3d newPoint = Eigen::Vector3d(point.position.x(), point.position.y(), point.position.z());
                        newVar(i,j) += (newMean(i) - newPoint(i)) * (newMean(j) - newPoint(j));
                    }
                    newVar(i,j) /= points.size() - 1;
                }
            }
        }

        Eigen::JacobiSVD<Mat33> svdVar(var, Eigen::ComputeFullU);
        Eigen::JacobiSVD<Mat33> svdNewVar(newVar, Eigen::ComputeFullU);
        Mat33 U = svdVar.matrixU();
        Mat33 Un = svdNewVar.matrixU();
        Eigen::Vector3d S = svdVar.singularValues();
        Eigen::Vector3d Sn = svdNewVar.singularValues();

        U = prostuj(U);
        Un = prostuj(Un);
        Mat33 U0 = U.inverse()*U;
        Mat33 Un0 = U.inverse()*Un;
        Un0 = prostuj(Un0);

        Eigen::Vector3d u = logmap(U0);
        if(u(2)<0)
            u = u - u/(u.norm()*2*M_PI);
        Eigen::VectorXd x0(9);
        x0 << mean(0), mean(1), mean(2), S(0), S(1), S(2), u(0), u(1), u(2);

        Eigen::Vector3d un = logmap(Un0);
        if(un(2)<0)
            un = un - un/(un.norm()*2*M_PI);
        Eigen::VectorXd x0n(9);
        x0n << newMean,Sn, un;
        x0n << newMean(0), newMean(1), newMean(2), Sn(0), Sn(1), Sn(2), un(0), un(1), un(2);

        Eigen::VectorXd xxx(9);

        xxx = x0 * sampNumber / (sampNumber + points.size()) + x0n * points.size() / (sampNumber + points.size());

        Eigen::MatrixXd c = Eigen::MatrixXd::Identity(9, 9);
        sampNumber += points.size();
        Eigen::VectorXd ux(9);
        ux = xxx;


        Eigen::VectorXd mp(9);
        Eigen::MatrixXd cp = Eigen::MatrixXd::Identity(9, 9);
        Eigen::MatrixXd P_pre(9,9);
        for(int i =0; i<9;i++) {
            for(int j =0; j<9;j++) {
                P_pre(i,j) = P_values[9*i+j];
            }
        }
        cp = (P_pre.inverse() + sampNumber*(c.inverse())).inverse();
        mp = cp*(sampNumber*(c.inverse())*ux + (P_pre.inverse())*x0);
        for(int i =0; i<9;i++) {
            for(int j =0; j<9;j++) {
                P_values[9*i+j] = cp(i,j);
            }
        }

        mean << mp(0), mp(1), mp(2);
        Eigen::Vector3d post_s;
        post_s << mp(3),mp(4),mp(5);
        Eigen::Vector3d post_r;
        post_r << mp(6),mp(7),mp(8);

        Mat33 postS;
        postS << post_s(0), 0, 0, 0,  post_s(1), 0, 0, 0, post_s(2);
        Mat33 postR;
        postR = expmap(Vec3(post_r(0), post_r(1), post_r(2)));
        postR = U*postR;
        var = postR * postS * postR.transpose();

    }
}

void Voxel::updateKalmanDistribution() {
    if (sampNumber == 0) {
        mean = Eigen::Vector3d(0, 0, 0);
        var << 0, 0, 0, 0, 0, 0, 0, 0, 0;
        color = RGBA(255, 255, 255, 255);
        sampNumber = points.size();
        if (sampNumber == 1) {
            mean = Eigen::Vector3d(points[0].position.x(), points[0].position.y(), points[0].position.z());
            //var = uncertaintyErrors[0];
        } else {
            updateSimpleDistribution();
        }
    } else {
        Eigen::Vector3d newMean = Eigen::Vector3d(0, 0, 0);
        for(mapping::Point3D &point : points) {
            Eigen::Vector3d newPoint = Eigen::Vector3d(point.position.x(), point.position.y(), point.position.z());
            newMean += newPoint;
        }
        newMean = newMean / points.size();

        Mat33 newVar;
        newVar << 0, 0, 0, 0, 0, 0, 0, 0, 0;
        if(points.size() > 1) {
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    newVar(i,j) = 0.0;
                    for(mapping::Point3D &point : points) {
                        Eigen::Vector3d newPoint = Eigen::Vector3d(point.position.x(), point.position.y(), point.position.z());
                        newVar(i,j) += (newMean(i) - newPoint(i)) * (newMean(j) - newPoint(j));
                    }
                    newVar(i,j) /= points.size() - 1;
                }
            }
        }

        Eigen::JacobiSVD<Mat33> svdVar(var, Eigen::ComputeFullU);
        Eigen::JacobiSVD<Mat33> svdNewVar(newVar, Eigen::ComputeFullU);
        Mat33 U = svdVar.matrixU();
        Mat33 Un = svdNewVar.matrixU();
        Eigen::Vector3d S = svdVar.singularValues();
        Eigen::Vector3d Sn = svdNewVar.singularValues();

        U = prostuj(U);
        Un = prostuj(Un);
        Mat33 U0 = U.inverse()*U;
        Mat33 Un0 = U.inverse()*Un;
        Un0 = prostuj(Un0);

        Eigen::Vector3d u = logmap(U0);
        if(u(2)<0)
            u = u - u/(u.norm()*2*M_PI);
        Eigen::VectorXd x0(9);
        x0 << mean(0), mean(1), mean(2), S(0), S(1), S(2), u(0), u(1), u(2);

        Eigen::Vector3d un = logmap(Un0);
        if(un(2)<0)
            un = un - un/(un.norm()*2*M_PI);
        Eigen::VectorXd x0n(9);
        x0n << newMean,Sn, un;
        x0n << newMean(0), newMean(1), newMean(2), Sn(0), Sn(1), Sn(2), un(0), un(1), un(2);

        Eigen::VectorXd xxx(9);

        xxx = x0 * sampNumber / (sampNumber + points.size()) + x0n * points.size() / (sampNumber + points.size());

        /// Krok predykcji
        Eigen::MatrixXd A(9,9);
        A = Eigen::MatrixXd::Identity(9,9);
        Eigen::MatrixXd B(9,9);
        B = Eigen::MatrixXd::Zero(9,9);
        Eigen::VectorXd us(9);
        us << 0,0,0,0,0,0,0,0,0;
        Eigen::MatrixXd C(9,9);
        C = Eigen::MatrixXd::Identity(9,9);

        Eigen::VectorXd xp(9);
        xp = A*x0 + B*us;
        Eigen::MatrixXd P(9,9);
        Eigen::MatrixXd P_pre(9,9);
        for(int i =0; i<9;i++) {
            for(int j =0; j<9;j++) {
                P_pre(i,j) = P_values[9*i+j];
            }
        }
        P = A*P_pre*A.transpose();
        Eigen::VectorXd y(9);
        y = x0n;

        Eigen::VectorXd yPred(9);
        yPred << 0,0,0,0,0,0,0,0,0;

        Eigen::MatrixXd R(9,9);
        R = Eigen::MatrixXd::Identity(9,9);
        Eigen::VectorXd e(9);
        e = y - C*xp;

        Eigen::MatrixXd SS(9,9);
        SS = C*P*C.transpose() + R;
        Eigen::MatrixXd K(9,9);
        K = P*C.transpose()*SS.inverse();

        Eigen::VectorXd postX(9);
        postX = xp + K*e;
        P_pre = P - K*SS*K.transpose();
        for(int i =0; i<9;i++) {
            for(int j =0; j<9;j++) {
                P_values[9*i+j] = P_pre(i,j);
            }
        }
        xp = postX;

        sampNumber += points.size();

        mean << xp(0), xp(1), xp(2);
        Eigen::Vector3d post_s;
        post_s << xp(3),xp(4),xp(5);
        Eigen::Vector3d post_r;
        post_r << xp(6),xp(7),xp(8);

        Mat33 postS;
        postS << post_s(0), 0, 0, 0,  post_s(1), 0, 0, 0, post_s(2);
        Mat33 postR;
        postR = expmap(Vec3(post_r(0), post_r(1), post_r(2)));
        postR = U*postR;
        var = postR * postS * postR.transpose();
    }
}

void Voxel::updateNDTOM() {
    if (sampNumber == 0) {
        Eigen::Vector3d newMean = Eigen::Vector3d(0, 0, 0);
        for(mapping::Point3D &point : points) {
            Eigen::Vector3d newPoint = Eigen::Vector3d(point.position.x(), point.position.y(), point.position.z());
            newMean += newPoint;
        }
        meanSum = newMean;
        mean = meanSum / points.size();

        Mat33 newVar;
        newVar << 0, 0, 0, 0, 0, 0, 0, 0, 0;
        if(points.size() > 1) {

            for(mapping::Point3D &point : points) {
                Eigen::Vector3d newPoint = Eigen::Vector3d(point.position.x(), point.position.y(), point.position.z());
                newVar += (newPoint - mean) * (newPoint - mean).transpose();
            }

        }
        varSum = newVar;
        var = varSum / (points.size() - 1);
        sampNumber = points.size();
    } else {
        Eigen::Vector3d newMeanSum = Eigen::Vector3d(0, 0, 0);
        for(mapping::Point3D &point : points) {
            Eigen::Vector3d newPoint = Eigen::Vector3d(point.position.x(), point.position.y(), point.position.z());
            newMeanSum += newPoint;
        }

        Eigen::Vector3d newMean = newMeanSum / points.size();

        Mat33 newVarSum;
        newVarSum << 0, 0, 0, 0, 0, 0, 0, 0, 0;
        if(points.size() > 1) {
            for(mapping::Point3D &point : points) {
                Eigen::Vector3d newPoint = Eigen::Vector3d(point.position.x(), point.position.y(), point.position.z());
                newVarSum += (newPoint - newMean) * (newPoint - newMean).transpose();
            }
        }

        varSum= varSum + newVarSum + (sampNumber/(points.size()*(points.size() + sampNumber)))*((points.size()/sampNumber)*meanSum - newMeanSum)*((points.size()/sampNumber)*meanSum - newMeanSum).transpose();
        meanSum += newMeanSum;
        mean = meanSum / (sampNumber + points.size());
        var = varSum / (sampNumber + points.size() - 1);
        sampNumber += points.size();
    }
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












