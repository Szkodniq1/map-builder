#ifndef VOXEL_H
#define VOXEL_H

#include "Defs/defs.h"
#include "occmethodtype.h"
#include "../../3rdParty/Eigen/Dense"
#include <tuple>

namespace mapping {
    class Voxel {
    public:
        // [ mean_x mean_y mean_z]
        Eigen::Vector3d mean;
        /*
         * --                    --
         * | var_xx var_xy var_xz |
         * | var_yx var_yy var_yz |
         * | var_zx var_yz var_zz |
         * --                    --
         */
        Mat33 var;
        int probability;
        unsigned int sampNumber;
        Eigen::Vector3d sampMean;
        RGBA color;
        /// Method type

        //For simple method
        PointCloud points;
        std::vector<Mat33> uncertaintyErrors;

        Eigen::MatrixXd P_pre;

        ///default constructor
        Voxel();
        //default contructor int OcTree structure
        Voxel(int res);

        void preinitParameters(double res, Eigen::Vector3d center);

        void insertPoint(Point3D point, Mat33 uncertaintyError);

        void updateWithSimpleMethod();
        void updateSimpleDistribution();
        void updateSimpleColor();

        void updateNaiveDistribution();
        Mat33 prostuj(Mat33 R);
        std::tuple<Mat33, Eigen::Vector3d> changeOrder(Mat33 Rot, Eigen::Vector3d S);
        Eigen::Vector3d castVector(Mat33 Rot, Eigen::Vector3d S);
        Mat33 expmap(const Vec3& omega);
        Eigen::Vector3d logmap(const Mat33& R);
        Mat33 skewSymetric(const Vec3& omega);
        Eigen::Vector3d invSkewSymetric(const Mat33& R);

        void updateNaiveColor();

        void updateBayesDistribution(Point3D point, Mat33 uncertaintyError);

        void updateKalmanDistribution(Point3D point, Mat33 uncertaintyError);

        void updateColor(RGBA color);

        void updateOccupancy();
        void updateNullOccupancy();
    };




}

#endif // VOXEL_H
