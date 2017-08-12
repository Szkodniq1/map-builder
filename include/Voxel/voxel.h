#ifndef VOXEL_H
#define VOXEL_H

#include "Defs/defs.h"
#include "../../3rdParty/Eigen/Dense"


namespace mapping {
    class Voxel {
    public:
        /// Voxel type
        enum Type {
            /// For simple method
            TYPE_SIMPLE,
            /// For bayesian update
            TYPE_BAYES,
            /// For kalman filter implementation
            TYPE_KALMAN
        };
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
        RGBA color;
        /// Voxel type
        Type type;

        ///default constructor
        Voxel();
        //default contructor int OcTree structure
        Voxel(int res);
        ///constructor
        Voxel(double prob, unsigned int samps, Eigen::Vector3d mean, mapping::Mat33 dev, mapping::RGBA color);

        void insertPoint(Point3D point, Mat33 uncertaintyError);
        void updateOccupancy();
        void updateDistribution(Point3D point, Mat33 uncertaintyError);
        void updateColor(RGBA color);
        void updateNullOccupancy();
    };




}

#endif // VOXEL_H
