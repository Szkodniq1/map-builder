#ifndef VOXEL_H
#define VOXEL_H

#include "Defs/defs.h"
#include "../../3rdParty/Eigen/Dense"


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
        RGBA color;
        PointCloud points;

        ///default constructor
        Voxel();

        //default contructor int OcTree structure
        Voxel(int res);

        ///constructor
        Voxel(double prob, unsigned int samps, Eigen::Vector3d mean, Mat33 dev, RGBA color);

        void insertPoint(Point3D point);
        void update();
        void updateOccupancy();
        void updateDistribution();
        void updateColor();
        void updateNullOccupancy();
    };




}

#endif // VOXEL_H
