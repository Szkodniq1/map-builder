#ifndef VOXEL_H
#define VOXEL_H

#include "Defs/defs.h"
#include "../../3rdParty/Eigen/Dense"


namespace mapping {
    class Voxel {
    public:
        // [ mean_x mean_y mean_z]
        Vec3 mean;
        /*
         * --                    --
         * | var_xx var_xy var_xz |
         * | var_yx var_yy var_yz |
         * | var_zx var_yz var_zz |
         * --                    --
         */
        Mat33 var;
        double probability;
        unsigned int sampNumber;
        RGBA color;

        ///default constructor
        Voxel();

        //default contructor int OcTree structure
        Voxel(int res);

        ///constructor
        Voxel(double prob, unsigned int samps, Vec3 mean, Mat33 dev, RGBA color);

        void update(std::vector<Vec3> measurements, double distance);
        void updateOccupancy();
        void updateDistribution(std::vector<Vec3> measurements);
    };




}

#endif // VOXEL_H
