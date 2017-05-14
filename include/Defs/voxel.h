#ifndef VOXEL_H
#define VOXEL_H

#include "defs.h"

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
        inline Voxel(){
            probability = 0;
            sampNumber = 0;
            mean = Vec3(0, 0, 0);
            var << 0, 0, 0, 0, 0, 0, 0, 0, 0;
            color = RGBA(255, 255, 255);
        }

        //default contructor int OcTree structure
        inline Voxel(int res){
            probability = 0;
            sampNumber = 0;
            mean = Vec3(0, 0, 0);
            var << 0, 0, 0, 0, 0, 0, 0, 0, 0;
            color = RGBA(255, 255, 255);
        }

        ///constructor
        inline Voxel(double prob, unsigned int samps, Vec3 mean, Mat33 dev, RGBA color) {
            probability = prob;
            sampNumber = samps;
            this->mean = mean;
            this->var = dev;
            this->color = color;

        }

        void update(std::vector<Vec3> measurements, double distance);
        void updateOccupancy();
        void updateDistribution(std::vector<Vec3> measurements);
    };
}

#endif // VOXEL_H
