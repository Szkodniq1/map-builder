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
         * | dev_xx dev_xy dev_xz |
         * | dev_yx dev_yy dev_yz |
         * | dev_zx dev_yz dev_zz |
         * --                    --
         */
        Mat33 dev;
        double probability;
        unsigned int sampNumber;
        RGBA color;

        ///default constructor
        inline Voxel(){
            probability = 0;
            sampNumber = 0;
            mean = Vec3(0, 0, 0);
            dev << 0, 0, 0, 0, 0, 0, 0, 0, 0;
            color = RGBA(255, 255, 255);
        }

        //default contructor int OcTree structure
        inline Voxel(int res){
            probability = 0;
            sampNumber = 0;
            mean = Vec3(0, 0, 0);
            dev << 0, 0, 0, 0, 0, 0, 0, 0, 0;
            color = RGBA(255, 255, 255);
        }

        ///constructor
        inline Voxel(double prob, unsigned int samps, Vec3 mean, Mat33 dev, RGBA color) {
            probability = prob;
            sampNumber = samps;
            this->mean = mean;
            this->dev = dev;
            this->color = color;
        }
    };
}

#endif // VOXEL_H
