#ifndef VOXEL_H
#define VOXEL_H

#include "Defs/defs.h"
#include "occmethodtype.h"
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
        /// Method type
        OccMethodType methodType;
        //For simple method
        PointCloud points;

        ///default constructor
        Voxel();
        //default contructor int OcTree structure
        Voxel(int res);

        //default initializers for types
        void defaultSimpleInit();
        void defaultBayesInit();

        void insertPoint(Point3D point, Mat33 uncertaintyError);

        void updateWithSimpleMethod();
        void updateSimpleDistribution();
        void updateSimpleColor();

        void updateBayesDistribution(Point3D point, Mat33 uncertaintyError);
        void updateBayesColor(RGBA color);

        void updateOccupancy();
        void updateNullOccupancy();
    };




}

#endif // VOXEL_H
