#ifndef _GAUSSMAP_H
#define _GAUSSMAP_H

#include "Defs/defs.h"
#include "3rdParty/octree/octree.h"
#include "map.h"


namespace  mapping {

class Gaussmap : public mapping::Map {
private:
    Octree<Voxel> map;
    const double res;
    PointCloud cloud;

public:
};

}



#endif //_GAUSSMAP_H
