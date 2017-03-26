#ifndef _GAUSSMAP_H
#define _GAUSSMAP_H

#include "Defs/defs.h"
#include "3rdParty/octree/octree.h"
#include "map.h"


namespace mapping {
    /// create a single Maping object
    Map* createMapGauss(void);
    Map* createMapGauss(PointCloud PC);
}

class Gaussmap : public mapping::Map, public Subject {
private:
    Octree<mapping::Voxel> map;
    const double res = 0.1;
    PointCloud cloud;

public:
    /// Pointer
    typedef std::unique_ptr<Gaussmap> Ptr;

    Gaussmap(void);
    Gaussmap(mapping::PointCloud PC);

    /// Name of the map
    const std::string& getName() const {return name;}

    /// Insert point cloud into map
    void insertCloud(mapping::GrabbedImage grab);

    /// save map in file
    void saveMap();

    /// print map
    void printMap();

    ///Attach visualizer
    void attachVisualizer(QGLVisualizer* visualizer);

    /// descrutor
    ~Gaussmap() {}
};

#endif //_GAUSSMAP_H
