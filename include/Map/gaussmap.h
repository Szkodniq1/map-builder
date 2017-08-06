#ifndef _GAUSSMAP_H
#define _GAUSSMAP_H

#include "Defs/defs.h"
#include "Defs/voxel.h"
#include "3rdParty/octree/octree.h"
#include "map.h"
#include "math.h"
#include <unordered_map>

#define MAP_SIZE 128

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
    std::vector<Mat33> uncertinatyErrors;

    std::unordered_map<std::string, Eigen::Vector3i> indexes;

    //Map borders
    double xmin, xmax;
    double ymin, ymax;
    double zmin, zmax;


protected:
    //Protected functions
    void updateMap(bool isLast);
    double normalize(double p, double min);
    int xCoordinate(double x);
    int yCoordinate(double y);
    int zCoordinate(double z);

public:
    /// Pointer
    typedef std::unique_ptr<Gaussmap> Ptr;

    Gaussmap(void);
    Gaussmap(mapping::PointCloud PC);
    Gaussmap(mapping::PointCloud PC, float vxmin, float vxmax, float vymin, float vymax, float vzmin, float vzmax);

    /// Name of the map
    const std::string& getName() const {return name;}

    /// Insert point cloud into map
    void insertCloud(mapping::GrabbedImage grab, bool isLast);

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
