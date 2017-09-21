#ifndef _GAUSSMAP_H
#define _GAUSSMAP_H
#include "main.h"
#include "Defs/defs.h"
#include "Voxel/voxel.h"
#include "3rdParty/octree/octree.h"
#include "map.h"
#include "math.h"
#include <unordered_map>

namespace mapping {
    /// create a single Maping object
    Map* createMapGauss(void);
    Map* createMapGauss(PointCloud PC);
    Map* createMapGauss(std::string mapPath);
}


class Gaussmap : public mapping::Map, public Subject {
private:
    /// Method type
    OccMethodType methodType;
    Octree<mapping::Voxel> map;
    PointCloud cloud;
    std::vector<Mat33> uncertinatyErrors;
    Eigen::Vector3d cameraPos;

    std::unordered_map<std::string, Eigen::Vector3i> indexes;
    std::unordered_map<std::string, Eigen::Vector3i> simpleMethodIndexes;

    //Map borders
    double xmin, xmax;
    double ymin, ymax;
    double zmin, zmax;

    int prevX,prevY,prevZ = -1;

    void preinitVoxels();
    void raytracePoint(mapping::Point3D point, int x, int y, int z);


protected:
    //Protected functions
    void updateMap(bool isLast);
    double normalize(double p, double min);
    int xCoordinate(double x);
    int yCoordinate(double y);
    int zCoordinate(double z);
    double backwardXCoordinate(int x);
    double backwardYCoordinate(int y);
    double backwardZCoordinate(int z);

public:
    /// Pointer
    typedef std::unique_ptr<Gaussmap> Ptr;

    Gaussmap(void);
    Gaussmap(mapping::PointCloud PC);
    Gaussmap(std::string mapPath);
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

    void mapLoaded();

    /// descrutor
    ~Gaussmap() {}

    std::string currentDateTime();

    std::vector<std::string> split(const std::string &s, char delim);
};

#endif //_GAUSSMAP_H
