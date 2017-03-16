/** @file octomap.h
 *
 * implementation - Octomap
 *
 */

#ifndef OCTOMAP_H_INCLUDED
#define OCTOMAP_H_INCLUDED

#include "map.h"
#include "octomap/Pointcloud.h"
#include <octomap/octomap.h>
#include <octomap/math/Quaternion.h>
#include <octomap/math/Vector3.h>
#include <octomap/OcTree.h>
#include "../../3rdParty/tinyXML/tinyxml2.h"
#include "Utilities/observer.h"
#include "Utilities/img2pcl.h"
#include <iostream>
#include <memory>

namespace mapping {
    /// create a single Maping object
    Map* createMapOcto(void);
    Map* createMapOcto(PointCloud PC);
}

/// Grabber implementation
class OctoMap : public mapping::Map, public Subject {
public:
    /// Pointer
    typedef std::unique_ptr<OctoMap> Ptr;

    OctoMap(void);
    OctoMap(mapping::PointCloud PC);

    /// Name of the map
    const std::string& getName() const {return name;}

    /// Insert point cloud into map
    void insertCloud(octomap::Pointcloud& pcl);

    /// Insert point cloud into map
    void insertCloud(mapping::PointCloud& PC, octomap::pose6d);

    /// save map in file
    void saveMap();

    /// print map
    void printMap();

    ///Attach visualizer
    void attachVisualizer(QGLVisualizer* visualizer);

    /// descrutor
    ~OctoMap() {}

private:
    const double MAP_RES = 0.1;
    mapping::PointCloud cloud;
    octomap::Pointcloud octoCloud;
    octomap::OcTree map;
};

#endif // OCTOMAP_H_INCLUDED
