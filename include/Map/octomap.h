/** @file octomap.h
 *
 * implementation - Octomap
 *
 */

#ifndef OCTOMAP_H_INCLUDED
#define OCTOMAP_H_INCLUDED

#include "map.h"
#include "../../3rdParty/tinyXML/tinyxml2.h"
#include "Utilities/observer.h"
#include <iostream>
#include <memory>

namespace mapping {
    /// create a single Maping object
    Map* createMapOcto(void);
}

/// Grabber implementation
class OctoMap : public mapping::Map, public Subject {
public:
    /// Pointer
    typedef std::unique_ptr<OctoMap> Ptr;

    OctoMap(void);

    /// Name of the map
    const std::string& getName() const {return name;}

    /// Insert point cloud into map
    void insertCloud(void);

    /// save map in file
    void saveMap();

    /// print map
    void printMap();

    ///Attach visualizer
    void attachVisualizer(QGLVisualizer* visualizer);

    /// descrutor
    ~OctoMap() {}

private:
    mapping::PointCloud cloud;
};

#endif // OCTOMAP_H_INCLUDED
