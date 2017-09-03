/** @file map.h
 *
 * Map interface
 *
 */

#ifndef _MAP_H_
#define _MAP_H_

#include "../Defs/defs.h"
#include "grabbedimage.h"
#include "octomap/Pointcloud.h"
#include "Visualizer/Qvisualizer.h"
#include <iostream>
#include <string>
#include <vector>
#include <mutex>

/// Map interface
namespace mapping {
    class Map {
    public:

        /// Map type
        enum Type {
            /// RGB camera
            TYPE_OCTOMAP,
            /// 2D Depth sensor
            TYPE_PROB,
        };

        ///constructor
        Map(void) : name("unknown"){}

        /// overloaded constructor
        Map(const std::string _name, Type _type) : name(_name), type(_type) {}

        /// Name of the map
        virtual const std::string& getName() const = 0;

        /// Insert point cloud into map
        virtual void insertCloud(mapping::GrabbedImage grab, bool isLast) = 0;

        /// save map in file
        virtual void saveMap() = 0;

        /// print map
        virtual void printMap() = 0;

        ///Attach visualizer
        virtual void attachVisualizer(QGLVisualizer* visualizer) = 0;

        virtual void mapLoaded() = 0;

        /// Virtual descrutor
        virtual ~Map() {}

    protected:
        /// Map type
        Type type;

        /// Map name
        const std::string name;

    };
}


#endif // _MAP_H_
