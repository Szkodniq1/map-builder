/** @file point.h
 *
 * Point Cloud Grabber interface
 *
 */

#ifndef _POINT_H_
#define _POINT_H_

#include "../Defs/defs.h"
#include <iostream>
#include <string>
#include <vector>
#include <mutex>

/// Point interface

class Point {
public:

    /// Point type
    enum Type {
        /// Octomap point
        TYPE_OCTOPOINT,
        /// Own point
        TYPE_PROB,
        /// PCL
        TYPE_PCL
    };

    /// overloaded constructor
    Point(const std::string _name, Type _type) : name(_name), type(_type) {}

    /// Name of the point
    virtual const std::string& getName() const = 0;

    /// Converts point from given type
    virtual void convertToType(Point& point) const = 0;

    /// Get converted Point
    virtual const Point& getPoint() = 0;

    /// Virtual descrutor
    virtual ~Point() {}

protected:
    /// Point type
    Type type;

    /// Point name
    const std::string name;
};


#endif // _POINT_H_
