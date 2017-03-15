#ifndef POSITIONPROVIDER_H
#define POSITIONPROVIDER_H

#include "../../3rdParty/tinyXML/tinyxml2.h"
#include <iostream>
#include <memory>
#include <vector>
#include "../../3rdParty/Eigen/Geometry"
#include <octomap/octomap.h>


class PositionProvider {
public:
    /// Pointer
    typedef std::unique_ptr<PositionProvider> Ptr;

    PositionProvider(void);
    octomap::point3d provideOctoPosition();
private:

};


#endif // POSITIONPROVIDER_H
