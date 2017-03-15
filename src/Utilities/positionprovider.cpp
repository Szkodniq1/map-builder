#include "../../include/Utilities/positionprovider.h"


PositionProvider::PositionProvider() {

}

octomap::point3d PositionProvider::provideOctoPosition() {
    octomap::point3d origin (0,0,0);
    return origin;
}
