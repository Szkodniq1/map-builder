#include "../include/Map/gaussmap.h"

/// A single instance of Octomap
Gaussmap::Ptr gaussMap;

Gaussmap::Gaussmap(void) : map(MAP_SIZE) {
    xmin = ymin = zmin = -1 * MAP_SIZE * res/2;
    xmax = ymax = zmax = -1* xmin;
}

Gaussmap::Gaussmap(mapping::PointCloud PC) : map(MAP_SIZE) {
    cloud = PC;
    xmin = ymin = zmin = -1 * MAP_SIZE * res/2;
    xmax = ymax = zmax = -1* xmin;
}

Gaussmap::Gaussmap(mapping::PointCloud PC, float vxmin, float vxmax, float vymin, float vymax, float vzmin, float vzmax) : map(MAP_SIZE) {
    cloud = PC;
    xmin = vxmin; xmax = vxmax;
    ymin = vymin; ymax = vymax;
    zmin = vzmin; zmax = vzmax;
}

/// Insert point cloud into map
void Gaussmap::insertCloud(mapping::GrabbedImage grab, bool isLast) {
    cloud = grab.transformedPointCloud();

    updateMap();
}

/// save map in file
void Gaussmap::saveMap(){

}

///Attach visualizer
void Gaussmap::attachVisualizer(QGLVisualizer* visualizer){
    attach(visualizer);
}

/// print map
void Gaussmap::printMap(){

}

mapping::Map* mapping::createMapGauss(void) {
    gaussMap.reset(new Gaussmap());
    return gaussMap.get();
}

mapping::Map* mapping::createMapGauss(PointCloud PC) {
    gaussMap.reset(new Gaussmap(PC));
    return gaussMap.get();
}


void Gaussmap::updateMap() {
    int xCoor, yCoor, zCoor;
    for(mapping::Point3D &point : cloud) {
        xCoor = xCoordinate(point.position.x());
        yCoor = yCoordinate(point.position.y());
        zCoor = zCoordinate(point.position.z());

        updateVoxel(xCoor, yCoor, zCoor, point);
    }
}



int Gaussmap::xCoordinate(double x) {
    double  a = MAP_SIZE/(xmax-xmin);
    double b = -1 * a*xmin;
    return a*x + b;
}

int Gaussmap::yCoordinate(double y) {
    double  a = MAP_SIZE/(ymax-ymin);
    double b = -1 * a*ymin;
    return a*y + b;
}

int Gaussmap::zCoordinate(double z) {
    double  a = MAP_SIZE/(zmax-zmin);
    double b = -1 * a*zmin;
    return a*z + b;
}






















