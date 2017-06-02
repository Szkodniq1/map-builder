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
    uncertinatyErrors = grab.uncertinatyErrors;
    int64 e1 = cv::getTickCount();
    updateMap(isLast);
    int64 e2 = cv::getTickCount();
    double time = ((e2 - e1)/ cv::getTickFrequency());
    std::cout<<"Gaussmap insert time and update: "<<time<<std::endl;
    //notify(grab.transformedPointCloud(),grab.uncertinatyErrors, isLast);
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


void Gaussmap::updateMap(bool isLast) {
    int xCoor, yCoor, zCoor;
    int i = 0;
    for(mapping::Point3D &point : cloud) {
        xCoor = xCoordinate(point.position.x());
        yCoor = yCoordinate(point.position.y());
        zCoor = zCoordinate(point.position.z());

        std::string key = std::to_string(xCoor) + std::to_string(yCoor) + std::to_string(zCoor);

        std::unordered_map<std::string, Eigen::Vector3i>::iterator got = indexes.find(key);

        if(got == indexes.end()) {
            indexes[key] = Eigen::Vector3i(xCoor, yCoor, zCoor);
        }

        map(xCoor, yCoor, zCoor).update(point, uncertinatyErrors[i], xCoor==50&&yCoor==29&&zCoor==26);
        i++;
    }
    notify(map, res, indexes, isLast);
}

double Gaussmap::normalize(double p, double min) {
    return (fmod((p - min), res) * 2/res) - 1.0;
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






















