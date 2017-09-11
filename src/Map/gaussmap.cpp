#include "../include/Map/gaussmap.h"

/// A single instance of Octomap
Gaussmap::Ptr gaussMap;

Gaussmap::Gaussmap(void) : map(MAP_SIZE) {
    xmin = ymin = zmin = -1 * MAP_SIZE * res/2;
    xmax = ymax = zmax = -1* xmin;
    //preinitVoxels();
}

bool exists_test1 (const std::string& name) {
    if (FILE *file = fopen(name.c_str(), "r")) {
        fclose(file);
        return true;
    } else {
        return false;
    }
}

Gaussmap::Gaussmap(std::string mapPath) : map(MAP_SIZE)  {
    if(exists_test1(mapPath)) {
        std::cout<<"File found"<<std::endl;
    } else {
        std::cout<<"Map file not found"<<std::endl;
    }
    std::ifstream mapfile;
    mapfile.open(mapPath.c_str(), std::ios_base::binary);
    map.readBinary(mapfile);
    mapfile.close();
    xmin = ymin = zmin = -1 * MAP_SIZE * res/2;
    xmax = ymax = zmax = -1* xmin;
}

Gaussmap::Gaussmap(mapping::PointCloud PC) : map(MAP_SIZE) {
    cloud = PC;
    xmin = ymin = zmin = -1 * MAP_SIZE * res/2;
    xmax = ymax = zmax = -1* xmin;
    //preinitVoxels();
}

Gaussmap::Gaussmap(mapping::PointCloud PC, float vxmin, float vxmax, float vymin, float vymax, float vzmin, float vzmax) : map(MAP_SIZE) {
    cloud = PC;
    xmin = vxmin; xmax = vxmax;
    ymin = vymin; ymax = vymax;
    zmin = vzmin; zmax = vzmax;
    //preinitVoxels();
}

void Gaussmap::mapLoaded() {
    notify(map, res, indexes, true);
}

void Gaussmap::preinitVoxels() {
    for(int i = 0 ; i < map.size();  i++) {
        for(int j = 0 ; j < map.size();  j++) {
            for(int k = 0 ; k < map.size();  k++) {
                map(i, j, k).preinitParameters(res, Eigen::Vector3d(backwardXCoordinate(i), backwardYCoordinate(j), backwardZCoordinate(k)));
            }
        }
    }
}

/// Insert point cloud into map
void Gaussmap::insertCloud(mapping::GrabbedImage grab, bool isLast) {
    cloud = grab.transformedPointCloud();
    cameraPos = grab.cameraPos;
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
//    std::cout<<"Map read parameters: "<<map.size()<<", "<<map.bytes()<<std::endl;
//    std::ofstream mapfile;
//    std::string path = "asd";
//    mapfile.open(path.c_str(), std::ios_base::binary);
//    map.writeBinary(mapfile);
//    mapfile.close();
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

mapping::Map* mapping::createMapGauss(std::string mapPath) {
    gaussMap.reset(new Gaussmap(mapPath));
    return gaussMap.get();
}


void Gaussmap::updateMap(bool isLast) {
    int xCoor, yCoor, zCoor;
    int i = 0;
    simpleMethodIndexes.clear();
    for(mapping::Point3D &point : cloud) {
        xCoor = xCoordinate(point.position.x());
        yCoor = yCoordinate(point.position.y());
        zCoor = zCoordinate(point.position.z());

        std::string key = std::to_string(xCoor) + std::to_string(yCoor) + std::to_string(zCoor);
        std::unordered_map<std::string, Eigen::Vector3i>::iterator got = indexes.find(key);
        if(got == indexes.end()) {
            indexes[key] = Eigen::Vector3i(xCoor, yCoor, zCoor);
        }

        if (methodType.type == MethodType::TYPE_SIMPLE || methodType.type == MethodType::TYPE_NAIVE) {
            got = simpleMethodIndexes.find(key);
            if(got == simpleMethodIndexes.end()) {
                simpleMethodIndexes[key] = Eigen::Vector3i(xCoor, yCoor, zCoor);
            }
        }
        //raytracePoint(point, xCoor, yCoor, zCoor);
        prevX,prevY,prevZ = -1;
        map(xCoor, yCoor, zCoor).insertPoint(point, uncertinatyErrors[i]);
        i++;
    }
    if (methodType.type == MethodType::TYPE_SIMPLE || methodType.type == MethodType::TYPE_NAIVE) {
        for( const auto& n : simpleMethodIndexes ) {
            Eigen::Vector3i index = n.second;
            if(index.x() == 58 && index.y() == 57 && index.z() == 61) {
                std::cout<<"Here!"<<std::endl;
            }
            map(index.x(), index.y(), index.z()).updateWithSimpleMethod();
        }
    }
    notify(map, res, indexes, isLast);
}

void Gaussmap::raytracePoint(mapping::Point3D point, int x, int y, int z) {
    Eigen::Vector3d incrementValue = Eigen::Vector3d((point.position.x() - cameraPos(0))/raytraceFactor, (point.position.y() - cameraPos(1))/raytraceFactor, (point.position.z() - cameraPos(2))/raytraceFactor);
    Eigen::Vector3d incrementedPoint = cameraPos;
    int i = 0;
    while (i < (raytraceFactor - 1)) {
        int xCoor = xCoordinate(incrementedPoint[0]);
        int yCoor = yCoordinate(incrementedPoint[1]);
        int zCoor = zCoordinate(incrementedPoint[2]);
        if(xCoor != x && yCoor != y && zCoor != z ) {
            map(xCoor, yCoor, zCoor).updateNullOccupancy();
            prevX = xCoor;
            prevY = yCoor;
            prevZ = zCoor;
        }
        incrementedPoint += incrementValue;
        i++;
    }
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

double Gaussmap::backwardXCoordinate(int x) {
    double  a = MAP_SIZE/(xmax-xmin);
    double b = -1 * a*xmin;
    return (x - b) / a;
}

double Gaussmap::backwardYCoordinate(int y) {
    double  a = MAP_SIZE/(ymax-ymin);
    double b = -1 * a*ymin;
    return (y - b) / a;
}

double Gaussmap::backwardZCoordinate(int z) {
    double  a = MAP_SIZE/(zmax-zmin);
    double b = -1 * a*zmin;
    return (z - b) / a;
}






















