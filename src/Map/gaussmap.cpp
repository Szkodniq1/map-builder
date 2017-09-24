#include "../include/Map/gaussmap.h"

/// A single instance of Octomap
Gaussmap::Ptr gaussMap;

Gaussmap::Gaussmap(void) : map(MAP_SIZE) {
    xmin = ymin = zmin = -1 * MAP_SIZE * res/2;
    xmax = ymax = zmax = -1* xmin;
    indexes.clear();
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
    indexes.clear();
    loadIndexes(mapPath);
}

void Gaussmap::loadIndexes(std::string mapPath) {
    tinyxml2::XMLDocument doc;
    std::string xmlPath = mapPath + ".xml";
    doc.LoadFile(xmlPath.c_str());
    tinyxml2::XMLElement* parent = doc.FirstChildElement("indexes");
    tinyxml2::XMLElement *row = parent->FirstChildElement();
    while (row != NULL) {
        tinyxml2::XMLElement *col = row->FirstChildElement();
        Eigen::Vector3i index;
        while (col != NULL) {
            std::string sKey;
            int sVal;
            char *sTemp1 = (char *)col->Value();
            if (sTemp1 != NULL) {
                sKey = static_cast<std::string>(sTemp1);
            } else {
                sKey = "";
            }
            char *sTemp2 = (char *)col->GetText();
            if (sTemp2 != NULL) {
                std::string temp = static_cast<std::string>(sTemp2);
                sVal = std::stoi(temp);
            } else {
                sVal = 0;
            }
            if (sKey == "x") {
                index(0) = sVal;
            }
            if (sKey == "y") {
                index(1) = sVal;
            }
            if (sKey == "z") {
                index(2) = sVal;
            }
            col = col->NextSiblingElement();
        }
        std::string key = std::to_string(index(0)) + std::to_string(index(1)) + std::to_string(index(2));
        indexes[key] = index;
        row = row->NextSiblingElement();
    }
}

Gaussmap::Gaussmap(mapping::PointCloud PC) : map(MAP_SIZE) {
    cloud = PC;
    xmin = ymin = zmin = -1 * MAP_SIZE * res/2;
    xmax = ymax = zmax = -1* xmin;
    indexes.clear();
    //preinitVoxels();
}

Gaussmap::Gaussmap(mapping::PointCloud PC, float vxmin, float vxmax, float vymin, float vymax, float vzmin, float vzmax) : map(MAP_SIZE) {
    cloud = PC;
    xmin = vxmin; xmax = vxmax;
    ymin = vymin; ymax = vymax;
    zmin = vzmin; zmax = vzmax;
    indexes.clear();
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
    //notify(grab.orientation, grab.translation);
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
    std::string path = currentDateTime();
    std::ofstream mapfile;
    mapfile.open(path.c_str(), std::ios_base::binary);
    map.writeBinary(mapfile);
    mapfile.close();

    tinyxml2::XMLDocument doc, config, img2pcl;
    config.LoadFile("../../resources/config.xml");
    img2pcl.LoadFile("../../resources/img2pcl.xml");
    tinyxml2::XMLElement * mapSize = doc.NewElement("MapSize");
    mapSize->SetText(MAP_SIZE);
    doc.InsertFirstChild(mapSize);
    tinyxml2::XMLElement * mapRes = doc.NewElement("MapRes");
    mapRes->SetText(res);
    doc.InsertFirstChild(mapRes);
    tinyxml2::XMLElement * rayFactor = doc.NewElement("RaytraceFactor");
    rayFactor->SetText(raytraceFactor);
    doc.InsertFirstChild(rayFactor);

    std::string datasetPath;
    datasetPath = img2pcl.FirstChildElement( "Path" )->GetText();
    std::vector<std::string> splitString = split(datasetPath, '/');
    tinyxml2::XMLElement * datasetDir = doc.NewElement("DatasetDirectory");
    datasetDir->SetText(splitString[splitString.size()-1].c_str());
    doc.InsertFirstChild(datasetDir);

    int method, every, until;
    config.FirstChildElement("OccupancyMethod")->QueryIntText(&method);
    config.FirstChildElement("CalculateEvery")->QueryIntText(&every);
    config.FirstChildElement("CalculateUntil")->QueryIntText(&until);
    tinyxml2::XMLElement * methodField = doc.NewElement("OccupancyMethod");
    methodField->SetText(method);
    doc.InsertFirstChild(methodField);
    tinyxml2::XMLElement * everyField = doc.NewElement("CalculateEvery");
    everyField->SetText(every);
    doc.InsertFirstChild(everyField);
    tinyxml2::XMLElement * untilField = doc.NewElement("CalculateUntil");
    untilField->SetText(until);
    doc.InsertFirstChild(untilField);

    tinyxml2::XMLElement* indexes = doc.NewElement("indexes");

    for( const auto& n : this->indexes ) {
        Eigen::Vector3i index = n.second;
        tinyxml2::XMLElement* item = doc.NewElement("item");
        tinyxml2::XMLElement* x = doc.NewElement("x");
        tinyxml2::XMLElement* y = doc.NewElement("y");
        tinyxml2::XMLElement* z = doc.NewElement("z");
        x->SetText(index.x());
        y->SetText(index.y());
        z->SetText(index.z());
        item->LinkEndChild(x);
        item->LinkEndChild(y);
        item->LinkEndChild(z);
        indexes->LinkEndChild(item);
    }

    doc.InsertEndChild(indexes);

    std::string xmlPath = path + ".xml";
    doc.SaveFile(xmlPath.c_str());
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
        //std::cout<<"Point size "<<point.position.x()<<" "<<point.position.y()<<" "<<point.position.z()<<std::endl;
        xCoor = xCoordinate(point.position.x());
        yCoor = yCoordinate(point.position.y());
        zCoor = zCoordinate(point.position.z());
        //std::cout<<"Point size "<<xCoor<<yCoor<<zCoor<<std::endl;
        if(xCoor >= map.size() || yCoor >=map.size() || zCoor>= map.size()) {
            std::cout<<"Point out of bounds"<<std::endl;
        } else {

            std::string key = std::to_string(xCoor) + std::to_string(yCoor) + std::to_string(zCoor);
            std::unordered_map<std::string, Eigen::Vector3i>::iterator got = indexes.find(key);
            if(got == indexes.end()) {
                indexes[key] = Eigen::Vector3i(xCoor, yCoor, zCoor);
            }

            got = simpleMethodIndexes.find(key);
            if(got == simpleMethodIndexes.end()) {
                simpleMethodIndexes[key] = Eigen::Vector3i(xCoor, yCoor, zCoor);
            }

            raytracePoint(point, xCoor, yCoor, zCoor);
            prevX,prevY,prevZ = -1;
            map(xCoor, yCoor, zCoor).insertPoint(point, uncertinatyErrors[i]);
            i++;
        }
    }


    for( const auto& n : simpleMethodIndexes ) {
        Eigen::Vector3i index = n.second;
        map(index.x(), index.y(), index.z()).updateWithSimpleMethod();
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

std::string Gaussmap::currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d-%X", &tstruct);

    return buf;
}

std::vector<std::string> Gaussmap::split(const std::string &s, char delim) {
    std::stringstream ss(s);
    std::string item;
    std::vector<std::string> elems;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
        // elems.push_back(std::move(item)); // if C++11 (based on comment from @mchiasson)
    }
    return elems;
}






















