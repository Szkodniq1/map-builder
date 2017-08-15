#include "occmethodtype.h"
namespace mapping {

OccMethodType::OccMethodType() {
    initMethodType();
}

void OccMethodType::initMethodType() {
    tinyxml2::XMLDocument xmlDoc;
    tinyxml2::XMLError Result = xmlDoc.LoadFile(this->configPath.c_str());
    int typeNum;
    if(Result == tinyxml2::XML_SUCCESS) {
        xmlDoc.FirstChildElement("OccupancyMethod")->QueryIntText(&typeNum);
        this->assignType(typeNum);
    } else {
        std::cout << "Loading configuration file failed!\n" << std::endl << configPath;
        exit(404);
    }
}

void OccMethodType::assignType(int typeNum) {
    switch (typeNum) {
    case 0:
        this->type = MethodType::TYPE_SIMPLE;
        break;
    case 1:
        this->type = MethodType::TYPE_BAYES;
        break;
    case 2:
        this->type = MethodType::TYPE_KALMAN;
        break;
    default:
        this->type = MethodType::TYPE_SIMPLE;
        break;
    }
}
}
