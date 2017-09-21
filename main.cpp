
#include <iostream>
#include "main.h"
#include "Map/octomap.h"
#include "Map/gaussmap.h"
#include "Visualizer/Qvisualizer.h"
#include <GL/glut.h>
#include <qapplication.h>
#include "include/Utilities/img2pcl.h"

using namespace std;

int MAP_SIZE;
double res;
double raytraceFactor;

unsigned int filesSaved = 0;
bool saveCloud(false), noColor(false);

int main(int argc, char** argv) {

    tinyxml2::XMLDocument config;
    config.LoadFile("../../resources/config.xml");
    int mode, mapType;
    config.FirstChildElement("MapMode")->QueryIntText(&mode);
    config.FirstChildElement("MapType")->QueryIntText(&mapType);
    std::cout << "Start\n";

    if(mode == 0) {
        config.FirstChildElement("MapSize")->QueryIntText(&MAP_SIZE);
        config.FirstChildElement("MapRes")->QueryDoubleText(&res);
        config.FirstChildElement("RaytraceFactor")->QueryDoubleText(&raytraceFactor);
    } else {
        std::string mapSettingsPath = config.FirstChildElement( "ReadMapPath" )->GetText();
        mapSettingsPath += ".xml";
        tinyxml2::XMLDocument mapSettings;
        mapSettings.LoadFile(mapSettingsPath.c_str());
        mapSettings.FirstChildElement("MapSize")->QueryIntText(&MAP_SIZE);
        mapSettings.FirstChildElement("MapRes")->QueryDoubleText(&res);
        mapSettings.FirstChildElement("RaytraceFactor")->QueryDoubleText(&raytraceFactor);
    }

    int a=0;
    mapping::GrabbedImage PC;
    mapping::img2pcl img2pcl("../../resources/img2pcl.xml");

    QApplication application(argc,argv);
    setlocale(LC_NUMERIC,"C");
    glutInit(&argc, argv);

    QGLVisualizer visu;
    visu.setWindowTitle("Simulator viewer");
    visu.show();

    mapping::Map* map;
    if(mode == 0) {
        if(mapType != 0) {
            map = mapping::createMapGauss(PC.pointCloud);
        } else {
            map = mapping::createMapOcto(PC.pointCloud);
        }
        map->attachVisualizer(&visu);
    } else {
        std::string path = config.FirstChildElement( "ReadMapPath" )->GetText();
        map = mapping::createMapGauss(path);
        map->attachVisualizer(&visu);
    }

    if(mode == 0) {

        int calcEvery, calcUntil;
        config.FirstChildElement("CalculateEvery")->QueryIntText(&calcEvery);
        config.FirstChildElement("CalculateUntil")->QueryIntText(&calcUntil);

        while(img2pcl.grabFrame()) {
            if(a%calcEvery == 0) {
                img2pcl.calcPCL();
                PC = img2pcl.returnPC();

                if(a == calcUntil) {
                    map->insertCloud(PC, true);
                    break;
                } else {
                    map->insertCloud(PC, false);
                }

            }
            a++;
        }
        int shouldSave;
        config.FirstChildElement("SaveMap")->QueryIntText(&shouldSave);
        if(shouldSave != 0) {
            map->saveMap();
        }
    } else {
        map->mapLoaded();
    }

    application.exec();

    std::cout << "Done\n";
}
