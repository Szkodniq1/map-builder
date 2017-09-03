
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
    int mode;
    config.FirstChildElement("MapMode")->QueryIntText(&mode);
    std::cout << "Start\n";

    if(mode == 0) {
        config.FirstChildElement("MapSize")->QueryIntText(&MAP_SIZE);
        config.FirstChildElement("MapRes")->QueryDoubleText(&res);
        config.FirstChildElement("RaytraceFator")->QueryDoubleText(&raytraceFactor);
    } else {
         std::string mapSettingsPath = config.FirstChildElement( "ReadMapPath" )->GetText();
         mapSettingsPath += ".xml";
         tinyxml2::XMLDocument mapSettings;
         mapSettings.LoadFile(mapSettingsPath.c_str());
         mapSettings.FirstChildElement("MapSize")->QueryIntText(&MAP_SIZE);
         mapSettings.FirstChildElement("MapRes")->QueryDoubleText(&res);
         mapSettings.FirstChildElement("RaytraceFator")->QueryDoubleText(&raytraceFactor);
    }

    int a=0;
    mapping::GrabbedImage PC;
    mapping::img2pcl troll("../../resources/img2pcl.xml");

    QApplication application(argc,argv);
    setlocale(LC_NUMERIC,"C");
    glutInit(&argc, argv);

    QGLVisualizer visu;
    visu.setWindowTitle("Simulator viewer");
    visu.show();

    mapping::Map* map;
    if(mode == 0) {
        map = mapping::createMapGauss(PC.pointCloud);
        map->attachVisualizer(&visu);
    } else {
        std::string path = config.FirstChildElement( "ReadMapPath" )->GetText();
        map = mapping::createMapGauss(path);
        map->attachVisualizer(&visu);
        map->mapLoaded();
    }
    std::cout << map->getName() << "\n";

    if(mode == 0) {

        while(troll.grabFrame()) {
            if(a%100 == 0) {
                troll.calcPCL();
                PC = troll.returnPC();

                if(a == 100) {
                    map->insertCloud(PC, true);
                    break;
                } else {
                    map->insertCloud(PC, false);
                }

            }
            a++;
        }
        map->saveMap();
    }

    application.exec();

    std::cout << "Done\n";
}
