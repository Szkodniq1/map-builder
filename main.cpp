#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <iostream>

#include "octothread.h"
#include "Map/octomap.h"
#include "Visualizer/Qvisualizer.h"
#include <GL/glut.h>
#include <qapplication.h>

using namespace std;
using namespace octomap;

unsigned int filesSaved = 0;
bool saveCloud(false), noColor(false);
OcTree tree (0.01);

void printUsage(const char* programName)
{
    cout << "Usage: " << programName << " [options]"
         << endl
         << endl
         << "Options:\n"
         << endl
         << "\t<none>     start capturing from an OpenNI device.\n"
         << "\t-v FILE    visualize the given .pcd file.\n"
         << "\t-h         shows this help.\n";
}

int main(int argc, char** argv) {
    std::cout << "Start\n";
    mapping::Map* map = mapping::createMapOcto();
    std::cout << map->getName() << "\n";

    QApplication application(argc,argv);
    setlocale(LC_NUMERIC,"C");
    glutInit(&argc, argv);

    QGLVisualizer visu;
    visu.setWindowTitle("Simulator viewer");
    visu.show();
    map->attachVisualizer(&visu);


    map->insertCloud();

    application.exec();

    std::cout << "Done\n";


    tinyxml2::XMLDocument model;
    model.LoadFile("../../resources/KinectModel.xml");


    /*model->FirstChildElement( "focalLength" )->QueryDoubleAttribute("fx", &focalLength[0]);
    model->FirstChildElement( "focalLength" )->QueryDoubleAttribute("fy", &focalLength[1]);
    model->FirstChildElement( "focalAxis" )->QueryDoubleAttribute("Cx", &focalAxis[0]);
    model->FirstChildElement( "focalAxis" )->QueryDoubleAttribute("Cy", &focalAxis[1]);*/
}
