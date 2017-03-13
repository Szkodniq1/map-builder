#include <iostream>

#include "Map/octomap.h"
#include "Visualizer/Qvisualizer.h"
#include <GL/glut.h>
#include <qapplication.h>
#include "include/Utilities/img2pcl.h"

using namespace std;

unsigned int filesSaved = 0;
bool saveCloud(false), noColor(false);

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

    int a=0;
    mapping::PointCloud PC, PD, PCD;
    mapping::img2pcl troll("../../resources/img2pcl.xml");
    while(troll.grabFrame()) {

        if(a%300 == 0) {
            troll.calcPCL();
            PC = troll.returnPC();
            PD.insert(PD.end(), PC.begin(), PC.end());
            break;
        }

        a++;
    }


    std::cout << "Start\n";
    mapping::Map* map = mapping::createMapOcto(PC);
    std::cout << map->getName() << "\n";

    QApplication application(argc,argv);
    setlocale(LC_NUMERIC,"C");
    glutInit(&argc, argv);

    QGLVisualizer visu;
    visu.setWindowTitle("Simulator viewer");
    visu.show();
    map->attachVisualizer(&visu);

    octomap::Pointcloud pcl = octomap::Pointcloud();

    map->insertCloud(pcl);

    application.exec();

    std::cout << "Done\n";


    tinyxml2::XMLDocument model;
    model.LoadFile("../../resources/KinectModel.xml");


    /*model->FirstChildElement( "focalLength" )->QueryDoubleAttribute("fx", &focalLength[0]);
    model->FirstChildElement( "focalLength" )->QueryDoubleAttribute("fy", &focalLength[1]);
    model->FirstChildElement( "focalAxis" )->QueryDoubleAttribute("Cx", &focalAxis[0]);
    model->FirstChildElement( "focalAxis" )->QueryDoubleAttribute("Cy", &focalAxis[1]);*/
}
