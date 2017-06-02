
#include <iostream>

#include "Map/octomap.h"
#include "Map/gaussmap.h"
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

    mapping::Mat33 mat;
    mat << 0.0096839, -0.0077051,  0.0155071,
           -0.0077051, 0.00613065, -0.0123384,
            0.0155071, -0.0123384,   0.024832;
    std::cout<<"TEST"<<std::endl<<mat.inverse()<<std::endl;
    int a=0;
    mapping::PointCloud PD, PCD;
    mapping::GrabbedImage PC;
    mapping::img2pcl troll("../../resources/img2pcl.xml");

    std::cout << "Start\n";
    mapping::Map* map = mapping::createMapGauss(PC.pointCloud);
    std::cout << map->getName() << "\n";

    QApplication application(argc,argv);
    setlocale(LC_NUMERIC,"C");
    glutInit(&argc, argv);

    QGLVisualizer visu;
    visu.setWindowTitle("Simulator viewer");
    visu.show();
    map->attachVisualizer(&visu);

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
    application.exec();

    std::cout << "Done\n";

    tinyxml2::XMLDocument model;
    model.LoadFile("../../resources/KinectModel.xml");
}
