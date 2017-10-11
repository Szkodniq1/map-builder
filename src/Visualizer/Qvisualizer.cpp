#include "Visualizer/Qvisualizer.h"
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <memory>
#include <cmath>
#include <stdexcept>
#include <chrono>
#include <GL/glut.h>
#include "Map/gaussmap.h"
#include "main.h"
#include <QFile>
#include <QTextStream>


using namespace mapping;

/// A single instance of Visualizer
QGLVisualizer::Ptr visualizer;

QGLVisualizer::QGLVisualizer(void) : map(MAP_SIZE) {

}

/// Construction
QGLVisualizer::QGLVisualizer(Config _config): config(_config), map(MAP_SIZE){

}

/// Construction
QGLVisualizer::QGLVisualizer(std::string configFile) :
    config(configFile), map(MAP_SIZE) {
    tinyxml2::XMLDocument configXML;
    std::string filename = "../../resources/" + configFile;
    configXML.LoadFile(filename.c_str());
    if (configXML.ErrorID())
        std::cout << "unable to load visualizer config file.\n";
}

/// Destruction
QGLVisualizer::~QGLVisualizer(void) {
}


/// Observer update
void QGLVisualizer::update(const mapping::PointCloud& newCloud, std::vector<Mat33> uncertinatyErrors, bool isLast) {
    this->uncertinatyErrors = uncertinatyErrors;
    mtxPointCloud.lock();
    pointClouds.push_back(newCloud);
    if(isLast) {
        createDisplayList();
    }
    mtxPointCloud.unlock();
}

void QGLVisualizer::update(Octree<mapping::Voxel>& map, double res, std::unordered_map<std::string, Eigen::Vector3i> indexes , bool isLast) {

    this->map = map;
    //TODO trzeba t? liste gdzie? czy?ci?, albo co update przepisywa?, ogólnie trzeba si? zastanowi? jak te dane trzyma? w ogóle, bo jedne mog? nadpisa? drugie
    this->updatedVoxels = indexes;
    if(isLast) {
        createMapDisplayList();
    }

}

void QGLVisualizer::update(mapping::Quaternion orientation, mapping::Vec3 translation) {
    camera()->setOrientation(qglviewer::Quaternion(orientation.x(), orientation.y(), orientation.z(), orientation.w()));
    camera()->setPosition(qglviewer::Vec(translation.x(), translation.y(), translation.z()));
}

void QGLVisualizer::createMapDisplayList() {
    int probabilityTreshold;
    tinyxml2::XMLDocument xmlDoc;
    xmlDoc.LoadFile("../../resources/config.xml");
    xmlDoc.FirstChildElement("ProbabilityTreshold")->QueryIntText(&probabilityTreshold);
    list = glGenLists(1);
    glNewList(list, GL_COMPILE);
    for( const auto& n : updatedVoxels ) {
        Eigen::Vector3i indexes = n.second;
        Voxel v = map(indexes.x(), indexes.y(), indexes.z());
        if(v.probability > probabilityTreshold) {
            drawEllipsoid(Vec3(v.mean.x(), v.mean.y(), v.mean.z()), v.var, v.color);
        }
    }

    /*for(int i = 28 ; i < 108;  i++) { //28 & 108
        for(int j = 48 ; j < 88;  j++) { //48 & 88
            for(int k = 48 ; k < 88;  k++) {
                Voxel v = map(i, j, k);
                if(v.probability > 0) {
                    drawEllipsoid(Vec3(v.mean.x(), v.mean.y(), v.mean.z()), v.var, v.color);
                }
            }
        }
    }*/

    glPointSize(3);
    glBegin(GL_POINTS);
    for(mapping::PointCloud pointCloud : pointClouds) {
        for (size_t i = 0;i<pointCloud.size();i++) {
            glColor3ub(pointCloud[i].color.r,pointCloud[i].color.g,pointCloud[i].color.b);
            glVertex3d(pointCloud[i].position.x()+5, pointCloud[i].position.y(), pointCloud[i].position.z());
        }
    }
    glEnd();
    glEndList();
}

void QGLVisualizer::createDisplayList() {
    list = glGenLists(1);
    glNewList(list, GL_COMPILE);
    glPointSize(3);
    glBegin(GL_POINTS);
    for(mapping::PointCloud pointCloud : pointClouds) {
        for (size_t i = 0;i<pointCloud.size();i++) {
            glColor4ub(pointCloud[i].color.r,pointCloud[i].color.g,pointCloud[i].color.b, pointCloud[i].color.a);
            glVertex3d(pointCloud[i].position.x(), pointCloud[i].position.y(), pointCloud[i].position.z());
        }
    }
    glEnd();
    glEndList();
}


/// Draw point clouds
void QGLVisualizer::drawPointCloud(void){
    mtxPointCloud.lock();
    glPushMatrix();
    glCallList(list);
    glPopMatrix();
    mtxPointCloud.unlock();
}
/// draw objects
void QGLVisualizer::draw(){
    // Here we are in the world coordinate system. Draw unit size axis.
    //drawAxis();
    //drawPointCloud();
    drawMap();
}

/// draw objects
void QGLVisualizer::animate(){
    if(shouldAnimate == 1) {
        camera()->setPosition(qglviewer::Vec(animationCenter.x() + animationRay*cos((angle * M_PI)/180.0), animationCenter.y(), animationCenter.z() + animationRay*sin((angle * M_PI)/180.0)));
        camera()->lookAt(qglviewer::Vec(animationCenter.x(), animationCenter.y()+(double)animationYOffset, animationCenter.z()));
        angle += 0.5;
        if(angle >= 360.0) {
            angle -= 360.0;
        }
    }
}


class dPoint{
public:
    double x;
    double y;

    dPoint();
};

dPoint::dPoint(){
    x = y = 0;
}

dPoint CircleCenter(double x1,double y1,double x2,double y2,double x3,double y3){
    dPoint dp;
    dp.x = 0.5 * ((x2 * x2 * y3 + y2 * y2 * y3 - x1 * x1 * y3 + x1 * x1 * y2 - y1 * y1 * y3 + y1 * y1 * y2 + y1 * x3 * x3 + y1 * y3 * y3 - y1 * x2 * x2 - y1 * y2 * y2 - y2 * x3 * x3 - y2 * y3 * y3) / (y1 * x3 - y1 * x2 - y2 * x3 - y3 * x1 + y3 * x2 + y2 * x1));
    dp.y = 0.5 * ((-x1 * x3 * x3 - x1 * y3 * y3 + x1 * x2 * x2 + x1 * y2 * y2 + x2 * x3 * x3 + x2 * y3 * y3 - x2 * x2 * x3 - y2 * y2 * x3 + x1 * x1 * x3 - x1 * x1 * x2 + y1 * y1 * x3 - y1 * y1 * x2) / (y1 * x3 - y1 * x2 - y2 * x3 - y3 * x1 + y3 * x2 + y2 * x1));
    return dp;
}

/// initialize visualizer
void QGLVisualizer::init(){
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    // Create light components
    GLfloat ambientLight[] = { 0.5f, 0.5f, 0.5f, 1.0f };
    GLfloat diffuseLight[] = { 0.8f, 0.8f, 0.8, 1.0f };
    GLfloat specularLight[] = { 0.5f, 0.5f, 0.5f, 1.0f };
    GLfloat emissiveLight[] = { 0.5f, 0.5f, 0.5f, 1.0f };
    GLfloat position[] = { -1.5f, 1.0f, -4.0f, 1.0f };

    // Assign created components to GL_LIGHT0
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
    //glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight);
    //glLightfv(GL_LIGHT0, GL_EMISSION, emissiveLight);
    //glLightfv(GL_LIGHT0, GL_POSITION, position);

    glShadeModel(GL_SMOOTH);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_AUTO_NORMAL);
    glEnable(GL_NORMALIZE);
    // Restore previous viewer state.
    //restoreStateFromFile();

    camera()->setZNearCoefficient(0.00001f);
    camera()->setZClippingCoefficient(100.0);

    tinyxml2::XMLDocument configFile;
    configFile.LoadFile("../../resources/config.xml");

    int shouldLoadCamera;

    configFile.FirstChildElement("LoadCamera")->QueryIntText(&shouldLoadCamera);
    configFile.FirstChildElement("Animate")->QueryIntText(&shouldAnimate);

    if(shouldAnimate == 1) {
        int decimal;
        configFile.FirstChildElement("AnimationYOffset")->QueryIntText(&decimal);
        if(decimal != 0) {
            animationYOffset = (float) decimal / 100.0;
        }
        tinyxml2::XMLDocument firstPoint, secondPoint, thirdPoint;
        std::string path;
        path = configFile.FirstChildElement( "CameraAnimPath1" )->GetText();
        path += ".xml";
        firstPoint.LoadFile(path.c_str());
        path = configFile.FirstChildElement( "CameraAnimPath2" )->GetText();
        path += ".xml";
        secondPoint.LoadFile(path.c_str());
        path = configFile.FirstChildElement( "CameraAnimPath3" )->GetText();
        path += ".xml";
        thirdPoint.LoadFile(path.c_str());

        double x1,y1,z1,x2,z2,x3,z3;

        x1 = firstPoint.FirstChildElement("Camera")->FirstChildElement("ManipulatedCameraFrame")->FirstChildElement("position")->DoubleAttribute("x");
        y1 = firstPoint.FirstChildElement("Camera")->FirstChildElement("ManipulatedCameraFrame")->FirstChildElement("position")->DoubleAttribute("y");
        z1 = firstPoint.FirstChildElement("Camera")->FirstChildElement("ManipulatedCameraFrame")->FirstChildElement("position")->DoubleAttribute("z");

        x2 = secondPoint.FirstChildElement("Camera")->FirstChildElement("ManipulatedCameraFrame")->FirstChildElement("position")->DoubleAttribute("x");
        z2 = secondPoint.FirstChildElement("Camera")->FirstChildElement("ManipulatedCameraFrame")->FirstChildElement("position")->DoubleAttribute("z");

        x3 = thirdPoint.FirstChildElement("Camera")->FirstChildElement("ManipulatedCameraFrame")->FirstChildElement("position")->DoubleAttribute("x");
        z3 = thirdPoint.FirstChildElement("Camera")->FirstChildElement("ManipulatedCameraFrame")->FirstChildElement("position")->DoubleAttribute("z");

        dPoint point = CircleCenter(x1,z1,x2,z2,x3,z3);
        this->animationCenter = Vec3(point.x, y1, point.y);
        animationRay = sqrt(pow(x1-animationCenter.x(), 2.0) + pow(y1-animationCenter.y(), 2.0) + pow(z1-animationCenter.z(), 2.0));
    }

    if(shouldLoadCamera == 1) {
        std::string path;
        path = configFile.FirstChildElement( "CameraPath" )->GetText();
        path += ".xml";

        // Load DOM from file
        QDomDocument document;
        QFile f(path.c_str());
        if (f.open(QIODevice::ReadOnly))
        {
            document.setContent(&f);
            f.close();
        }
        // Parse the DOM tree
        QDomElement main = document.documentElement();
        camera()->initFromDOMElement(main);
    }

    setBackgroundColor(config.backgroundColor);

    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Opens help
    help();
    setAnimationPeriod(1);
    startAnimation();
}

void QGLVisualizer::drawMap() {
    glPushMatrix();
    glCallList(list);
    glPopMatrix();
}

void QGLVisualizer::drawEllipsoid(const Vec3& pos, const Mat33& covariance, RGBA color) const{
    // ---------------------
    //    3D ellipsoid
    // ---------------------
    GLfloat		mat[16];
    Eigen::SelfAdjointEigenSolver<Mat33> es;
    es.compute(covariance);
    Mat33 m_eigVec(es.eigenvectors());
    m_eigVec = m_eigVec;

    //  A homogeneous transformation matrix, in this order:
    //
    //     0  4  8  12
    //     1  5  9  13
    //     2  6  10 14
    //     3  7  11 15
    //
    mat[3] = mat[7] = mat[11] = 0;
    mat[15] = 1;
    mat[12] = mat[13] = mat[14] = 0;

    mat[0] = m_eigVec(0,0); mat[1] = m_eigVec(1,0); mat[2] = m_eigVec(2,0); mat[12] = pos.x();// New X-axis
    mat[4] = m_eigVec(0,1); mat[5] = m_eigVec(1,1); mat[6] = m_eigVec(2,1);	mat[13] = pos.y();// New X-axis
    mat[8] = m_eigVec(0,2); mat[9] = m_eigVec(1,2); mat[10] = m_eigVec(2,2); mat[14] = pos.z();// New X-axis

    GLUquadricObj	*obj = gluNewQuadric();

    gluQuadricDrawStyle( obj, GLU_FILL);

    glPushMatrix();
    glMultMatrixf( mat );

    glScalef(sqrt(es.eigenvalues()(0)),sqrt(es.eigenvalues()(1)),sqrt(es.eigenvalues()(2)));
    float reflectColor[] = { 0.8f, 0.8f, 0.8f, 1.0f };
    //glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, reflectColor);
    GLfloat emissiveLight[] = { 0.1f, 0.1f, 0.1f, 1.0f };
    //glMaterialfv(GL_FRONT, GL_EMISSION, emissiveLight);
    glColor4ub(color.r,color.g,color.b, color.a);
    gluSphere( obj, 1,15,15);
    glPopMatrix();

    gluDeleteQuadric(obj);
}

void QGLVisualizer::keyPressEvent(QKeyEvent *e) {
    const Qt::KeyboardModifiers modifiers = e->modifiers();

    bool handled = false;
    if ((e->key() == Qt::Key_Z) && (modifiers == Qt::CTRL)) {

        tinyxml2::XMLDocument img2pcl;
        img2pcl.LoadFile("../../resources/img2pcl.xml");
        std::string datasetPath;
        datasetPath = img2pcl.FirstChildElement( "Path" )->GetText();
        std::vector<std::string> splitString = split(datasetPath, '/');
        std::string filename = splitString[splitString.size()-1] + "-" + currentDateTime() + ".xml";
        QDomDocument document(filename.c_str());
        document.appendChild( camera()->domElement("Camera", document) );
        QFile f(filename.c_str());
        if (f.open(QIODevice::WriteOnly))
        {
            QTextStream out(&f);
            document.save(out, 2);
        }
        handled = true;
    }

    if (!handled)
        QGLViewer::keyPressEvent(e);
}


/// generate help string
std::string QGLVisualizer::help() const{
    std::string text("S i m p l e V i e w e r");
    text += "Use the mouse to move the camera around the object. ";
    text += "You can respectively revolve around, zoom and translate with the three mouse buttons. ";
    text += "Left and middle buttons pressed together rotate around the camera view direction axis<br><br>";
    text += "Pressing <b>Alt</b> and one of the function keys (<b>F1</b>..<b>F12</b>) defines a camera keyFrame. ";
    text += "Simply press the function key again to restore it. Several keyFrames define a ";
    text += "camera path. Paths are saved when you quit the application and restored at next start.<br><br>";
    text += "Press <b>F</b> to display the frame rate, <b>A</b> for the world axis, ";
    text += "<b>Alt+Return</b> for full screen mode and <b>Control+S</b> to save a snapshot. ";
    text += "See the <b>Keyboard</b> tab in this window for a complete shortcut list.<br><br>";
    text += "Double clicks automates single click actions: A left button double click aligns the closer axis with the camera (if close enough). ";
    text += "A middle button double click fits the zoom of the camera and the right button re-centers the scene.<br><br>";
    text += "A left button double click while holding right button pressed defines the camera <i>Revolve Around Point</i>. ";
    text += "See the <b>Mouse</b> tab and the documentation web pages for details.<br><br>";
    text += "Press <b>Escape</b> to exit the viewer.";
    return text;
}

std::string QGLVisualizer::currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d-%X", &tstruct);

    return buf;
}

std::vector<std::string> QGLVisualizer::split(const std::string &s, char delim) {
    std::stringstream ss(s);
    std::string item;
    std::vector<std::string> elems;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
        // elems.push_back(std::move(item)); // if C++11 (based on comment from @mchiasson)
    }
    return elems;
}
