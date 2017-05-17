#include "Visualizer/Qvisualizer.h"
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <memory>
#include <cmath>
#include <stdexcept>
#include <chrono>
#include <GL/glut.h>

using namespace mapping;

/// A single instance of Visualizer
QGLVisualizer::Ptr visualizer;

class SolidSphere
{
protected:
    std::vector<GLfloat> vertices;
    std::vector<GLfloat> normals;
    std::vector<GLfloat> texcoords;
    std::vector<GLushort> indices;

public:
    SolidSphere(float radius, unsigned int rings, unsigned int sectors)
    {
        double const R = 1.0f/(double)(rings-1);
        double const S = 1.0f/(double)(sectors-1);
        unsigned int r, s;

        vertices.resize(rings * sectors * 3);
        normals.resize(rings * sectors * 3);
        texcoords.resize(rings * sectors * 2);
        std::vector<GLfloat>::iterator v = vertices.begin();
        std::vector<GLfloat>::iterator n = normals.begin();
        std::vector<GLfloat>::iterator t = texcoords.begin();
        for(r = 0; r < rings; r++)
            for(s = 0; s < sectors; s++) {
                double const y = (double) sin( -M_PI_2 + M_PI * r * R );
                double const x = (double) cos(2*M_PI * s * S) * (double) sin( M_PI * r * R );
                double const z = (double) sin(2*M_PI * s * S) * (double) sin( M_PI * r * R );

                *t++ = (double) s*S;
                *t++ = (double) r*R;

                *v++ = (double) x * radius;
                *v++ = (double) y * radius;
                *v++ = (double) z * radius;

                *n++ = (double) x;
                *n++ = (double) y;
                *n++ = (double) z;
            }

        indices.resize(rings * sectors * 4);
        std::vector<GLushort>::iterator i = indices.begin();
        for(r = 0; r < rings-1; r++)
            for(s = 0; s < sectors-1; s++) {
                *i++ = (GLushort) ((double) r * (double)sectors + (double) s);
                *i++ = (GLushort) ((double) r * (double)sectors + (double) (s+1));
                *i++ = (GLushort) ((double) (r+1) * (double)sectors + (double) (s+1));
                *i++ = (GLushort) ((double) (r+1) * (double)sectors + (double) s);
            }
    }

    void draw(GLfloat x, GLfloat y, GLfloat z)
    {
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glTranslatef(x,y,z);
        glEnable(GL_NORMALIZE);

        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_NORMAL_ARRAY);

        glVertexPointer(3, GL_FLOAT, 0, &vertices[0]);
        glNormalPointer(GL_FLOAT, 0, &normals[0]);
        glDrawElements(GL_QUADS, (int) indices.size(), GL_UNSIGNED_SHORT, &indices[0]);
        glPopMatrix();
    }
};

QGLVisualizer::QGLVisualizer(void) : map(256) {

}

/// Construction
QGLVisualizer::QGLVisualizer(Config _config): config(_config), map(256){

}

/// Construction
QGLVisualizer::QGLVisualizer(std::string configFile) :
    config(configFile), map(256) {
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
void QGLVisualizer::update(const mapping::PointCloud& newCloud, bool isLast) {
    mtxPointCloud.lock();
    pointClouds.push_back(newCloud);
    if(isLast) {
        createDisplayList();
    }
    mtxPointCloud.unlock();
}

void QGLVisualizer::update(Octree<mapping::Voxel>& map, double res, std::unordered_map<std::string, Eigen::Vector3i> indexes) {
    this->map = map;
    this->res = res;
    //TODO trzeba t¹ liste gdzieœ czyœciæ, albo co update przepisywaæ, ogólnie trzeba siê zastanowiæ jak te dane trzymaæ w ogóle, bo jedne mog¹ nadpisaæ drugie
    this->updatedVoxels = indexes;
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
    drawAxis();
    //drawPointCloud();
    drawMap(this->map);
}

/// draw objects
void QGLVisualizer::animate(){
}


/// initialize visualizer
void QGLVisualizer::init(){
    GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
       GLfloat mat_shininess[] = { 50.0 };
       GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };
       glClearColor (0.0, 0.0, 0.0, 0.0);
       glShadeModel (GL_SMOOTH);

       glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
       glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
       glLightfv(GL_LIGHT0, GL_POSITION, light_position);

       glEnable(GL_LIGHTING);
       glEnable(GL_LIGHT0);
       glEnable(GL_DEPTH_TEST);
    glEnable(GL_AUTO_NORMAL);
    glEnable(GL_NORMALIZE);
    // Restore previous viewer state.
    //restoreStateFromFile();

    camera()->setZNearCoefficient(0.00001f);
    camera()->setZClippingCoefficient(100.0);

    setBackgroundColor(config.backgroundColor);

    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Opens help window
    help();

    startAnimation();
}

Octree<mapping::Voxel> QGLVisualizer::prepareTestMap() {
    Octree<mapping::Voxel> map(4);
    Eigen::Vector3d mean = Eigen::Vector3d(1,1,1);
    Mat33 dev;
    dev <<  1, 0, 0,
            0, 1, 0,
            0, 0, 0.25;
    RGBA color = RGBA(255, 255, 0);
    map(0,0,2) = Voxel(1, 0, mean, dev, color);
    map(0,1,2) = Voxel(1, 0, mean, dev, color);
    map(0,1,1) = Voxel(1, 0, mean, dev, color);
    map(0,2,2) = Voxel(1, 0, mean, dev, color);
    map(0,3,2) = Voxel(1, 0, mean, dev, color);
    map(1,1,2) = Voxel(1, 0, mean, dev, color);
    map(1,2,2) = Voxel(1, 0, mean, dev, color);
    map(1,2,3) = Voxel(1, 0, mean, dev, color);
    map(1,3,2) = Voxel(1, 0, mean, dev, color);
    map(2,3,2) = Voxel(1, 0, mean, dev, color);
    map(2,2,2) = Voxel(1, 0, mean, dev, color);
    map(3,3,2) = Voxel(1, 0, mean, dev, color);
    return map;
}

void QGLVisualizer::drawMap(Octree<mapping::Voxel> map) {
    Eigen::Vector3d mean = Eigen::Vector3d(1,1,1);
    Mat33 dev;
    dev <<  0.25, 0, 0,
            0, 0.25, 0,
            0, 0, 0.125;
    RGBA color = RGBA(255, 255, 0);
    int n = map.size();
    for( const auto& n : updatedVoxels ) {
        Eigen::Vector3i indexes = n.second;
                    glPushMatrix();
                    Voxel v = Voxel(1, 0, mean, dev, color);
                    GLfloat mat[]={
                    v.var(0,0), v.var(1,0), v.var(2,0), 0, // vecteur1
                    v.var(0,1), v.var(1,1), v.var(2,1), 0, // vecteur2
                    v.var(0,2), v.var(1,2), v.var(2,2), 0, // vecteur3
                    (v.mean.x() * indexes(0) * res) - 3.2, (v.mean.y() * indexes(1) * res ) - 3.2, (v.mean.z() * indexes(2) * res ) - 3.2, 1
                    };
                    glMultMatrixf(mat);
                    glutSolidSphere(1,10,10);//drawCloudObj(pointsObjVox);
                    glColor4ub(v.color.r,v.color.g,v.color.b, v.color.a);
                    //glColor4ub(255,255,0,0);
                    glPopMatrix();
                    glFlush();
    }
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

