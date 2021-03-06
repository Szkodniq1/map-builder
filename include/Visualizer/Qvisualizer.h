/** @file QVisualizer.h
 *
 * implementation - QGLVisualizer
 *
 */

#ifndef QVISUALIZER_H_INCLUDED
#define QVISUALIZER_H_INCLUDED

#include "../Defs/defs.h"
#include "../Voxel/voxel.h"
#include "Utilities/observer.h"
#include "../../3rdParty/tinyXML/tinyxml2.h"
#include <QGLViewer/qglviewer.h>
#include <iostream>
#include <memory>
#include <thread>
#include <mutex>
#include "3rdParty/octree/octree.h"
#include <QKeyEvent>
#include <fstream>
#include "main.h"

using namespace mapping;

/// Map implementation
class QGLVisualizer: public QGLViewer, public Observer{
public:
    /// Pointer
    typedef std::unique_ptr<QGLVisualizer> Ptr;

    class Config{
      public:
        Config() {
            this->backgroundColor = QColor(255,255,255);
        }
        Config(std::string configFilename){
//            tinyxml2::XMLDocument config;
//            std::string filename = "../../resources/" + configFilename;
//            config.LoadFile(filename.c_str());
//            if (config.ErrorID())
//                std::cout << "unable to load Visualizer config file.\n";
//            tinyxml2::XMLElement * model = config.FirstChildElement( "VisualizerConfig" );
//            double rgba[4]={0,0,0,0};
//            model->FirstChildElement( "background" )->QueryDoubleAttribute("red", &rgba[0]);
//            model->FirstChildElement( "background" )->QueryDoubleAttribute("green", &rgba[1]);
//            model->FirstChildElement( "background" )->QueryDoubleAttribute("blue", &rgba[2]);
//            model->FirstChildElement( "background" )->QueryDoubleAttribute("alpha", &rgba[3]);
//            backgroundColor.setRedF(rgba[0]); backgroundColor.setGreenF(rgba[1]);
//            backgroundColor.setBlueF(rgba[2]); backgroundColor.setAlphaF(rgba[3]);
//            model->FirstChildElement( "trajectory" )->QueryBoolAttribute("drawTrajectory", &drawTrajectory);
//            model->FirstChildElement( "trajectory" )->QueryDoubleAttribute("red", &rgba[0]);
//            model->FirstChildElement( "trajectory" )->QueryDoubleAttribute("green", &rgba[1]);
//            model->FirstChildElement( "trajectory" )->QueryDoubleAttribute("blue", &rgba[2]);
//            model->FirstChildElement( "trajectory" )->QueryDoubleAttribute("alpha", &rgba[3]);
//            model->FirstChildElement( "trajectory" )->QueryDoubleAttribute("width", &trajectoryWidth);
//            trajectoryColor.setRedF(rgba[0]); trajectoryColor.setGreenF(rgba[1]);
//            trajectoryColor.setBlueF(rgba[2]); trajectoryColor.setAlphaF(rgba[3]);
//            model->FirstChildElement( "trajectoryPoints" )->QueryBoolAttribute("drawTrajectoryPoints", &drawTrajectoryPoints);
//            model->FirstChildElement( "trajectoryPoints" )->QueryDoubleAttribute("size", &trajectoryPointsSize);
//            model->FirstChildElement( "trajectoryPoints" )->QueryDoubleAttribute("red", &rgba[0]);
//            model->FirstChildElement( "trajectoryPoints" )->QueryDoubleAttribute("green", &rgba[1]);
//            model->FirstChildElement( "trajectoryPoints" )->QueryDoubleAttribute("blue", &rgba[2]);
//            model->FirstChildElement( "trajectoryPoints" )->QueryDoubleAttribute("alpha", &rgba[3]);
//            model->FirstChildElement( "trajectoryPoints" )->QueryIntAttribute("smoothness", &trajectoryPointSmoothness);
//            trajectoryPointsColor.setRedF(rgba[0]); trajectoryPointsColor.setGreenF(rgba[1]);
//            trajectoryPointsColor.setBlueF(rgba[2]); trajectoryPointsColor.setAlphaF(rgba[3]);
//            model->FirstChildElement( "features" )->QueryBoolAttribute("drawFeatures", &drawFeatures);
//            model->FirstChildElement( "features" )->QueryDoubleAttribute("size", &featuresSize);
//            model->FirstChildElement( "features" )->QueryDoubleAttribute("red", &rgba[0]);
//            model->FirstChildElement( "features" )->QueryDoubleAttribute("green", &rgba[1]);
//            model->FirstChildElement( "features" )->QueryDoubleAttribute("blue", &rgba[2]);
//            model->FirstChildElement( "features" )->QueryDoubleAttribute("alpha", &rgba[3]);
//            model->FirstChildElement( "features" )->QueryIntAttribute("smoothness", &featuresSmoothness);
//            featuresColor.setRedF(rgba[0]); featuresColor.setGreenF(rgba[1]);
//            featuresColor.setBlueF(rgba[2]); featuresColor.setAlphaF(rgba[3]);
//            model->FirstChildElement( "pose2feature" )->QueryBoolAttribute("drawLinks", &drawPose2Feature);
//            model->FirstChildElement( "pose2feature" )->QueryDoubleAttribute("red", &rgba[0]);
//            model->FirstChildElement( "pose2feature" )->QueryDoubleAttribute("green", &rgba[1]);
//            model->FirstChildElement( "pose2feature" )->QueryDoubleAttribute("blue", &rgba[2]);
//            model->FirstChildElement( "pose2feature" )->QueryDoubleAttribute("alpha", &rgba[3]);
//            model->FirstChildElement( "pose2feature" )->QueryDoubleAttribute("width", &pose2FeatureWidth);
//            pose2FeatureColor.setRedF(rgba[0]); pose2FeatureColor.setGreenF(rgba[1]);
//            pose2FeatureColor.setBlueF(rgba[2]); pose2FeatureColor.setAlphaF(rgba[3]);
//            model->FirstChildElement( "camera" )->QueryBoolAttribute("flyingCamera", &flyingCamera);
//            model->FirstChildElement( "pointCloud" )->QueryBoolAttribute("drawPointClouds", &drawPointClouds);
//            model->FirstChildElement( "pointCloud" )->QueryIntAttribute("cloudPointSize", &cloudPointSize);
//            // measurements
//            model->FirstChildElement( "measurements" )->QueryBoolAttribute("drawMeasurements", &drawMeasurements);
//            model->FirstChildElement( "measurements" )->QueryDoubleAttribute("red", &rgba[0]);
//            model->FirstChildElement( "measurements" )->QueryDoubleAttribute("green", &rgba[1]);
//            model->FirstChildElement( "measurements" )->QueryDoubleAttribute("blue", &rgba[2]);
//            model->FirstChildElement( "measurements" )->QueryDoubleAttribute("alpha", &rgba[3]);
//            model->FirstChildElement( "measurements" )->QueryDoubleAttribute("size", &measurementSize);
//            model->FirstChildElement( "measurements" )->QueryIntAttribute("featureIDMin", &measurementFeaturesIds.first);
//            model->FirstChildElement( "measurements" )->QueryIntAttribute("featureIDMax", &measurementFeaturesIds.second);
//            measurementsColor.setRedF(rgba[0]); measurementsColor.setGreenF(rgba[1]);
//            measurementsColor.setBlueF(rgba[2]); measurementsColor.setAlphaF(rgba[3]);
//            model->FirstChildElement( "measurements" )->QueryDoubleAttribute("ellipsoidRed", &rgba[0]);
//            model->FirstChildElement( "measurements" )->QueryDoubleAttribute("ellipsoidGreen", &rgba[1]);
//            model->FirstChildElement( "measurements" )->QueryDoubleAttribute("ellipsoidBlue", &rgba[2]);
//            model->FirstChildElement( "measurements" )->QueryDoubleAttribute("ellipsoidAlpha", &rgba[3]);
//            model->FirstChildElement( "measurements" )->QueryDoubleAttribute("ellipsoidScale", &ellipsoidScale);
//            model->FirstChildElement( "measurements" )->QueryBoolAttribute("drawEllipsoids", &drawEllipsoids);
//            ellipsoidColor.setRedF(rgba[0]); ellipsoidColor.setGreenF(rgba[1]);
//            ellipsoidColor.setBlueF(rgba[2]); ellipsoidColor.setAlphaF(rgba[3]);

//            model->FirstChildElement( "opencv" )->QueryBoolAttribute("showFrames", &showFrames);
        }
        public:
        /// Background color
        QColor backgroundColor;

    };

    /// Construction
    QGLVisualizer(void);

    /// Construction
    QGLVisualizer(std::string configFile);

    /// Construction
    QGLVisualizer(Config _config);

    /// Destruction
    ~QGLVisualizer(void);

    void update(const mapping::PointCloud& newCloud, std::vector<Mat33> uncertinatyErrors, bool isLast);

    void update(Octree<mapping::Voxel>& map, double res, std::unordered_map<std::string, Eigen::Vector3i> indexes, bool isLast);

    void update(mapping::Quaternion orientation, mapping::Vec3 translation);

private:
    double ellipsoidScale = 1.0;
    Config config;
    GLuint list;
    Octree<mapping::Voxel> map;
    std::unordered_map<std::string, Eigen::Vector3i> updatedVoxels;
    std::vector<Mat33> uncertinatyErrors;
    double angle = 0.0;
    Vec3 animationCenter;
    double animationRay;
    int shouldAnimate;
    float animationYOffset = 0.0;

    //pcl
    std::vector<mapping::PointCloud> pointClouds;

    /// mutex for critical section - point clouds
    std::mutex mtxPointCloud;

    /// draw objects
    void draw();

    /// draw objects
    void animate();

    /// initialize visualizer
    void init();

    /// generate help string
    std::string help() const;
    void keyPressEvent(QKeyEvent *e);

    /// Draw point clouds
    void drawPointCloud(void);

    void createDisplayList();

    void createMapDisplayList();

    void drawMap();

    void drawEllipsoid(const Vec3& pos, const Mat33& covariance, RGBA color) const;

    std::string currentDateTime();

    std::vector<std::string> split(const std::string &s, char delim);
};

#endif // QVISUALIZER_H_INCLUDED
