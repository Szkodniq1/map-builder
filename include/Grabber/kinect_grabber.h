/** @file kinect_grabber.h
 *
 * implementation - Kinect Grabber
 *
 */

#ifndef KINECT_GRABBER_H_INCLUDED
#define KINECT_GRABBER_H_INCLUDED

#include "grabber.h"
#include "../../3rdParty/tinyXML/tinyxml2.h"
#include <iostream>
#include <memory>

#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

/// create a single grabber (Kinect)
Grabber* createGrabberKinect(void);
Grabber* createGrabberKinect(std::string configFile);


/// Grabber implementation
class KinectGrabber : public Grabber {
public:

    /// Pointer
    typedef std::unique_ptr<KinectGrabber> Ptr;

    /// Construction
    KinectGrabber(void);

    /// Construction
    KinectGrabber(std::string configFilename) : Grabber("Kinect Grabber", TYPE_PRIMESENSE), poseVis(Mat34::Identity()){
        tinyxml2::XMLDocument config;
        std::string filename = "../../resources/" + configFilename;
        config.LoadFile(filename.c_str());
        if (config.ErrorID())
            std::cout << "unable to load Kinect config file.\n";
        tinyxml2::XMLElement * posXML = config.FirstChildElement( "pose" );
        double query[4];
        posXML->QueryDoubleAttribute("qw", &query[0]); posXML->QueryDoubleAttribute("qx", &query[1]); posXML->QueryDoubleAttribute("qy", &query[2]); posXML->QueryDoubleAttribute("qz", &query[3]);
        double queryPos[4];
        posXML->QueryDoubleAttribute("x", &queryPos[0]); posXML->QueryDoubleAttribute("y", &queryPos[1]); posXML->QueryDoubleAttribute("z", &queryPos[2]);
        pose = Quaternion (query[0], query[1], query[2], query[3])*Vec3(queryPos[0], queryPos[1], queryPos[2]);
    }

    /// Name of the grabber
    virtual const std::string& getName() const;

    /// Returns current point cloud
    virtual const PointCloud& getCloud(void) const;

    /// Grab image and/or point cloud
    virtual void grab();

    /// Calibrate sensor
    virtual void calibrate(void);

    ///Sensor uninitialize
    virtual int grabberClose(void);

private:
    ///output from run() and LoadFromFile()
    pcl::PointCloud<pcl::PointXYZRGBA> cloud_temp;
    Mat34 poseVis;
    bool stopGrabber;
    /// callback function
    void cloud_callback (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);



};

#endif // KINECT_GRABBER_H_INCLUDED
