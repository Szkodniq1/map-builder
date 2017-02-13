#include "../include/Grabber/kinect_grabber.h"
#include <memory>
#include <stdexcept>
#include <chrono>
#include <thread>

/// A single instance of Kinect grabber
KinectGrabber::Ptr grabberKinect;

KinectGrabber::KinectGrabber(void) : Grabber("Kinect Grabber", TYPE_PRIMESENSE), poseVis(Mat34::Identity()) {
    stopGrabber = false;
}

const std::string& KinectGrabber::getName() const {
	return name;
}

const PointCloud& KinectGrabber::getCloud(void) const {
    return cloud;
}

void KinectGrabber::grab(void) {
    pcl::Grabber* interface = new pcl::OpenNIGrabber();

    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
    boost::bind (&KinectGrabber::cloud_callback, this, _1);
    interface->registerCallback (f);
    interface->start ();
    while (!stopGrabber)
        boost::this_thread::sleep(boost::posix_time::seconds(1));
    interface->stop ();
}

void KinectGrabber::calibrate(void) {

}

int KinectGrabber::grabberClose(){
    stopGrabber = true;
    return 0;
}

Grabber* createGrabberKinect(void) {
    grabberKinect.reset(new KinectGrabber());
    return grabberKinect.get();
}

Grabber* createGrabberKinect(std::string configFile) {
    grabberKinect.reset(new KinectGrabber(configFile));
    return grabberKinect.get();
}

void KinectGrabber::cloud_callback (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud){
    cloud_temp = *cloud;
}

