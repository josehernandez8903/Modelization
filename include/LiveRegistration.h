/*
 * LiveRegistration.h
 *
 *  Created on: Dec 17, 2013
 *      Author: jose
 */

#ifndef LIVEREGISTRATION_H_
#define LIVEREGISTRATION_H_

#include <pcl/visualization/image_viewer.h>
#include <pcl/io/grabber.h>
#include <pcl/io/openni_grabber.h>

#include "include.hpp"
#include "Registration.hpp"

namespace Modelization {

class LiveRegistration {
public:
	LiveRegistration();
	virtual ~LiveRegistration();
	void run ();
	void cloud_callback (const PCXYZRGBConstPtr& cloud);


private:

	int v1;
	int v2;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;
	boost::shared_ptr<pcl::visualization::ImageViewer> image_viewer_;

	pcl::OpenNIGrabber grabber_;
	boost::mutex cloud_mutex_;

	PCXYZRGBConstPtr cloud_;
	boost::shared_ptr<openni_wrapper::Image> image_;
};

} /* namespace Modelization */
#endif /* LIVEREGISTRATION_H_ */
