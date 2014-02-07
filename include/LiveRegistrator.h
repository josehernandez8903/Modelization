/*
 * LiveRegistrator.h
 *
 *  Created on: Feb 7, 2014
 *      Author: jose
 */

/*
 * LiveRegistrator.h
 *
 *  Created on: Dec 17, 2013
 *      Author: jose
 */

#ifndef LIVEREGISTRATOR_H_
#define LIVEREGISTRATOR_H_

#include <pcl/visualization/image_viewer.h>
#include <pcl/io/grabber.h>
#include <pcl/io/openni_grabber.h>

#include "include.hpp"
#include "Registration.hpp"

namespace Modelization {

class LiveRegistrator {
public:
	LiveRegistrator(pcl::OpenNIGrabber::Mode mode);
	virtual ~LiveRegistrator();
	void run ();
	void cloud_callback (const PCXYZRGBCPtr& cloud);


private:
	void initializeCamera(pcl::visualization::PCLVisualizer::Ptr & viewer);
	void initilizeSensor(pcl::OpenNIGrabber::Ptr & sensor);

	int v1;
	int v2;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;
	boost::shared_ptr<pcl::visualization::ImageViewer> image_viewer_;

	pcl::OpenNIGrabber::Ptr grabber_;
	pcl::OpenNIGrabber::Mode sensorMode_;
	boost::signals2::connection cloud_connection;

	boost::mutex cloud_mutex_;

	PCXYZRGBCPtr cloud_;
	boost::shared_ptr<openni_wrapper::Image> image_;
};

} /* namespace Modelization */
#endif /* LiveRegistrator_H_ */
