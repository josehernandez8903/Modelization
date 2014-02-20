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

/**
 * \brief Live registration class using an OpenNI sensor
 */
class LiveRegistrator {
public:

	/**
	 * \brief constructor
	 * \param[in] mode Sensor mode
	 * \note possible options are pcl::OpenNIGrabber::OpenNI_QVGA_30Hz or pcl::OpenNIGrabber::OpenNI_VGA_30Hz
	 */
	LiveRegistrator(pcl::OpenNIGrabber::Mode mode);

	/**
	 * \brief default destructor
	 */
	virtual ~LiveRegistrator();

	/**
	 * \brief Initialize the capture and registration
	 */
	void run ();

	/**
	 * \brief Internal callback to retrieve the cloud from the sensor
	 * \param[out] cloud Return cloud
	 */
	void cloud_callback (const PCXYZRGBCPtr& cloud);


private:
	/**
	 * \brief Misc visualizer parameters settings
	 * \param[in] viewer visualizer to modify
	 */
	void initializeCamera(pcl::visualization::PCLVisualizer::Ptr & viewer);

	/**
	 * \brief sensor initialzations
	 * \param[in] sensor Sensor to initialize
	 */
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
