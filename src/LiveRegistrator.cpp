/*
 * LiveRegistrator.cpp
 *
 *  Created on: Feb 7, 2014
 *      Author: Jose Júan Hernández López
 */

#include "../include/LiveRegistrator.h"

namespace Modelization {

//*****************************************************************************************************************//
/* Default constructor with some initializations*/
//*****************************************************************************************************************//
LiveRegistrator::LiveRegistrator(pcl::OpenNIGrabber::Mode mode): v1(0),v2(0),sensorMode_(mode){
}

//*****************************************************************************************************************//
/* Default destructor with some initializations*/
//*****************************************************************************************************************//
LiveRegistrator::~LiveRegistrator() {
}

//*****************************************************************************************************************//
/* Image retrieving callback function*/
//*****************************************************************************************************************//
void LiveRegistrator::cloud_callback (const PCXYZRGBCPtr& cloud){
	boost::mutex::scoped_lock lock (cloud_mutex_); //Added for multithreading secureness
	cloud_ = cloud;
}

//*****************************************************************************************************************//
/* Viewer initialization Parameters */
//*****************************************************************************************************************//
void LiveRegistrator::initializeCamera(pcl::visualization::PCLVisualizer::Ptr & viewer){
	viewer.reset(new pcl::visualization::PCLVisualizer("Registration"));
	viewer->setPosition (0, 0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	pcl::visualization::Camera cam;
	cam.clip[0]=2.81783;
	cam.clip[1] = 23.4189;
	cam.focal[0] = 1.19782;
	cam.focal[1] = -0.0372366;
	cam.focal[2] = 6.063;
	cam.pos[0] = -1.23666;
	cam.pos[1] = -0.992101;
	cam.pos[2] = -5.58493;
	cam.view[0] = -0.00957236;
	cam.view[1] = -0.996446;
	cam.view[2] = 0.0836865;
	cam.fovy = 0.523599;
	cam.window_size[0] = 1463;
	cam.window_size[1] = 525;
	cam.window_pos[0] = 46;
	cam.window_pos[1] = 94;
	viewer->setCameraParameters(cam,v1);
	viewer->setCameraParameters(cam,v2);

}

void LiveRegistrator::initilizeSensor(pcl::OpenNIGrabber::Ptr & sensor){
	std::string device_id = "";
	sensor.reset(new pcl::OpenNIGrabber(device_id, sensorMode_, sensorMode_));
	boost::function<void (const PCXYZRGBCPtr&) > cloud_cb = boost::bind (&LiveRegistrator::cloud_callback, this, _1);
	cloud_connection = sensor->registerCallback (cloud_cb);
}

//*****************************************************************************************************************//
/* Start the Regitration process with the visualization as the main thread*/
//*****************************************************************************************************************//
void LiveRegistrator::run ()
{

	PCXYZRGBPtr Viewer_result;
	//	Viewer initialization
	initializeCamera(cloud_viewer_);

	// Senso initialization and start
	initilizeSensor(grabber_);
	grabber_->start();

	// Initialization of the registration object
	Modelization::Registration registrator(true, true, 0.09, 1000);
	PCXYZRGBPtr result;

	while (!cloud_viewer_->wasStopped ())
	{
		PCXYZRGBCPtr cloud;		// input cloud

		// See if we can get a cloud if not try again
		if (cloud_mutex_.try_lock ()){
			cloud_.swap (cloud);
			cloud_mutex_.unlock ();
		}

		// Run if a cloud is available
		if(cloud){
			// Start the Registration process using cloud
			if(registrator.addCloud(cloud)){
				cout<<"Added Cloud";
			}
			// Finished Processing?
			if(registrator.getResult(Viewer_result))
			{
				// Get the resulting image.
				cout<<"ShowingResult"<<endl;
				//				// Create view or update resulting cloud view depending on if its the first cloud or not.
				if (!cloud_viewer_->updatePointCloud(Viewer_result, "ResultCloud"))
				{
					//pcl::io::savePCDFileASCII("results/threadedresult.pcd",*Viewer_result);
					cloud_viewer_->addPointCloud(Viewer_result, "ResultCloud",v1);
					cout<<"ShowingResult first time"<<endl;
				}
			}
			// Create view or update input cloud view depending on if its the first cloud or not. (Video Feed)
			if (!cloud_viewer_->updatePointCloud (cloud, "OpenNICloud"))
			{
				cloud_viewer_->addPointCloud (cloud, "OpenNICloud",v2);
				//				cout<<"ShowingLive Feed"<<endl;
			}
		}

		// if no cloud is available do nothing placeholder for debug purposes.
		else
		{
			//			cout<<"No Image"<<endl;
			continue;
		}

		cloud_viewer_->spinOnce();

	}

	//When the window is closed stop the grabber and finish the function.
	grabber_->stop ();
	cloud_connection.disconnect ();
	cout<<"The End"<<endl;
}

} /* namespace Modelization */

