/*
 * LiveRegistration.cpp
 *
 *  Created on: Dec 17, 2013
 *      Author: jose
 */

#include "../include/LiveRegistration.h"

namespace Modelization {

//*****************************************************************************************************************//
/* Default constructor with some initializations*/
//*****************************************************************************************************************//
LiveRegistration::LiveRegistration(): cloud_viewer_ (new pcl::visualization::PCLVisualizer ("PCL OpenNI cloud"))
, grabber_(),v1(0),v2(0){
}

//*****************************************************************************************************************//
/* Default destructor with some initializations*/
//*****************************************************************************************************************//
LiveRegistration::~LiveRegistration() {
}

//*****************************************************************************************************************//
/* Image retrieving callback function*/
//*****************************************************************************************************************//
void LiveRegistration::cloud_callback (const PCXYZRGBConstPtr& cloud){
	boost::mutex::scoped_lock lock (cloud_mutex_); //Added for multithreading secureness
	cloud_ = cloud;
}

//*****************************************************************************************************************//
/* Start the Regitration process with the visualization as the main thread*/
//*****************************************************************************************************************//
void LiveRegistration::run ()
{
	//	Viewer initialization
	cloud_viewer_->setPosition (0, 0);
	cloud_viewer_->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	cloud_viewer_->createViewPort(0.5, 0.0, 1.0, 1.0, v2);

	// Cloud Grabber callback registration and start
	boost::function<void (const PCXYZRGBConstPtr&) > cloud_cb = boost::bind (&LiveRegistration::cloud_callback, this, _1);
	boost::signals2::connection cloud_connection = grabber_.registerCallback (cloud_cb);
	grabber_.start();

	// Initialization of the registration object
	Modelization::Registration registrator;
	PCXYZRGBPtr result;

	while (!cloud_viewer_->wasStopped ())
	{
		PCXYZRGBConstPtr cloud;		// input cloud

		// See if we can get a cloud if not try again
		if (cloud_mutex_.try_lock ()){
			cloud_.swap (cloud);
			cloud_mutex_.unlock ();
		}

		// Run if a cloud is available
		if(cloud){
			// Start the Registration process using cloud
			registrator.startThread(cloud);
			// Finished Processing?
			if(registrator.dataReady)
			{
				// Get the resulting image.
				PCXYZRGBPtr Viewer_result;
				registrator.getResult(Viewer_result);
				//cout<<"ShowingResult"<<endl;

				// Create view or update resulting cloud view depending on if its the first cloud or not.
				if (!cloud_viewer_->updatePointCloud<PointXYZRGB>(Viewer_result, "ResultCloud"))
				{
					//pcl::io::savePCDFileASCII("results/threadedresult.pcd",*Viewer_result);
					cloud_viewer_->addPointCloud<PointXYZRGB>(Viewer_result, "ResultCloud",v1);
					cloud_viewer_->resetCameraViewpoint ("ResultCloud");
					cout<<"ShowingResult first time"<<endl;
				}
			}
			// if the process hasn't finished o nothing : place holder for a debug code
			else
			{
//				cout<<"Still procesing"<<endl;

			}

			// Create view or update input cloud view depending on if its the first cloud or not. (Video Feed)
			if (!cloud_viewer_->updatePointCloud (cloud, "OpenNICloud"))
			{
				cloud_viewer_->addPointCloud (cloud, "OpenNICloud",v2);
				cout<<"ShowingLive Feed"<<endl;
				cloud_viewer_->resetCameraViewpoint ("OpenNICloud");
			}
		}

		// if no cloud is available do nothing placeholder for debug purposes.
		else
		{
//			cout<<"No Image"<<endl;
			continue;
		}

		cloud_viewer_->spinOnce(100);

	}

	//When the window is closed stop the grabber and finish the function.
	grabber_.stop ();
	cloud_connection.disconnect ();
}

} /* namespace Modelization */
