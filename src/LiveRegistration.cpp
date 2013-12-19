/*
 * LiveRegistration.cpp
 *
 *  Created on: Dec 17, 2013
 *      Author: jose
 */

#include "../include/LiveRegistration.h"

namespace Modelization {

LiveRegistration::LiveRegistration(): cloud_viewer_ (new pcl::visualization::PCLVisualizer ("PCL OpenNI cloud"))
, grabber_(){
}

LiveRegistration::~LiveRegistration() {
}

void LiveRegistration::cloud_callback (const PCXYZRGBConstPtr& cloud){



	boost::mutex::scoped_lock lock (cloud_mutex_);
	cloud_ = cloud;

}

void LiveRegistration::run ()
{
	//	Viewer initialization

	cloud_viewer_->setPosition (0, 0);
	cloud_viewer_->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	cloud_viewer_->createViewPort(0.5, 0.0, 1.0, 1.0, v2);

	boost::function<void (const PCXYZRGBConstPtr&) > cloud_cb = boost::bind (&LiveRegistration::cloud_callback, this, _1);
	boost::signals2::connection cloud_connection = grabber_.registerCallback (cloud_cb);

	Modelization::Registration registrator;


	grabber_.start ();

	PCXYZRGBPtr result;
	while (!cloud_viewer_->wasStopped ())
	{
		PCXYZRGBConstPtr cloud;

		// See if we can get a cloud if not try again
		if (cloud_mutex_.try_lock ()){
			cloud_.swap (cloud);
			cloud_mutex_.unlock ();
		}
		if(cloud){
			registrator.startThread(cloud);
			if(registrator.dataReady)// Finished Processing?
			{
				PCXYZRGBPtr Viewer_result;
				registrator.getResult(Viewer_result);
				cout<<"ShowingResult"<<endl;
				if (!cloud_viewer_->updatePointCloud (Viewer_result, "ResultCloud"))
				{
					pcl::io::savePCDFileASCII("results/threadedresult.pcd",*Viewer_result);
					cloud_viewer_->addPointCloud (Viewer_result, "ResultCloud",v1);
					cloud_viewer_->resetCameraViewpoint ("ResultCloud");
					cout<<"ShowingResult first time"<<endl;
				}
			}
			else // Start To process
			{
//				cout<<"Starting Process Main Thread"<<endl;

			}

			if (!cloud_viewer_->updatePointCloud (cloud, "OpenNICloud"))
			{
				cloud_viewer_->addPointCloud (cloud, "OpenNICloud",v2);
				cout<<"ShowingLive Feed"<<endl;
				cloud_viewer_->resetCameraViewpoint ("OpenNICloud");
			}
		}

		else
		{
//			cout<<"No Image"<<endl;
			continue;
		}

		cloud_viewer_->spinOnce(100);

	}

	grabber_.stop ();

	cloud_connection.disconnect ();
}


} /* namespace Modelization */
