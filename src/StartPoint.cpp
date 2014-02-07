
/*
 * Starting Point
 *
 *  Created on: Feb 7, 2014
 *      Author: José Juan Hernández López
 *      biref: Main live registration function
 */

#include "../include/include.hpp"
#include "../include/planeDetection.hpp"
#include "../include/LiveRegistrator.h"

int
main (int argc, char** argv)
{
#ifdef _DEBUG
	setVerbosityLevel (pcl::console::L_ALWAYS);
#endif

	Modelization::LiveRegistrator registrator(pcl::OpenNIGrabber::OpenNI_QVGA_30Hz);
	registrator.run();


	cout << "finished";
	return 0;
}


