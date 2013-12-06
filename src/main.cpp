#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include "../include/include.hpp"
#include "../include/planeDetection.hpp"


int
main (int argc, char** argv)
{
	if(argc!=2)
	{
		std::cerr<<"Error: filename required"<<std::endl;
		return 0;
	}

	pcl::PCLPointCloud2Ptr cloud_blob(new pcl::PCLPointCloud2);
	PCLXYZRGBPoint::Ptr cloud_filtered (new PCLXYZRGBPoint);
	PCLXYZRGBPoint::Ptr cloud_result (new PCLXYZRGBPoint);
	Modelization::planeDetection Modelizer;


	if(pcl::io::loadPCDFile(argv[1],*cloud_blob)!=0)
	{
		std::cerr<<"File not found or corrupt";
		return 0;
	}

	Modelizer.run(cloud_blob,cloud_filtered,cloud_result);
	pcl::visualization::CloudViewer viewer("Cloud Viewer");

	//blocks until the cloud is actually rendered
	viewer.showCloud(cloud_result);

	while (!viewer.wasStopped ())
	{

	}
	return 0;
}
