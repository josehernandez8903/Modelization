/* Test Proyect to register a chain of pointclouds*/

#include <iostream>
#include "../include/include.hpp"
#include "../include/planeDetection.hpp"
#include "../include/Registration.hpp"

#define INIT 1      //Initial Image
#define ITER 1		//Increment
#define MAX 30		//Max image

int
main (int argc, char** argv)
{
	//Check for name string before the numeration
	if(argc<2)
	{
		std::cerr<<"Error: filename required"<<std::endl;
		return 0;
	}

	uint i;

	PCXYZRGBPtr cloud_result (new PCXYZRGB);			//Resulting cloud
	PCXYZRGBPtr cloud_1 (new PCXYZRGB);					//temporal reading cloud
	std::vector<PCXYZRGBPtr> files;						//Containing all the read clouds

	std::cout<<"Starting"<<std::endl;

	// Load all images for faster processing
	for (i=INIT;i<=MAX;i+=ITER)
	{
		std::ostringstream fname;
		fname<<argv[1]<<i<<".pcd";
		if(pcl::io::loadPCDFile(fname.str(),*cloud_1)!=0)
		{
			std::cerr<<"File "<< fname.str()<<" not found or corrupt, Skipping...";
			continue;
		}
		cout<<"Loaded "<<fname.str()<<endl;
		files.push_back(cloud_1);

		cloud_1.reset(new PCXYZRGB);
	}

	// Verify for at least 1 image loaded
	if(files.size()==0)
	{
		std::cerr<<"No files were loaded, verify input parameters";
		return -1;
	}

	std::cout<<"Finished Loading Files"<<std::endl;
	pcl::console::TicToc timer;

	// Main Registration Loop
	Modelization::Registration registrator(true,true);
	i=0;
	while(files.size()>i)
	{
		timer.tic();
		cout<<"Image "<<i<<endl;
		registrator.runLoop(files[i++]);
		cout<<"registration "<<i<<": "<<timer.toc()<<" Miliseconds"<<endl;

	}
	registrator.getView(*cloud_result);

//	pcl::visualization::CloudViewer viewer2("Before");
//	viewer2.showCloud(cloud_initial,"Initial Cloud");
//	viewer2.showCloud(cloud_initial,"Initial Cloud");
//	while (!viewer2.wasStopped ())
//	{
//	}

	// Display the resulting cloud
	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	viewer.showCloud(cloud_result,"resultant Cloud");
	while (!viewer.wasStopped ())
	{
	}

//		pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGBNormal> color_handler (cloud_in);
//		pcl::visualization::PCLVisualizer viewer("Downsampled");
//		viewer.setBackgroundColor (0.0, 0.0, 0.0);
//		viewer.addPointCloud<PointXYZRGBNormal>(cloud_in,color_handler,"Input_cloud2");
//		viewer.addPointCloudNormals<PointXYZRGBNormal,PointXYZRGBNormal>(cloud_in, cloud_in,20,0.05,"Input_Normals2");
//		while (!viewer.wasStopped ())
//		{
//			viewer.spinOnce (100);
//		}

//	Modelization::planeDetection Modelizer;
//	Modelizer.run(cloud_in,cloud_filtered,cloud_result);
//	pcl::visualization::CloudViewer viewer("Cloud Viewer");
//	viewer.showCloud(cloud_result,"resultant Cloud");

	cout << "finished";
	return 0;
}
