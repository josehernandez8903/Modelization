/* Test Proyect to register a chain of pointclouds*/

#include <iostream>
#include <stdlib.h>
#include "../include/include.hpp"
#include "../include/planeDetection.hpp"
#include "../include/Registration.hpp"


// Darwin_final_1_4-14, 35-45, 98-103

//#define INIT 101      //Initial Image
//#define ITER 1		//Increment
//#define MAX 103		//Max image

int
main (int argc, char** argv)
{
	//Check for name string before the numeration
	if(argc<3)
	{
		std::cerr<<"Error: filename required"<<std::endl;
		std::cerr<<"Initial cloud number, Last cloud Number and increment required"<<std::endl;
		std::cerr<<"example: main Darwin_final_1_ 10 20 2"<<std::endl;
		std::cerr<<"From Darwin_final_1_10.pcd to Darwin_final_1_20.pcd every other cloud"<<std::endl;
		return 0;
	}

	const uint INIT = atoi(argv[2]);
	const uint MAX = atoi(argv[3]);
	const uint ITER = atoi(argv[4]);

	uint i;

	PCXYZRGBPtr cloud_result (new PCXYZRGB);			//Resulting cloud
	PCXYZRGBPtr cloud_result2 (new PCXYZRGB);
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

	// console vervosity for ICP functions
#ifdef _DEBUG
	setVerbosityLevel (pcl::console::L_ALWAYS);
#endif

	// Main Registration Loop
	Modelization::Registration registrator(true,true,10000);
	i=0;
	while(files.size()>i)
	{
		timer.tic();
		cout<<"Image "<<i<<endl;
		registrator.runLoop(files[i++]);
		cout<<"registration "<<i<<": "<<timer.toc()<<" Miliseconds"<<endl;

	}
	cout<<"Get View"<<endl;
	timer.tic();

	//Get views
	registrator.getView(*cloud_result,false,false); //do not apply transformation
	registrator.getView(*cloud_result2,true,true);	//Apply transformations quaternions and force update
	cout<<"Get View took :  "<<timer.toc()<<" Miliseconds"<<endl;
	//	pcl::visualization::CloudViewer viewer2("Before");
	//	viewer2.showCloud(cloud_initial,"Initial Cloud");
	//	viewer2.showCloud(cloud_initial,"Initial Cloud");
	//	while (!viewer2.wasStopped ())
	//	{
	//	}

	// Display the resulting cloud

	//		pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGBNormal> color_handler (cloud_in);
	//		pcl::visualization::PCLVisualizer viewer("Downsampled");
	//		viewer.setBackgroundColor (0.0, 0.0, 0.0);
	pcl::visualization::PCLVisualizer viewer("Non Registered");
	pcl::visualization::PCLVisualizer viewer2("Registered");

	viewer.addPointCloud(cloud_result,"Input_cloud2");
	viewer2.addPointCloud(cloud_result2,"Input_cloud");
	//		viewer.addPointCloudNormals<PointXYZRGBNormal,PointXYZRGBNormal>(cloud_in, cloud_in,20,0.05,"Input_Normals2");

	viewer2.spin();

	//	Modelization::planeDetection Modelizer;
	//	Modelizer.run(cloud_in,cloud_filtered,cloud_result);
	//	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	//	viewer.showCloud(cloud_result,"resultant Cloud");

	cout << "finished";
	return 0;
}
