/* Test Proyect to register a chain of pointclouds*/

#include <iostream>
#include "../include/include.hpp"
#include "../include/planeDetection.hpp"
#include "../include/Registration.hpp"

#define INIT 1      //Initial Image
#define ITER 2		//Increment
#define MAX  52		//Max image

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

	// console vervosity for ICP functions
#ifdef _DEBUG
	setVerbosityLevel (pcl::console::L_ALWAYS);
#endif

	// Main Registration Loop
	pcl::visualization::PCLVisualizer viewer("Result Viewer");
	pcl::visualization::Camera cam;
	cam.clip[0]=2.7175;
	cam.clip[1] = 5.70627;
	cam.focal[0] = 0.0657875;
	cam.focal[1] = 0.101832;
	cam.focal[2] = 0.910947;
	cam.pos[0] = -0.396767;
	cam.pos[1] = -0.571929;
	cam.pos[2] = -3.04181;
	cam.view[0] = 0.28833;
	cam.view[1] = -0.948936;
	cam.view[2] = 0.128009;
	cam.fovy = 0.523599;
	cam.window_size[0] = 840;
	cam.window_size[1] = 525;
	cam.window_pos[0] = 1;
	cam.window_pos[1] = 52;
	viewer.setCameraParameters(cam);
	Modelization::Registration registrator(true,true,0.01,2000);
	i=0;
	while(files.size()>i)
	{
		timer.tic();
		if(registrator.addCloud(files[i])){
			i++;
			cout<<"Image "<<i<<endl;
			cout<< "advance"<<endl;
		}
		if(registrator.getResult(cloud_result)){
			cout<<"Refreshing View"<<endl;
			if(!viewer.updatePointCloud(cloud_result,"Resultant Cloud")){
				viewer.addPointCloud(cloud_result,"Resultant Cloud");
			}
		}
		viewer.spinOnce(100);

//		cout<<"registration "<<i<<": "<<timer.toc()<<" Miliseconds"<<endl;

	}

	viewer.spin();

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
