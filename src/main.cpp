#include <iostream>

#include "../include/include.hpp"
#include "../include/planeDetection.hpp"
#include "../include/LiveRegistration.h"

#define INIT 1
#define ITER 3
#define MAX 4

int
main (int argc, char** argv)
{
#ifdef _DEBUG
	setVerbosityLevel (pcl::console::L_ALWAYS);
#endif
//	if(argc<2)
//	{
//		std::cerr<<"Error: filename required"<<std::endl;
//		return 0;
//	}
//
//	uint i;
//
//
//	PCXYZRGBPtr cloud_result (new PCXYZRGB);
//
//	PCXYZRGBPtr cloud_1 (new PCXYZRGB);
//
//	PCXYZRGBPtr cloud_downsampled_1 (new PCXYZRGB);
//
//	std::vector<PCXYZRGBPtr> files;
//
//	std::cout<<"Starting"<<std::endl;
//	for (i=INIT;i<=MAX;i++)
//	{
//		std::ostringstream fname;
//		fname<<argv[1]<<i<<".pcd";
//		if(pcl::io::loadPCDFile(fname.str(),*cloud_1)!=0)
//		{
//			std::cerr<<"File "<< fname.str()<<" not found or corrupt, Skipping...";
//			continue;
//		}
//		cout<<"Loaded "<<fname.str()<<endl;
//		files.push_back(cloud_1);
//		cloud_1.reset(new PCXYZRGB);
//	}
//	if(files.size()==0)
//	{
//		std::cerr<<"No files were loaded, verify input parameters";
//		return -1;
//	}
//
//	std::cout<<"Finished Loading Files"<<std::endl;
//	pcl::console::TicToc timer;

	//	Modelization::planeDetection::voxel_filter(cloud_1,0.01f,cloud_downsampled_1);


	Modelization::LiveRegistration registrator;
	registrator.run();
//	i=0;
//	while(files.size()>i)
//	{
//		timer.tic();
//		registrator.runLoop(files[i++],*cloud_result);
//		cout<<"Image "<<i<<endl;
//		cout<<"registration "<<i<<": "<<timer.toc()<<" Miliseconds"<<endl;
//
//	}
//	pcl::visualization::CloudViewer viewer("Cloud Viewer");
//	viewer.showCloud(cloud_result,"resultant Cloud");
//	while (!viewer.wasStopped ())
//	{
//	}

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
