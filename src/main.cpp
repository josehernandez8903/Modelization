#include <iostream>

#include "../include/include.hpp"
#include "../include/planeDetection.hpp"
#include "../include/Registration.hpp"


int
main (int argc, char** argv)
{
	if(argc!=3)
	{
		std::cerr<<"Error: filename required"<<std::endl;
		return 0;
	}


	PCXYZRGBPtr cloud_result (new PCXYZRGB);

	PCXYZRGBPtr cloud_1 (new PCXYZRGB);
	PCXYZRGBPtr cloud_tg (new PCXYZRGB);


	PCXYZRGBPtr cloud_downsampled_1 (new PCXYZRGB);



	std::cout<<"Starting"<<std::endl;
	if(pcl::io::loadPCDFile(argv[1],*cloud_1)!=0 || pcl::io::loadPCDFile(argv[2],*cloud_tg)!=0)
	{
		std::cerr<<"File not found or corrupt";
		return 0;
	}
	std::cout<<"Finished Loading Files"<<std::endl;
	pcl::console::TicToc timer;

//	Modelization::planeDetection::voxel_filter(cloud_1,0.01f,cloud_downsampled_1);
	timer.tic();

	Modelization::Registration registrator(true,true);
	registrator.run(cloud_1,cloud_tg,*cloud_result);
//	registrator.estimateNormals(cloud_1,*cloud_in);

	cout<<"registration : "<<timer.toc()<<" Miliseconds"<<endl;

//	pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGBNormal> color_handler (cloud_in);
//	pcl::visualization::PCLVisualizer viewer("Downsampled");
//	viewer.setBackgroundColor (0.0, 0.0, 0.0);
//	viewer.addPointCloud<PointXYZRGBNormal>(cloud_in,color_handler,"Input_cloud2");
//	viewer.addPointCloudNormals<PointXYZRGBNormal,PointXYZRGBNormal>(cloud_in, cloud_in,20,0.05,"Input_Normals2");


//	while (!viewer.wasStopped ())
//	{
//		viewer.spinOnce (100);
//	}

	//	Modelization::planeDetection Modelizer;
	//	Modelizer.run(cloud_in,cloud_filtered,cloud_result);
	//	pcl::visualization::CloudViewer viewer("Cloud Viewer");

	//blocks until the cloud is actually rendered
	//	viewer.showCloud(cloud_result,"resultant Cloud");

	//	while (!viewer.wasStopped ())
	//	{
	//
	//	}
	return 0;
}
