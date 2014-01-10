#include <iostream>

#include "../include/include.hpp"
#include "../include/planeDetection.hpp"
#include "../include/Registration.hpp"

#define INIT 1
#define ITER 9
#define MAX 15

int
main (int argc, char** argv)
{
	if(argc<2)
	{
		std::cerr<<"Error: filename required"<<std::endl;
		return 0;
	}

	uint i;


	PCXYZRGBPtr cloud_result (new PCXYZRGB);
	PCXYZRGBPtr cloud_initial (new PCXYZRGB);
	PCXYZRGBPtr key1 (new PCXYZRGB);
	PCXYZRGBPtr key2 (new PCXYZRGB);

	PCXYZRGBPtr cloud_1 (new PCXYZRGB);

	PCXYZRGBPtr cloud_downsampled_1 (new PCXYZRGB);

	std::vector<PCXYZRGBPtr> files;

	std::cout<<"Starting"<<std::endl;
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

		//Debug
//		if(cloud_initial.get()==NULL)
//			cloud_initial = cloud_1->makeShared();
//		else
//		*cloud_initial+=*cloud_1;
		//Debug


		cloud_1.reset(new PCXYZRGB);

	}
	if(files.size()==0)
	{
		std::cerr<<"No files were loaded, verify input parameters";
		return -1;
	}

	std::cout<<"Finished Loading Files"<<std::endl;
	pcl::console::TicToc timer;

	//	Modelization::planeDetection::voxel_filter(cloud_1,0.01f,cloud_downsampled_1);


	Modelization::Registration registrator(true,true);
	i=0;
	while(files.size()>i)
	{
		timer.tic();
		cout<<"Image "<<i<<endl;
		registrator.runLoop(files[i++],*cloud_result);
		cout<<"registration "<<i<<": "<<timer.toc()<<" Miliseconds"<<endl;

	}

//	pcl::visualization::CloudViewer viewer2("Before");
//	viewer2.showCloud(cloud_initial,"Initial Cloud");
//	viewer2.showCloud(cloud_initial,"Initial Cloud");
//	while (!viewer2.wasStopped ())
//	{
//	}

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
