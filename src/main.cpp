#include <iostream>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>

#include "../include/include.hpp"
#include "../include/planeDetection.hpp"


int
main (int argc, char** argv)
{
	if(argc!=3)
	{
		std::cerr<<"Error: filename required"<<std::endl;
		return 0;
	}

	CloudNormalPtr cloud_in(new CloudNormal);
	PCLXYZRGBPointPtr cloud_tg(new PCLXYZRGBPoint);

	PCLXYZRGBPointPtr cloud_filtered (new PCLXYZRGBPoint);
	PCLXYZRGBPointPtr cloud_result (new PCLXYZRGBPoint);

	PCLXYZRGBPointPtr cloud_1 (new PCLXYZRGBPoint);
	PCLXYZRGBPointPtr cloud_downsampled_1 (new PCLXYZRGBPoint);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);

	std::cout<<"Starting"<<std::endl;
	if(pcl::io::loadPCDFile(argv[1],*cloud_1)!=0 || pcl::io::loadPCDFile(argv[2],*cloud_tg)!=0)
	{
		std::cerr<<"File not found or corrupt";
		return 0;
	}
	std::cout<<"Finished Loading Files"<<std::endl;
	pcl::console::TicToc timer;

	//  ... create a downsampled version of it ...
	//http://pointclouds.org/documentation/tutorials/normal_estimation_using_integral_images.php#normal-estimation-using-integral-images


	Modelization::planeDetection::voxel_filter(cloud_1,0.01f,cloud_downsampled_1);

	// Create the normal estimation class, and pass the input dataset to it

	//	timer.tic();
	//	pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal> ne1;
	//	ne1.setInputCloud (cloud_downsampled_1);
	//	// Create an empty kdtree representation, and pass it to the normal estimation object.
	//	// Its content will be filled inside the object, based on the given surface dataset.
	//	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
	//	ne1.setSearchMethod (tree);
	//	// Output datasets
	//	// Use all neighbors in a sphere of radius 3cm
	//	ne1.setRadiusSearch (0.1);
	//	// Compute the features
	//	ne1.compute (*cloud_normals);
	//	std::cout<<"Unorganized Data "<<timer.toc()<<" ms"<<std::endl;


	timer.tic();
	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
	ne.setMaxDepthChangeFactor(0.01f);
	ne.setNormalSmoothingSize(10.0f);
	ne.setInputCloud(cloud_1);
	//  ne.setIndices()
	ne.compute(*cloud_normals2);
	pcl::concatenateFields (*cloud_normals2, *cloud_1, *cloud_in);
	std::cout<<"Organized Data "<<timer.toc()<<" ms"<<std::endl;

	pcl::visualization::PCLVisualizer viewer("Organized Data");
	viewer.setBackgroundColor (0.0, 0.0, 0.0);
	viewer.addPointCloud(cloud_downsampled_1,"Input_cloud");
	viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(cloud_1, cloud_normals2,20,0.05,"Input_Normals");

	//todo : chech if the Cloud point has all the normals.

	//	pcl::visualization::PCLVisualizer viewer2("Downsampled");
	//		viewer2.setBackgroundColor (0.0, 0.0, 0.0);
	//		viewer2.addPointCloud(cloud_downsampled_1,"Input_cloud2");
	//		viewer2.addPointCloudNormals<pcl::PointXYZRGBA,pcl::Normal>(cloud_downsampled_1, cloud_normals,20,0.05,"Input_Normals2");

	while (!viewer.wasStopped ())
	{
		viewer.spinOnce ();
		//		viewer2.spinOnce ();
	}
	return 0;

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
