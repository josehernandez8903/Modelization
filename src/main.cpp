#include <iostream>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
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

	PCLXYZRGBPointPtr cloud_in(new PCLXYZRGBPoint);
	PCLXYZRGBPointPtr cloud_tg(new PCLXYZRGBPoint);

	PCLXYZRGBPoint::Ptr cloud_filtered (new PCLXYZRGBPoint);
	PCLXYZRGBPoint::Ptr cloud_result (new PCLXYZRGBPoint);


	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_downsampled_1 (new pcl::PointCloud<pcl::PointXYZRGBA>);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	std::cout<<"Starting"<<std::endl;
	if(pcl::io::loadPCDFile(argv[1],*cloud_1)!=0 || pcl::io::loadPCDFile(argv[2],*cloud_tg)!=0)
	{
		std::cerr<<"File not found or corrupt";
		return 0;
	}
	std::cout<<"Finished Loading Files"<<std::endl;

	//  ... create a downsampled version of it ...
	//todo : set index for search in normal estimation using fastest algorithm available
	//http://pointclouds.org/documentation/tutorials/normal_estimation_using_integral_images.php#normal-estimation-using-integral-images
	boost::shared_ptr<std::vector<int> > indicesptr (new std::vector<int>);
	Modelization::planeDetection::voxel_filter(cloud_1,0.01f,cloud_downsampled_1);

//	// Create the normal estimation class, and pass the input dataset to it
//	pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
//	ne.setInputCloud (cloud_downsampled_1);
//
//
//	// Create an empty kdtree representation, and pass it to the normal estimation object.
//	// Its content will be filled inside the object, based on the given surface dataset.
//	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
//	ne.setSearchMethod (tree);
//
//	// Output datasets

//
//	// Use all neighbors in a sphere of radius 3cm
//	ne.setRadiusSearch (0.1);
//
//	// Compute the features
//	ne.compute (*cloud_normals);

    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud_1);
    ne.setIndices()

    ne.compute(*cloud_normals);


	std::cout<<"Downsampled size "<<cloud_downsampled_1->size()
					 <<"  Normals size "<<  cloud_normals->points.size ()<< std::endl;

	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor (0.0, 0.0, 0.0);
	viewer.addPointCloud(cloud_downsampled_1,"Input_cloud");
	viewer.addPointCloudNormals<pcl::PointXYZRGBA,pcl::Normal>(cloud_1, cloud_normals,20,0.09,"Input_Normals");

	while (!viewer.wasStopped ())
	{
		viewer.spinOnce ();
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
