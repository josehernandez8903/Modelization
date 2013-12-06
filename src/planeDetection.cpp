/*
 * planeDetection.cpp
 *
 *  Created on: Dec 5, 2013
 *      Author: jose
 */

#include "../include/planeDetection.hpp"

namespace Modelization {

planeDetection::planeDetection() {
	srand((unsigned)time(0));
}

planeDetection::~planeDetection() {
}

void planeDetection::run(const pcl::PCLPointCloud2ConstPtr &cloud_blob, PCLXYZRGBPoint::Ptr cloud_filtered, PCLXYZRGBPoint::Ptr cloud_r){

	int RandR,RandG,RandB,it(0);

	pcl::PCLPointCloud2Ptr cloud_filtered_blob (new pcl::PCLPointCloud2);


	PCLXYZRGBPoint::Ptr cloud (new PCLXYZRGBPoint);
	PCLXYZRGBPoint::Ptr cloud_p = cloud_filtered;

	srand((unsigned)time(0));

	// Voxel Filetring
	voxel_filter(cloud_blob,0.01f,cloud_filtered);


	// Planar approximation
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.1);

	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
	while(true)
	{
		PCLXYZRGBPoint::Ptr cloud_f (new PCLXYZRGBPoint);
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloud_p);
		seg.segment (*inliers, *coefficients);
		if ((inliers->indices.size () == 0 )||cloud_p->size()<0.5*cloud_filtered->size())
		{
			std::cout << "Finished Planar Segmentation" << std::endl;
			break;
		}

		//	std::cerr << "Model coefficients: " << coefficients->values[0] << " "
		//			<< coefficients->values[1] << " "
		//			<< coefficients->values[2] << " "
		//			<< coefficients->values[3] << std::endl;

		std::cout << "Model inliers "<<it<<": " << inliers->indices.size () << std::endl;

		// Extract the inliers
		extract.setInputCloud (cloud_p);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (*cloud_f);
		std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
		std::stringstream str;
		str<<"results/Segmented_"<<it<<".pcd";
		pcl::io::savePCDFileASCII(str.str(),*cloud_f);

		RandR = static_cast<int>(255*(((double)rand())/RAND_MAX));
		RandG = static_cast<int>(255*(((double)rand())/RAND_MAX));
		RandB = static_cast<int>(255*(((double)rand())/RAND_MAX));


		for (size_t i = 0; i < cloud_f->size(); i++)
		{
			cloud_f->points[i].r = RandR;
			cloud_f->points[i].g = RandG;
			cloud_f->points[i].b = RandB;

			//    std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
			//                                               << cloud->points[inliers->indices[i]].y << " "
			//                                               << cloud->points[inliers->indices[i]].z << std::endl;
		}
		*cloud_r+=*cloud_f;
		extract.setNegative(true);
		extract.filter(*cloud_f);

		cloud_p=cloud_f;
		std::stringstream str2;
		str2<<"results/residue_"<<it++<<".pcd";
		pcl::io::savePCDFileASCII(str2.str(),*cloud_p);
	}
	*cloud_r+=*cloud_p;
}

void planeDetection::voxel_filter(const pcl::PCLPointCloud2ConstPtr &cloud_blob,float leaf_size, PCLXYZRGBPoint::Ptr cloud_filtered){

	pcl::PCLPointCloud2Ptr cloud_filtered_blob (new pcl::PCLPointCloud2);
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;

	sor.setInputCloud (cloud_blob);
	sor.setLeafSize (leaf_size, leaf_size, leaf_size);
	sor.filter (*cloud_filtered_blob);
	pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);
}

} /* namespace Modelization */
