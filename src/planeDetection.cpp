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

void planeDetection::run(const PCXYZRGBPtr& cloud_in
		, double percentage
		, PCXYZRGB& cloud_filtered
		, PCXYZRGB& cloud_r){

	int RandR,RandG,RandB,it(0);

	PCXYZRGBPtr cloud (new PCXYZRGB);
	PCXYZRGBPtr cloud_p (new PCXYZRGB(cloud_filtered));

	srand((unsigned)time(0));

	// Voxel Filetring
	voxel_filter(cloud_in,0.01f,cloud_filtered);

	// Planar approximation
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	seg.setOptimizeCoefficients (true);
	// Setup segmentation parameters
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.1);
	PCXYZRGBPtr cloud_f (new PCXYZRGB);
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	while(true)
	{

		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloud_p);
		seg.segment (*inliers, *coefficients);
		if ((inliers->indices.size () == 0 )||cloud_p->size()<percentage*cloud_filtered.size())
		{
			//std::cout << "Finished Planar Segmentation" << std::endl;
			break;
		}

		// Extract the inliers
		extract.setInputCloud (cloud_p);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (*cloud_f);

		//setup colors for segmented plane
		RandR = static_cast<int>(255*(((double)rand())/RAND_MAX));
		RandG = static_cast<int>(255*(((double)rand())/RAND_MAX));
		RandB = static_cast<int>(255*(((double)rand())/RAND_MAX));

		//paint the point cloud
		for (size_t i = 0; i < cloud_f->size(); i++)
		{
			cloud_f->points[i].r = RandR;
			cloud_f->points[i].g = RandG;
			cloud_f->points[i].b = RandB;
		}

		//save results
		cloud_r+=*cloud_f;
		extract.setNegative(true);
		extract.filter(*cloud_f);

		cloud_p=cloud_f;
		std::stringstream str2;
		str2<<"results/residue_"<<it++<<".pcd";
		pcl::io::savePCDFileASCII(str2.str(),*cloud_p);
		cloud_f.reset(new PCXYZRGB);
	}
	cloud_r+=*cloud_p;
}
} /* namespace Modelization */
