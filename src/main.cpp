#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

int
main (int argc, char** argv)
{

	int RandR,RandG,RandB,it(0);
	if(argc!=2)
	{
		std::cerr<<"Error: filename required"<<std::endl;
		return 0;

	}


	pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_p  = cloud_filtered;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_r (new pcl::PointCloud<pcl::PointXYZRGBA>);



	if(pcl::io::loadPCDFile(argv[1],*cloud_blob)!=0)
	{
		std::cerr<<"File not found or corrupt";
		return 0;
	}

	srand((unsigned)time(0));

	// Voxel Filetring
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloud_blob);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*cloud_filtered_blob);
	pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);
	// Planar approximation
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.1);




	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
	while(true)
	{
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGBA>);
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
		str<<"Segmented_"<<it<<".pcd";
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
		str2<<"residue_"<<it++<<".pcd";
		pcl::io::savePCDFileASCII(str2.str(),*cloud_p);
		cloud_f.reset();
	}
	*cloud_r+=*cloud_p;
	pcl::visualization::CloudViewer viewer("Cloud Viewer");

	//blocks until the cloud is actually rendered
	viewer.showCloud(cloud_r);





	while (!viewer.wasStopped ())
	{

	}
	return 0;
}
