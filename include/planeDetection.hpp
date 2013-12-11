/*
 * planeDetection.hpp
 *
 *  Created on: Dec 5, 2013
 *      Author: Jose Juan Hernandez Lopez
 */

#ifndef PLANEDETECTION_HPP_
#define PLANEDETECTION_HPP_

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include "include.hpp"
namespace Modelization {

class planeDetection {
public:
	planeDetection();
	virtual ~planeDetection();


	void run(const PCXYZRGBPtr& cloud_in
			, PCXYZRGB& cloud_filtered
			, PCXYZRGB& cloud_r);

	template <typename PointT>
	static void voxel_filter(const typename pcl::PointCloud<PointT>::ConstPtr& cloud_in
			, float leaf_size
			,typename pcl::PointCloud<PointT>& cloud_filtered)
	{
		pcl::VoxelGrid<PointT> sor;
		sor.setInputCloud (cloud_in);
		sor.setLeafSize (leaf_size, leaf_size, leaf_size);
		sor.filter (cloud_filtered);
	}
};

} /* namespace Modelization */
#endif /* PLANEDETECTION_HPP_ */
