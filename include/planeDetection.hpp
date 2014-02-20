/*
 * planeDetection.hpp
 *
 *  Created on: Dec 5, 2013
 *  Author: Jose Juan Hernandez Lopez
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

/**
 * \brief planar detection elimination class, namely the floor.
 */
class planeDetection {
public:
	planeDetection();
	virtual ~planeDetection();


	/**
	 * \brief detects and extracts the largests planes u to a percentage of the image
	 * \param[in] cloud_in Input cloud
	 * \param[in] percentage Maximum cloud percentage allowed 0.0->1.0
	 * \param[out] cloud_filtered Non segmented voxel filtered cloud result
	 * \param[out] cloud_r Segmented voxel filtered cloud result
	 */
	void run(const PCXYZRGBPtr& cloud_in
			, double percentage
			, PCXYZRGB& cloud_filtered
			, PCXYZRGB& cloud_r);

	/**
	 * \brief template function that applies a squared voxel filter
	 * \param[in] cloud_in Input cloud
	 * \param[in] leaf_size Box size in meters
	 * \param[out] cloud_filtered Filtered cloud
	 */
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
