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


	void run(PCLXYZRGBPointPtr &cloud_blob
			, PCLXYZRGBPointPtr cloud_filtered
			, PCLXYZRGBPointPtr cloud_r);


	static void voxel_filter(const PCLXYZRGBPointPtr &cloud_blob
			, float leaf_size
			, PCLXYZRGBPointPtr cloud_filtered);
};

} /* namespace Modelization */
#endif /* PLANEDETECTION_HPP_ */
