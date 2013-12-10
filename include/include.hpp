/*
 * include.hpp
 *
 *  Created on: Dec 5, 2013
 *      Author: jose
 */

#ifndef INCLUDE_HPP_
#define INCLUDE_HPP_

#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/common/angles.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

//XYZ and RGB Pointcloud

typedef pcl::PointCloud<pcl::PointXYZRGB> PCLXYZRGBPoint;
typedef PCLXYZRGBPoint::Ptr PCLXYZRGBPointPtr;

//XYZ and normal Pointcloud
typedef pcl::PointXYZRGBNormal pointRGBNormal;
typedef pcl::PointCloud<pointRGBNormal> CloudNormal;
typedef CloudNormal::Ptr CloudNormalPtr;


#endif /* INCLUDE_HPP_ */
