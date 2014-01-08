/*
 * Registration.hpp
 *
 *  Created on: Dec 6, 2013
 *      Author: José Juan Hernández López
 */

#ifndef REGISTRATION_HPP_
#define REGISTRATION_HPP_

#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/sample_consensus/prosac.h>
#include <pcl/sample_consensus/ransac.h>

#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>

#include <pcl/registration/default_convergence_criteria.h>

#include <boost/iterator/counting_iterator.hpp>

#include "planeDetection.hpp"
#include "include.hpp"

//**************************************************
// Version 2 Includes
//**************************************************
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_svd.h>

namespace Modelization {
using pcl::Normal;
//Correspondences
typedef std::vector< pcl::Correspondence, Eigen::aligned_allocator<pcl::Correspondence> > Correspondences;
typedef boost::shared_ptr<Correspondences> CorrespondencesPtr;

class Registration {
public:
	Registration(bool _rejection = true, bool _reiprocal = true);
	virtual ~Registration();

	void runLoop(const PCXYZRGBPtr &_in
				, PCXYZRGB &cloud_r);

	void runLoop2(const PCXYZRGBPtr &_in
			, PCXYZRGB &cloud_r);

	void run(const PCXYZRGBPtr &_src
			, const PCXYZRGBPtr &_tgt
			, PCXYZRGB &cloud_r);

	void estimateKeypoints (const PCNormalPtr &cloud
			, PCNormalPtr &keypoints);


private:
	void rejectBadCorrespondences (const CorrespondencesPtr& all_correspondences,
			const PCNormalPtr& src,
			const PCNormalPtr& tgt,
			Correspondences& remaining_correspondences);

	void findCorrespondences (const PCNormalPtr& src,
			const PCNormalPtr& tgt,
			Correspondences& all_correspondences);

	void findTransformation (const PCNormalPtr& src,
			const PCNormalPtr& tgt,
			const CorrespondencesPtr& correspondences,
			Eigen::Matrix4d &transform);

	void view (const PCNormal::ConstPtr& src,
			const PCNormal::ConstPtr& tgt,
			const CorrespondencesPtr& correspondences);

	void view (const PCXYZRGB::ConstPtr &src
			, const PCXYZRGB::ConstPtr &tgt);

	void estimateNormals(const PCXYZRGBPtr& cloud_in
			, PCNormal& cloud_out
			, bool _downsample
			, float _leaf_size = 0.01f);

	void icp (const PCNormalPtr& src,
			const PCNormalPtr& tgt,
			Eigen::Matrix4d& transform);

	int Ransacregistration (const PCNormalPtr &src
			, const PCNormalPtr &tgt
			, Eigen::Matrix4d &transform);



	void estimateFPFH (const PCNormalPtr &cloud
			, const PCNormalPtr &keypoints
			, PCFPFHPtr &fpfhs);

	void findCorrespondences (const PCFPFHPtr &fpfhs_src
			, const PCFPFHPtr &fpfhs_tgt
			, Correspondences &all_correspondences);

	void rejectBadCorrespondencesDistance (const CorrespondencesPtr &all_correspondences,
	                          const PCNormalPtr &keypoints_src,
	                          const PCNormalPtr &keypoints_tgt,
	                          Correspondences &remaining_correspondences);

	void computeTransformation(const PCNormalPtr &src
			, const PCNormalPtr &tgt
			, const PCNormalPtr &keypoints_src
			, const PCNormalPtr &keypoints_tgt
			, const PCFPFHPtr & fpfhs_src
			, const PCFPFHPtr & fpfhs_tgt
			, Eigen::Matrix4d &transform);

	void visualizeKeypoints(const PCNormalPtr &cloud
				, const PCNormalPtr &kpoint);

	void visualizeCorrespondances(const PCNormalPtr &cloud_src
			, const PCNormalPtr &cloud_tgt
			, const PCNormalPtr &keypoints_src
			, const PCNormalPtr &keypoints_tgt
			, const CorrespondencesPtr &corr);


	bool rejection;
	bool reciprocal;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> vis;

	PCNormalPtr previousCloud;
	PCNormalPtr previousKeypoints;
	PCFPFHPtr previousFeatures;
	PCXYZRGBPtr fullCloud;

	Eigen::Matrix4d previousTransform;

};

} /* namespace Modelization */
#endif /* REGISTRATION_HPP_ */
