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

	void run(const PCXYZRGBPtr &_src
			, const PCXYZRGBPtr &_tgt
			, PCXYZRGB &cloud_r);


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

	void checkforKeyframe(const PCXYZRGBPtr &fullCloud
			, const PCNormalPtr &Cloud
			, transformPtr &transform);

	bool getView(PCXYZRGB &result);

	void visualizeKeypoints(const PCNormalPtr &cloud
				, const PCNormalPtr &kpoint);

	void visualizeCorrespondances(const PCNormalPtr &cloud_src
			, const PCNormalPtr &cloud_tgt
			, const PCNormalPtr &keypoints_src
			, const PCNormalPtr &keypoints_tgt
			, const CorrespondencesPtr &corr);


	bool rejection;
	bool reciprocal;
	std::size_t viewframes;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> vis;

	std::vector<PCNormalPtr> Keyframes;    				//Set of Clouds Representing the full environment
	std::vector<transformPtr> KeyTransform;				//Transform Matrix relative to previous keyframe
	std::vector<PCXYZRGBPtr> KeyframeFull;    			//Set of non voxelFiltered Clouds Representing the full environment intended for display
	transformPtr accTransform;						//Sum of transformation matrices until a new keyframe is found

	PCNormalPtr previousCloud;
	PCXYZRGBPtr fullCloud;

	Eigen::Matrix4d previousTransform;
	Eigen::Matrix4d identityMatrix;

};

} /* namespace Modelization */
#endif /* REGISTRATION_HPP_ */
