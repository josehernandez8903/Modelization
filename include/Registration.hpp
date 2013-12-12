/*
 * Registration.hpp
 *
 *  Created on: Dec 6, 2013
 *      Author: José Juan Hernández López
 */

#ifndef REGISTRATION_HPP_
#define REGISTRATION_HPP_

#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>

#include <pcl/registration/default_convergence_criteria.h>

#include "planeDetection.hpp"
#include "include.hpp"


namespace Modelization {
using pcl::Normal;
//Correspondences
typedef std::vector< pcl::Correspondence, Eigen::aligned_allocator<pcl::Correspondence> > Correspondences;
typedef boost::shared_ptr<Correspondences> CorrespondencesPtr;

class Registration {
public:
	Registration(bool _rejection = true, bool _reiprocal = false);
	virtual ~Registration();

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



	bool rejection;
	bool reciprocal;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> vis;
};

} /* namespace Modelization */
#endif /* REGISTRATION_HPP_ */
