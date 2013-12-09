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

#include <pcl/registration/default_convergence_criteria.h>

#include "planeDetection.hpp"
#include "include.hpp"


namespace Modelization {

//Correspondences
typedef std::vector< pcl::Correspondence, Eigen::aligned_allocator<pcl::Correspondence> > Correspondences;
typedef boost::shared_ptr<Correspondences> CorrespondencesPtr;

class Registration {
public:
	Registration(bool _rejection = true, bool _reiprocal = false);
	virtual ~Registration();

	void run(const PCLXYZRGBPointPtr &_src
			, const PCLXYZRGBPointPtr &_tgt
			, PCLXYZRGBPoint &cloud_r);

private:
	void rejectBadCorrespondences (const CorrespondencesPtr &all_correspondences,
			const CloudNormalPtr &src,
			const CloudNormalPtr &tgt,
			Correspondences &remaining_correspondences);

	void findCorrespondences (const CloudNormalPtr &src,
			const CloudNormalPtr &tgt,
			Correspondences &all_correspondences);

	void findTransformation (const CloudNormalPtr &src,
			const CloudNormalPtr &tgt,
			const CorrespondencesPtr &correspondences,
			Eigen::Matrix4d &transform);

	void view (const CloudNormal::ConstPtr &src,
			const CloudNormal::ConstPtr &tgt,
			const CorrespondencesPtr &correspondences);

	void icp (const CloudNormalPtr &src,
			const CloudNormalPtr &tgt,
			Eigen::Matrix4d &transform);



	bool rejection;
	bool reciprocal;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> vis;
};

} /* namespace Modelization */
#endif /* REGISTRATION_HPP_ */
