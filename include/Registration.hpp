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
// Correspondences typedefs
typedef std::vector< pcl::Correspondence, Eigen::aligned_allocator<pcl::Correspondence> > Correspondences;
typedef boost::shared_ptr<Correspondences> CorrespondencesPtr;

class Registration {
public:

	/*
	 *\brief Registration Main constructor.
	 *\param[in] _rejection Enable the bad correspondance rejections for the transformation estimation
	 *\param[in] _reciprocal if true all correspondances must
	 *\param[in] _Threshold Minimum number of correspondences to determine a valid keyframe
	 * be reciprocal in order to be considered as a valid correspondance
	 */
	Registration(bool _rejection = true, bool _reciprocal = true, float VfiltSize=0.01,  int _Threshold = 1000);

	/*
	 *\brief Default empty destructor
	 */
	virtual ~Registration();

	/*
	 * \brief Main Loop registration function Multithreaded
	 * \warn the image must not be changed.
	 * \param[in] _in the input cloud to be added to the registration
	 */
	bool addCloud(const PCXYZRGBPtr &_in);

	/*
		 * \brief Get View Multithreaded
		 * \param[out] _in the resultant cloud
		 */
	bool getResult(PCXYZRGBPtr &_out);

	/*
	 * \brief Main Loop registration function
	 * \param[in] _in the input cloud to be added to the registration
	 */
	void runLoop(const PCXYZRGBPtr &_in);

	/*
	 * \brief Registration test function not looped
	 * \param[in] _src source cloud.
	 * \param[in] _tgt target cloud.
	 * \param[out] cloud_r Resulting cloud from registration.
	 */
	void run(const PCXYZRGBPtr &_src
			, const PCXYZRGBPtr &_tgt
			, PCXYZRGB &cloud_r);

	/*
	 * \brief returns the registration result from images given by the \ref runLoop function
	 * \param[out] result the complete registered cloud.
	 */
	void getView(PCXYZRGBPtr &result);

private:
	/*
	 * \brief correspondence rejection function using the median distance as elimination parameter
	 */
	void rejectBadCorrespondences (const CorrespondencesPtr& all_correspondences,
			const PCNormalPtr& src,
			const PCNormalPtr& tgt,
			Correspondences& remaining_correspondences);

	/*
	 * \brief Correspondence finding of the two point clouds using backprojection
	 */
	void findCorrespondences (const PCNormalPtr& src,
			const PCNormalPtr& tgt,
			Correspondences& all_correspondences);

	/*
	 * \brief transformation estimation using point to plane algorithm
	 */
	void findTransformation (const PCNormalPtr& src,
			const PCNormalPtr& tgt,
			const CorrespondencesPtr& correspondences,
			Eigen::Matrix4d &transform);

	/*
	 * brief Debug function to visualize correspondences between clouds src and tgt
	 */
	void view (const PCNormal::ConstPtr& src,
			const PCNormal::ConstPtr& tgt,
			const CorrespondencesPtr& correspondences);

	/*
	 * \brief visualize both point clouds with no registration transformation
	 */
	void view (const PCXYZRGB::ConstPtr &src
			, const PCXYZRGB::ConstPtr &tgt);

	/*
	 * \brief Hardware based normal esimation function if \ref _downsample set to true a cloud
	 * downsample is performed, using voxel filtering.
	 */
	void estimateNormals(const PCXYZRGBPtr& cloud_in
			, PCNormal& cloud_out
			, bool _downsample
			, float _leaf_size = 0.01f);

	/*
	 * \brief Main function, performs the ICP loop and return the transformation quaternion
	 */
	void icp (const PCNormalPtr& src,
			const PCNormalPtr& tgt,
			Eigen::Matrix4d& transform);

	/*
	 * \brief verifies if the frameshould be considered as key frame and adds it to the result
	 */
	void checkforKeyframe(const PCXYZRGBPtr &fullCloud
			, const PCNormalPtr &Cloud
			, transformPtr &transform);

	/*
	 * \brief verifies the initial number of correspondences between two frames
	 * \return true if the number of correspondences exceeds a threshold
	 */
	bool verifyCorrespondences(const PCNormalPtr &source
			, const PCNormalPtr &target);

	/*
	 * \brief keypoint visualization function.
	 */
	void visualizeKeypoints(const PCNormalPtr &cloud
			, const PCNormalPtr &kpoint);

	/*
	 * \brief correspondance visualization with keypoints.
	 */
	void visualizeCorrespondances(const PCNormalPtr &cloud_src
			, const PCNormalPtr &cloud_tgt
			, const PCNormalPtr &keypoints_src
			, const PCNormalPtr &keypoints_tgt
			, const CorrespondencesPtr &corr);


	bool rejection;
	bool reciprocal;
	std::size_t viewframes;
	float vFiltSize;
	std::size_t minThreshold;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> vis;

	std::vector<PCNormalPtr> Keyframes;    				//Set of Clouds Representing the full environment
	std::vector<transformPtr> KeyTransform;				//Transform Matrix relative to previous keyframe
	std::vector<PCXYZRGBPtr> KeyframeFull;    			//Set of non voxelFiltered Clouds Representing the full environment intended for display
	transformPtr accTransform;							//Sum of transformation matrices until a new keyframe is found

	PCNormalPtr previousCloud;
	PCXYZRGBPtr fullCloud;
	PCXYZRGBPtr Resultcloud;

	Eigen::Matrix4d previousTransform;
	Eigen::Matrix4d identityMatrix;

	boost::thread LoopThread;
	boost::thread getViewThread;

	bool isRunning;
	bool gettingView;
	bool viewReady;

};

} /* namespace Modelization */
#endif /* REGISTRATION_HPP_ */
