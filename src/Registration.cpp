/*
 * Registration.cpp
 *
 *  Created on: Dec 6, 2013
 *      Author: jose
 */

#include "../include/Registration.hpp"

namespace Modelization {

Registration::Registration(bool _rejection, bool _reciprocal):
				rejection(_rejection), reciprocal(_reciprocal)
{

}

Registration::~Registration() {

}

void Registration::run(const PCLXYZRGBPointPtr &_src
		, const PCLXYZRGBPointPtr &_tgt
		, PCLXYZRGBPoint &cloud_r){
#ifdef _DEBUG
	setVerbosityLevel (pcl::console::L_DEBUG);
#endif

	//TODO : convert input to Normal_points
	Eigen::Matrix4d transform;
	CloudNormalPtr src(new CloudNormal);
	CloudNormalPtr tgt(new CloudNormal);


	icp (src, tgt, transform);

}

////////////////////////////////////////////////////////////////////////////////////
//Private Member functions
////////////////////////////////////////////////////////////////////////////////////


void Registration::rejectBadCorrespondences (const CorrespondencesPtr &all_correspondences,
		const CloudNormalPtr &src,
		const CloudNormalPtr &tgt,
		Correspondences &remaining_correspondences)
{
	pcl::registration::CorrespondenceRejectorMedianDistance rej;
	rej.setMedianFactor (8.79241104);
	rej.setInputCorrespondences (all_correspondences);

	rej.getCorrespondences (remaining_correspondences);
	//Todo : angle correspondances

	return;

	CorrespondencesPtr remaining_correspondences_temp (new Correspondences);
	rej.getCorrespondences (*remaining_correspondences_temp);
	PCL_DEBUG ("[rejectBadCorrespondences] Number of correspondences remaining after rejection: %d\n", remaining_correspondences_temp->size ());

	// Reject if the angle between the normals is really off
	pcl::registration::CorrespondenceRejectorSurfaceNormal rej_normals;
	rej_normals.setThreshold (acos (pcl::deg2rad (45.0)));
	rej_normals.initializeDataContainer<pointRGBNormal, pointRGBNormal> ();
	rej_normals.setInputCloud<pointRGBNormal> (src);
	rej_normals.setInputNormals<pointRGBNormal, pointRGBNormal> (src);
	rej_normals.setInputTarget<pointRGBNormal> (tgt);
	rej_normals.setTargetNormals<pointRGBNormal, pointRGBNormal> (tgt);
	rej_normals.setInputCorrespondences (remaining_correspondences_temp);
	rej_normals.getCorrespondences (remaining_correspondences);
}

/************************************************************************************/
/************************************************************************************/
/************************************************************************************/
void Registration::findCorrespondences (const CloudNormalPtr &src,
		const CloudNormalPtr &tgt,
		Correspondences &all_correspondences)
{
	//CorrespondenceEstimationNormalShooting<PointT, PointT, PointT> est;
	//CorrespondenceEstimation<PointT, PointT> est;
	pcl::registration::CorrespondenceEstimationBackProjection<pointRGBNormal, pointRGBNormal, pointRGBNormal> est;
	est.setInputSource (src);
	est.setInputTarget (tgt);

	//Todo : Normals Normals Normals...
	est.setSourceNormals (src);
	est.setTargetNormals (tgt);
	est.setKSearch (10);

	if (reciprocal)
		est.determineReciprocalCorrespondences (all_correspondences);
	else
		est.determineCorrespondences (all_correspondences);

}

/************************************************************************************/
/************************************************************************************/
/************************************************************************************/

void Registration::findTransformation (const CloudNormalPtr &src,
		const CloudNormalPtr &tgt,
		const CorrespondencesPtr &correspondences,
		Eigen::Matrix4d &transform)
{
	pcl::registration::TransformationEstimationPointToPlaneLLS<pointRGBNormal, pointRGBNormal, double> trans_est;
	trans_est.estimateRigidTransformation (*src, *tgt, *correspondences, transform);
}

/************************************************************************************/
/************************************************************************************/
/************************************************************************************/

void Registration::view (const CloudNormal::ConstPtr &src, const CloudNormal::ConstPtr &tgt, const CorrespondencesPtr &correspondences)
{

	pcl::visualization::PointCloudColorHandlerCustom<pointRGBNormal> green (tgt, 0, 255, 0);
	if (!vis->updatePointCloud<pointRGBNormal> (src, "source"))
	{
		vis->addPointCloud<pointRGBNormal> (src, "source");
		vis->resetCameraViewpoint ("source");
	}
	if (!vis->updatePointCloud<pointRGBNormal> (tgt, green, "target")) vis->addPointCloud<pointRGBNormal> (tgt, green, "target");
	vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "source");
	vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.7, "target");
	vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "source");
	pcl::console::TicToc tt;
	tt.tic ();
	if (!vis->updateCorrespondences<pointRGBNormal> (src, tgt, *correspondences, 1))
		vis->addCorrespondences<pointRGBNormal> (src, tgt, *correspondences, 1, "correspondences");
	tt.toc_print ();
	vis->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "correspondences");
	//vis->setShapeRenderingProperties (PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "correspondences");
	vis->spin ();
}

/************************************************************************************/
/************************************************************************************/
/************************************************************************************/
void Registration::icp (const CloudNormalPtr &src
		, const CloudNormalPtr &tgt
		, Eigen::Matrix4d &transform)
{
	vis.reset(new pcl::visualization::PCLVisualizer("Registration example"));
	CorrespondencesPtr all_correspondences (new Correspondences),
			good_correspondences (new Correspondences);

	CloudNormalPtr output (new CloudNormal);
	*output = *src;

	Eigen::Matrix4d final_transform (Eigen::Matrix4d::Identity ());

	int iterations = 0;
	pcl::registration::DefaultConvergenceCriteria<double> converged (iterations, transform, *good_correspondences);

	// ICP loop
	do
	{
		// Find correspondences
		findCorrespondences (output, tgt, *all_correspondences);
		PCL_DEBUG ("Number of correspondences found: %d\n", all_correspondences->size ());

		if (rejection)
		{
			// Reject correspondences
			rejectBadCorrespondences (all_correspondences, output, tgt, *good_correspondences);
			PCL_DEBUG ("Number of correspondences remaining after rejection: %d\n", good_correspondences->size ());
		}
		else
			*good_correspondences = *all_correspondences;

		// Find transformation
		findTransformation (output, tgt, good_correspondences, transform);

		// Obtain the final transformation
		final_transform = transform * final_transform;

		// Transform the data
		transformPointCloudWithNormals (*src, *output, final_transform.cast<float> ());

		// Check if convergence has been reached
		++iterations;

		// Visualize the results
		view (output, tgt, good_correspondences);
	}
	while (!converged);
	transform = final_transform;
}
} /* namespace Modelization */
