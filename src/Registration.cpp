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

void Registration::run(const PCXYZRGBPtr &_src
		, const PCXYZRGBPtr &_tgt
		, PCXYZRGB &cloud_r){
#ifdef _DEBUG
	setVerbosityLevel (pcl::console::L_DEBUG);
#endif

	Eigen::Matrix4d transform;
	PCNormalPtr src(new PCNormal);
	PCNormalPtr tgt(new PCNormal);
	PCXYZRGBPtr output(new PCXYZRGB);
	estimateNormals(_src,*src,true,0.01);
	estimateNormals(_tgt,*tgt,true,0.01);

#ifdef _DEBUG
	//	pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGBNormal> color_handler1 (src);
	//	pcl::visualization::PCLVisualizer viewer1("Input1");
	//	viewer1.setBackgroundColor (0.0, 0.0, 0.0);
	//	viewer1.addPointCloud<PointXYZRGBNormal>(src,color_handler1,"Input_cloud2");
	//	viewer1.addPointCloudNormals<PointXYZRGBNormal,PointXYZRGBNormal>(src, src,20,0.05,"Input_Normals2");
	//	viewer1.spin();
	//
	//	pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGBNormal> color_handler2 (tgt);
	//	pcl::visualization::PCLVisualizer viewer2("Input2");
	//	viewer2.setBackgroundColor (0.0, 0.0, 0.0);
	//	viewer2.addPointCloud<PointXYZRGBNormal>(tgt,color_handler2,"Input_cloud2");
	//	viewer2.addPointCloudNormals<PointXYZRGBNormal,PointXYZRGBNormal>(tgt, tgt,20,0.05,"Input_Normals2");
	//	viewer2.spin();
#endif
	std::vector<int> temp;
pcl::removeNaNFromPointCloud(*src,*src,temp);
pcl::removeNaNFromPointCloud(*tgt,*tgt,temp);

	icp (src, tgt, transform);
	transformPointCloud (*_src, *output, transform.cast<float> ());
	view (output, _tgt);

}

////////////////////////////////////////////////////////////////////////////////////
//Private Member functions
////////////////////////////////////////////////////////////////////////////////////


void Registration::rejectBadCorrespondences (const CorrespondencesPtr &all_correspondences,
		const PCNormalPtr &src,
		const PCNormalPtr &tgt,
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
	rej_normals.initializeDataContainer<PointXYZRGBNormal, PointXYZRGBNormal> ();
	rej_normals.setInputCloud<PointXYZRGBNormal> (src);
	rej_normals.setInputNormals<PointXYZRGBNormal, PointXYZRGBNormal> (src);
	rej_normals.setInputTarget<PointXYZRGBNormal> (tgt);
	rej_normals.setTargetNormals<PointXYZRGBNormal, PointXYZRGBNormal> (tgt);
	rej_normals.setInputCorrespondences (remaining_correspondences_temp);
	rej_normals.getCorrespondences (remaining_correspondences);
}

/************************************************************************************/
/************************************************************************************/
/************************************************************************************/
void Registration::findCorrespondences (const PCNormalPtr &src,
		const PCNormalPtr &tgt,
		Correspondences &all_correspondences)
{
	//CorrespondenceEstimationNormalShooting<PointT, PointT, PointT> est;
	//CorrespondenceEstimation<PointT, PointT> est;
	pcl::registration::CorrespondenceEstimationBackProjection<PointXYZRGBNormal, PointXYZRGBNormal, PointXYZRGBNormal> est;
	est.setInputSource (src);
	est.setInputTarget (tgt);

	est.setSourceNormals (src);
	est.setTargetNormals (tgt);
	est.setKSearch (50);

	if (reciprocal)
		est.determineReciprocalCorrespondences (all_correspondences);
	else
		est.determineCorrespondences (all_correspondences);

}

/************************************************************************************/
/************************************************************************************/
/************************************************************************************/

void Registration::findTransformation (const PCNormalPtr &src,
		const PCNormalPtr &tgt,
		const CorrespondencesPtr &correspondences,
		Eigen::Matrix4d &transform)
{
	pcl::registration::TransformationEstimationPointToPlaneLLS<PointXYZRGBNormal, PointXYZRGBNormal, double> trans_est;
	trans_est.estimateRigidTransformation (*src, *tgt, *correspondences, transform);
}

/************************************************************************************/
/************************************************************************************/
/************************************************************************************/

void Registration::view (const PCNormal::ConstPtr &src, const PCNormal::ConstPtr &tgt, const CorrespondencesPtr &correspondences)
{

	pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGBNormal> handler_src(src);
	pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGBNormal> handler_tgt(tgt);
	if (!vis->updatePointCloud<PointXYZRGBNormal> (src,handler_src, "source"))
	{
		vis->addPointCloud<PointXYZRGBNormal> (src,handler_src, "source");
		vis->resetCameraViewpoint ("source");
	}
	if (!vis->updatePointCloud<PointXYZRGBNormal> (tgt, handler_tgt, "target")) vis->addPointCloud<PointXYZRGBNormal> (tgt, handler_tgt, "target");
	vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "source");
	vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.7, "target");
	//	if (!vis->updateCorrespondences<PointXYZRGBNormal> (src, tgt, *correspondences, 1))
	//		vis->addCorrespondences<PointXYZRGBNormal> (src, tgt, *correspondences, 1, "correspondences");
	//	vis->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "correspondences");
	//vis->setShapeRenderingProperties (PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "correspondences");
	vis->spin();
}

/************************************************************************************/
/************************************************************************************/
/************************************************************************************/

void Registration::view (const PCXYZRGB::ConstPtr &src, const PCXYZRGB::ConstPtr &tgt)
{

	pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGB> handler_src(src);
	pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGB> handler_tgt(tgt);
	if (!vis->updatePointCloud<PointXYZRGB> (src,handler_src, "source"))
	{
		vis->addPointCloud<PointXYZRGB> (src,handler_src, "source");
		vis->resetCameraViewpoint ("source");
	}
	if (!vis->updatePointCloud<PointXYZRGB> (tgt, handler_tgt, "target")) vis->addPointCloud<PointXYZRGB> (tgt, handler_tgt, "target");
	vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "source");
	vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.7, "target");
	vis->spin();
}


/************************************************************************************/
/************************************************************************************/
/************************************************************************************/
void Registration::estimateNormals(const PCXYZRGBPtr& cloud_in
		, PCNormal& cloud_out
		, bool _downsample
		, float _leaf_size){

	pcl::PointCloud<Normal>::Ptr cloud_normals (new pcl::PointCloud<Normal>);
	PCNormalPtr cloud_normals_pf (new PCNormal);

	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
	ne.setMaxDepthChangeFactor(0.01f);
	ne.setNormalSmoothingSize(10.0f);
	ne.setInputCloud(cloud_in);

	ne.compute(*cloud_normals);
	pcl::concatenateFields (*cloud_in, *cloud_normals, *cloud_normals_pf);

	if (_downsample)
		Modelization::planeDetection::voxel_filter(cloud_normals_pf,_leaf_size,cloud_out);
	else
		cloud_out=*cloud_normals_pf;
}

/************************************************************************************/
/************************************************************************************/
/************************************************************************************/
void Registration::icp (const PCNormalPtr &src
		, const PCNormalPtr &tgt
		, Eigen::Matrix4d &transform)
{
	vis.reset(new pcl::visualization::PCLVisualizer("Registration example"));
	CorrespondencesPtr all_correspondences (new Correspondences),
			good_correspondences (new Correspondences);

	PCNormalPtr output (new PCNormal);
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

	}
	while (!converged);
	cout<<"Converged "<<converged<<endl;
//	view (src, tgt, good_correspondences);
//	view (output, tgt, good_correspondences);
	transform = final_transform;
}
} /* namespace Modelization */
