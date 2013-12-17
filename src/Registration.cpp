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

void Registration::runLoop(const PCXYZRGBPtr &_in
		, PCXYZRGB &cloud_r){

#ifdef _DEBUG
	setVerbosityLevel (pcl::console::L_DEBUG);
#endif

	PCXYZRGBPtr output(new PCXYZRGB);
	Eigen::Matrix4d transform;
	PCNormalPtr src(new PCNormal);
	std::vector<int> temp;

	estimateNormals(_in,*src,true,0.01);
	pcl::removeNaNFromPointCloud(*src,*src,temp);

	if(previousCloud.get()==0){
		fullCloud.reset(new PCXYZRGB);
		previousCloud.swap(src);
		previousTransform = Eigen::Matrix4d::Identity ();
		cerr<<"Setting up"<<endl;
	}
	else{

		icp (src, previousCloud, transform);
		previousCloud.swap(src);
		previousTransform=previousTransform*transform;
	}

	transformPointCloud (*_in, *output, previousTransform.cast<float> ());
	cout<<"Matrix transform"<<endl<< previousTransform<<endl;

	PCXYZRGBPtr tempCloud = fullCloud->makeShared();
	cout << "Number of points before merging = "<<tempCloud->size()<<endl;
	*tempCloud+=*output;
	cout << "Number of points pre-filter = "<<tempCloud->size()<<endl;
	Modelization::planeDetection::voxel_filter(tempCloud,0.005,*fullCloud);
	cout << "Number of points post-filter = "<<fullCloud->size()<<endl;
	cloud_r=*fullCloud;

//	pcl::visualization::CloudViewer viewer("Example");
//	viewer.showCloud(fullCloud);
//	while(!viewer.wasStopped())
//	{
//
//	}
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
	estimateNormals(_src,*src,true,0.005);
	estimateNormals(_tgt,*tgt,true,0.005);

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
	rej.setMedianFactor (4.79241104);
	rej.setInputCorrespondences (all_correspondences);
	rej.getCorrespondences (remaining_correspondences);

	return;

	CorrespondencesPtr remaining_correspondences_temp (new Correspondences);
	rej.getCorrespondences (*remaining_correspondences_temp);
	//PCL_DEBUG ("[rejectBadCorrespondences] Number of correspondences remaining after rejection: %d\n", remaining_correspondences_temp->size ());

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
	pcl::registration::CorrespondenceEstimationNormalShooting<PointXYZRGBNormal, PointXYZRGBNormal, PointXYZRGBNormal> est;
	//pcl::registration::CorrespondenceEstimation<PointT, PointT> est;
	//pcl::registration::CorrespondenceEstimationBackProjection<PointXYZRGBNormal, PointXYZRGBNormal, PointXYZRGBNormal> est;
	est.setInputSource (src);
	est.setInputTarget (tgt);

	est.setSourceNormals (src);
//	est.setTargetNormals (tgt);
	est.setKSearch (20);

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
	Eigen::Matrix4d final_transform (Eigen::Matrix4d::Identity ());
	pcl::console::TicToc timer;
//	vis.reset(new pcl::visualization::PCLVisualizer("Registration example"));
	CorrespondencesPtr all_correspondences (new Correspondences),
			good_correspondences (new Correspondences);
	PCNormalPtr output (new PCNormal);

	*output = *src;



	int iterations = 0;
	pcl::registration::DefaultConvergenceCriteria<double> converged (iterations, transform, *good_correspondences);

	// ICP loop
	do
	{
		// Find correspondences
		timer.tic();
		findCorrespondences (output, tgt, *all_correspondences);
		cout<<"Find Correspondances "<<": "<<timer.toc()<<" Miliseconds"<<endl;
		PCL_DEBUG ("Number of correspondences found: %d\n", all_correspondences->size ());

		if (rejection)
		{
			// Reject correspondences
			timer.tic();
			rejectBadCorrespondences (all_correspondences, output, tgt, *good_correspondences);
			cout<<"Reject Correspondances "<<": "<<timer.toc()<<" Miliseconds"<<endl;
			PCL_DEBUG ("Number of correspondences remaining after rejection: %d\n", good_correspondences->size ());
		}
		else
			timer.tic();
			*good_correspondences = *all_correspondences;

		// Find transformation
		findTransformation (output, tgt, good_correspondences, transform);
		cout<<"Find Transformation "<<": "<<timer.toc()<<" Miliseconds"<<endl;
		// Obtain the final transformation
		final_transform = transform * final_transform;

		// Transform the data
		timer.tic();
		transformPointCloudWithNormals (*src, *output, final_transform.cast<float> ());
		cout<<"Transform Cloud "<<": "<<timer.toc()<<" Miliseconds"<<endl;
		// Check if convergence has been reached
		++iterations;

	}
	while (!converged);
	cout<<"Converged "<<converged<<endl;
//	view (src, tgt, good_correspondences);
//	view (output, tgt, good_correspondences);
	transform = final_transform;
}
} /* namespace Modelization */
