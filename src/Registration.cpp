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
		//Ransacregistration(src, previousCloud, transform);
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
//	pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZRGBNormal> rej;
//	rej.setInlierThreshold(0.001); //10 cm
//	rej.setInputSource(src);
//	rej.setInputTarget(tgt);
//	rej.setMaximumIterations(100);
//	Slow

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
	//pcl::registration::CorrespondenceEstimationNormalShooting<PointXYZRGBNormal, PointXYZRGBNormal, PointXYZRGBNormal> est;
	//pcl::registration::CorrespondenceEstimation<PointT, PointT> est;
	pcl::registration::CorrespondenceEstimationBackProjection<PointXYZRGBNormal, PointXYZRGBNormal, PointXYZRGBNormal> est;
	est.setInputSource (src);
	est.setInputTarget (tgt);

	est.setSourceNormals (src);
	est.setTargetNormals (tgt);
	est.setKSearch (10); //Also try with 30

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

//		Visualize first iteration all correspondances
//		if(iterations==0)
//			visualizeCorrespondances(output,tgt,output,tgt,all_correspondences);
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

//				Visualize first iteration good correspondances
//		if(iterations==0)
//					visualizeCorrespondances(output,tgt,output,tgt,good_correspondences);

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

int Registration::Ransacregistration (const PCNormalPtr &src
		, const PCNormalPtr &tgt
		, Eigen::Matrix4d &transform)
{

	PCNormalPtr temptgt =  tgt;
	PCNormalPtr tempsrc =  src;

	if(tgt->size()<src->size())
	{
		std::vector<int> index( boost::counting_iterator<int>( 0 ),
				boost::counting_iterator<int>(tgt->size()) );
		tempsrc.reset(new PCNormal(*src,index));
	}
	else if(src->size()<tgt->size())
	{
		std::vector<int> index( boost::counting_iterator<int>( 0 ),
				boost::counting_iterator<int>(src->size()) );
		temptgt.reset(new PCNormal(*tgt,index));
	}


	pcl::SampleConsensusModelRegistration<PointXYZRGBNormal>::Ptr sac_model(new pcl::SampleConsensusModelRegistration<PointXYZRGBNormal>(temptgt));
	sac_model->setInputTarget(tempsrc);

	pcl::RandomSampleConsensus<PointXYZRGBNormal> ransac(sac_model);
	//pcl::LeastMedianSquares<pcl::PointNormal> ransac(sac_model); //might as well try these out too!
	//pcl::ProgressiveSampleConsensus<PointXYZRGBNormal> ransac(sac_model);
	ransac.setDistanceThreshold(0.01);
	ransac.setMaxIterations(100);

	//upping the verbosity level to see some info
	ransac.computeModel(1);


	Eigen::VectorXf coeffs;
	ransac.getModelCoefficients(coeffs);
	assert(coeffs.size() == 16);
	transform = Eigen::Map<Eigen::Matrix4f>(coeffs.data(),4,4).cast<double>();

	std::cout<<"Image Sizes :  "<< tempsrc->size()<<"       "<<temptgt->size()<<std::endl;
	std::vector<int> inliers; ransac.getInliers(inliers);
	return inliers.size();
}


// important : Radious search in m..... 1m manual adjustment for the moment
void Registration::estimateKeypoints (const PCNormalPtr &cloud
		, PCNormalPtr &keypoints)
{
	pcl::PointCloud<int> keypoints_idx;
	// Get an uniform grid of keypoints
	pcl::UniformSampling<PointXYZRGBNormal> uniform;
	uniform.setRadiusSearch(0.005);  // 10 cm

	uniform.setInputCloud (cloud);
	uniform.compute(keypoints_idx);
	pcl::copyPointCloud(*cloud, keypoints_idx.points, *keypoints);

//	const float min_scale = 0.005;
//	const int nr_octaves = 4;
//	const int nr_scales_per_octave = 5;
//	const float min_contrast = 1;
//
//	PCXYZRGBPtr cloudtemp (new PCXYZRGB);
//	pcl::copyPointCloud (*cloud,*cloudtemp);
//	  pcl::SIFTKeypoint<PointXYZRGB, pcl::PointWithScale> sift_detect;
//	  sift_detect.setSearchMethod (pcl::search::Search<PointXYZRGB>::Ptr (new pcl::search::KdTree<PointXYZRGB>));
//	  sift_detect.setScales (min_scale, nr_octaves, nr_scales_per_octave);
//	  sift_detect.setMinimumContrast (min_contrast);
//	  sift_detect.setInputCloud (cloudtemp);
//	  pcl::PointCloud<pcl::PointWithScale> keypoints_temp;
//	  sift_detect.compute (keypoints_temp);
//	  pcl::copyPointCloud (keypoints_temp, *keypoints);

}

void Registration::estimateFPFH (const PCNormalPtr &cloud
		, const PCNormalPtr &keypoints
		, PCFPFHPtr &fpfhs)
{
//	pcl::FPFHEstimation<PointXYZRGBNormal, PointXYZRGBNormal, FPFHSignature> fpfh_est;
	pcl::FPFHEstimationOMP<PointXYZRGBNormal, PointXYZRGBNormal, FPFHSignature> fpfh_est;
	fpfh_est.setInputCloud (keypoints);
	fpfh_est.setInputNormals (cloud);
	fpfh_est.setRadiusSearch (0.005); // 5cm
	fpfh_est.setSearchSurface (cloud);
	fpfh_est.compute (*fpfhs);

}


void Registration::findCorrespondences (const PCFPFHPtr &fpfhs_src
		, const PCFPFHPtr &fpfhs_tgt
		, Correspondences &all_correspondences)
{
  pcl::registration::CorrespondenceEstimation<FPFHSignature, FPFHSignature> est;
  est.setInputSource (fpfhs_src);
  est.setInputTarget (fpfhs_tgt);
  est.determineReciprocalCorrespondences (all_correspondences);
}

void Registration::rejectBadCorrespondencesDistance (const CorrespondencesPtr &all_correspondences,
                          const PCNormalPtr &keypoints_src,
                          const PCNormalPtr &keypoints_tgt,
                          Correspondences &remaining_correspondences)
{
  pcl::registration::CorrespondenceRejectorDistance rej;
  rej.setInputSource<PointXYZRGBNormal>(keypoints_src);
  rej.setInputTarget<PointXYZRGBNormal>(keypoints_tgt);
  rej.setMaximumDistance (0.1);    // 1cm
  rej.setInputCorrespondences (all_correspondences);
  rej.getCorrespondences (remaining_correspondences);
}

void Registration::computeTransformation(const PCNormalPtr &src
		, const PCNormalPtr &tgt
		, const PCNormalPtr &keypoints_src
		, const PCNormalPtr &keypoints_tgt
		, const PCFPFHPtr & fpfhs_src
		, const PCFPFHPtr & fpfhs_tgt,
		Eigen::Matrix4d &transform)
{

	Eigen::Matrix4f tempTransform;
	CorrespondencesPtr all_correspondences (new Correspondences),
	                     good_correspondences (new Correspondences);
	  findCorrespondences (fpfhs_src, fpfhs_tgt, *all_correspondences);

	  visualizeCorrespondances(src,tgt,keypoints_src,keypoints_tgt,all_correspondences);

	  rejectBadCorrespondencesDistance (all_correspondences, keypoints_src, keypoints_tgt, *good_correspondences);
	  visualizeCorrespondances(src,tgt,keypoints_src,keypoints_tgt,good_correspondences);


	  pcl::registration::TransformationEstimationSVD<PointXYZRGBNormal, PointXYZRGBNormal> trans_est;
	    trans_est.estimateRigidTransformation (*keypoints_src, *keypoints_tgt, *good_correspondences, tempTransform);
	    transform = tempTransform.cast<double>();

}



void Registration::runLoop2(const PCXYZRGBPtr &_in
		, PCXYZRGB &cloud_r){

#ifdef _DEBUG
	setVerbosityLevel (pcl::console::L_DEBUG);
#endif

	PCXYZRGBPtr output(new PCXYZRGB);
	Eigen::Matrix4d transform;
	PCNormalPtr src(new PCNormal);
	PCNormalPtr keypoints;
	PCFPFHPtr features;
	std::vector<int> temp;

	estimateNormals(_in,*src,true,0.001);
	pcl::removeNaNFromPointCloud(*src,*src,temp);

	//Initialization
	if(previousCloud.get()==0){
		fullCloud.reset(new PCXYZRGB);
		previousCloud.swap(src);
		previousKeypoints.reset(new PCNormal);
		estimateKeypoints(previousCloud,previousKeypoints);
		previousFeatures.reset(new PCFPFH);
		estimateFPFH(previousCloud,previousKeypoints,previousFeatures);
		previousTransform = Eigen::Matrix4d::Identity ();
		cerr<<"Setting up"<<endl;
	}
	//Common cycle
	else{

		keypoints.reset(new PCNormal);
		features.reset(new PCFPFH);
		estimateKeypoints(src,keypoints);
		estimateFPFH(src,keypoints,features);
		computeTransformation(src,previousCloud,keypoints,previousKeypoints,features,previousFeatures,transform);

		//		do all
		//		Ransacregistration(src, previousCloud, transform);
		previousFeatures.swap(features);
		previousCloud.swap(src);
		previousKeypoints.swap(keypoints);
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

	visualizeKeypoints(previousCloud,previousKeypoints);
	cout<<"keypoint size: "<<previousKeypoints->size()<<"  Features Size: "<< previousFeatures->size()<<endl;

	//	pcl::visualization::CloudViewer viewer("Example");
	//	viewer.showCloud(fullCloud);
	//	while(!viewer.wasStopped())
	//	{
	//
	//	}
}

void Registration::visualizeKeypoints(const PCNormalPtr &cloud
		, const PCNormalPtr &kpoint)
{
	pcl::visualization::PCLVisualizer viewer ("Keypoint viewer");
	pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGBNormal> handler_cloud(cloud);
	viewer.addPointCloud(cloud,handler_cloud,"cloud");

	pcl::visualization::PointCloudColorHandlerCustom<PointXYZRGBNormal> kpoint_color_handler (kpoint, 0, 255, 0);
	viewer.addPointCloud(kpoint, kpoint_color_handler, "keypoints");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

	//	 viewer.addPointCloudNormals<PCNormal,PCNormal>(kpoint,kpoint,0,1.0,"normals");
	viewer.spin();

}

void Registration::visualizeCorrespondances(const PCNormalPtr &cloud_src
		, const PCNormalPtr &cloud_tgt
		, const PCNormalPtr &keypoints_src
		, const PCNormalPtr &keypoints_tgt
		, const CorrespondencesPtr &corr)
{
//this is the visualizer
pcl::visualization::PCLVisualizer viscorr;
pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGBNormal> color_handler1 (cloud_src);
viscorr.addPointCloud(cloud_src,color_handler1, "src_points");
viscorr.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.2,"src_points");

pcl::visualization::PointCloudColorHandlerCustom<PointXYZRGBNormal> kpoint_color_handler (keypoints_src, 0, 255, 0);
viscorr.addPointCloud<PointXYZRGBNormal>(keypoints_src, kpoint_color_handler, "keypoints1");
viscorr.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints1");

Eigen::Matrix4f t;
t<<1,0,0,1,
     0,1,0,0,
     0,0,1,0,
     0,0,0,1;

//cloud view contains the translated cloud
PCNormalPtr cloudview(new PCNormal);
PCNormalPtr keypointsView(new PCNormal);
//cloudNext is my target cloud
pcl::transformPointCloud(*cloud_tgt,*cloudview,t);
pcl::transformPointCloud(*keypoints_tgt,*keypointsView,t);

pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGBNormal> color_handler2 (cloudview);
viscorr.addPointCloud(cloudview,color_handler2, "tgt_points");
viscorr.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.2,"tgt_points");

pcl::visualization::PointCloudColorHandlerCustom<PointXYZRGBNormal> kpoint_color_handler2 (keypointsView, 0, 255, 0);
viscorr.addPointCloud<PointXYZRGBNormal>(keypointsView, kpoint_color_handler2, "keypoints2");
viscorr.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints2");

viscorr.addCorrespondences<PointXYZRGBNormal>(keypoints_src,keypointsView,*corr,"Correspondences");
viscorr.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "Correspondences");
viscorr.resetCamera ();
viscorr.spin ();
}

} /* namespace Modelization */
