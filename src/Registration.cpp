/*
 * Registration.cpp
 *
 *  Created on: Dec 6, 2013
 *      Author: jose
 */

#include "../include/Registration.hpp"

namespace Modelization {
/************************************************************************************/
/* Default constructor */
/************************************************************************************/
Registration::Registration(bool _rejection, bool _reciprocal, float _VfiltSize ,int Threshold):
					rejection(_rejection), reciprocal(_reciprocal), viewframes(0),
					vFiltSize(_VfiltSize), minThreshold(Threshold), Resultcloud(new PCXYZRGB),
					identityMatrix(Eigen::Matrix4d::Identity()), isRunning(false), gettingView(false),
					viewReady(false)
{

}

/************************************************************************************/
/* Default Destructor */
/************************************************************************************/
Registration::~Registration() {
	if(isRunning)
		LoopThread.join();
	if(gettingView)
		getViewThread.join();

}

/************************************************************************************/
/* Thread Running */
/************************************************************************************/
bool Registration::addCloud(const PCXYZRGBCPtr &_in){
	if(!isRunning){
		isRunning = true;
		LoopThread = boost::thread(&Registration::runLoop,this,_in->makeShared());
		return true;
	}

	else{
//		cout<<"StillRunning";
		return false;
	}
}

bool Registration::getResult(PCXYZRGBPtr &_out){
	bool retval = false;
	if(viewReady){
		cout<<"ViewReady"<<endl;
		_out.swap(Resultcloud);
		Resultcloud.reset(new PCXYZRGB);
		retval = true;
		viewReady = false;

	}
	if(!gettingView){
//		cout<<"test"<<endl;
		gettingView = true;
		LoopThread = boost::thread(&Registration::getView,this,Resultcloud);
	}
	else{
//		cout<<"Still getting View"<<endl;
	}
	return retval;
}

/************************************************************************************/
/* Main Running loop, takes an input and adds to the registration cloud */
/************************************************************************************/
void Registration::runLoop(const PCXYZRGBPtr &_in){

	isRunning = true;
	transformPtr transform(new Eigen::Matrix4d);		//ICP resultant transformation Matrix
	PCNormalPtr src(new PCNormal);						//Input cloud containing surface normals
	std::vector<int> temp;

	// Estimate surface normals
	estimateNormals(_in,*src,true,vFiltSize);

	pcl::removeNaNFromPointCloud(*src,*src,temp);

	//if first cloud just copy to the output
	if(previousCloud.get()==0){
		previousCloud.swap(src);
		*transform = identityMatrix;
		//		cerr<<"Setting up"<<endl;
	}

	// if not run the registration function to determine the transformation matrix
	else{
		icp (src, previousCloud, *transform);
		//Ransacregistration(src, previousCloud, *transform);
		previousCloud.swap(src);
	}

	// Check if it is a key frame or not
	checkforKeyframe(_in,previousCloud,transform);

	//transformPointCloud (*_in, *output, previousTransform.cast<float> ());
	//cout<<"Matrix transform"<<endl<< previousTransform<<endl;


	//	PCXYZRGBPtr tempCloud = fullCloud->makeShared();
	//
	//	cout << "Number of points before merging = "<<tempCloud->size()<<endl;
	//	*tempCloud+=*output;
	//	cout << "Number of points pre-filter = "<<tempCloud->size()<<endl;
	//	Modelization::planeDetection::voxel_filter(tempCloud,0.005,*fullCloud);
	//	cout << "Number of points post-filter = "<<fullCloud->size()<<endl;
	//	cloud_r=*fullCloud;

	//		pcl::visualization::CloudViewer viewer("Example");
	//		viewer.showCloud(cloud_r.makeShared());
	//		while(!viewer.wasStopped())
	//		{
	//
	//		}
	isRunning = false;
}

/************************************************************************************/
/************************************************************************************/
/************************************************************************************/
void Registration::run(const PCXYZRGBPtr &_src
		, const PCXYZRGBPtr &_tgt
		, PCXYZRGB &cloud_r){
#ifdef _DEBUG
	setVerbosityLevel (pcl::console::L_ALWAYS);
#endif

	Eigen::Matrix4d transform;					//Transform Quaternion
	PCNormalPtr src(new PCNormal);				//Source Cloud
	PCNormalPtr tgt(new PCNormal);				//Target Cloud
	PCXYZRGBPtr output(new PCXYZRGB);			//Merged Cloud

	// Obtain both cloud normals using the hardware method.
	estimateNormals(_src,*src,true,0.005);
	estimateNormals(_tgt,*tgt,true,0.005);

	//Remove the invalid measurements to optimise the search.
	std::vector<int> temp;
	pcl::removeNaNFromPointCloud(*src,*src,temp);
	pcl::removeNaNFromPointCloud(*tgt,*tgt,temp);

	//Start the ICP algorithm
	icp (src, tgt, transform);
	//transform the source cloud and store it on the output
	transformPointCloud (*_src, *output, transform.cast<float> ());
	//visualize both transformed source and resuolt
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

	//Reject found correspondence if too far from median distance
	pcl::registration::CorrespondenceRejectorMedianDistance rej;
	rej.setMedianFactor (4.79241104);
	rej.setInputCorrespondences (all_correspondences);
	rej.getCorrespondences (remaining_correspondences);

	return;

	//	CorrespondencesPtr remaining_correspondences_temp (new Correspondences);
	//	rej.getCorrespondences (*remaining_correspondences_temp);
	//	//PCL_DEBUG ("[rejectBadCorrespondences] Number of correspondences remaining after rejection: %d\n", remaining_correspondences_temp->size ());
	//
	//	// Reject if the angle between the normals is really off
	//	pcl::registration::CorrespondenceRejectorSurfaceNormal rej_normals;
	//	rej_normals.setThreshold (acos (pcl::deg2rad (45.0)));
	//	rej_normals.initializeDataContainer<PointXYZRGBNormal, PointXYZRGBNormal> ();
	//	rej_normals.setInputCloud<PointXYZRGBNormal> (src);
	//	rej_normals.setInputNormals<PointXYZRGBNormal, PointXYZRGBNormal> (src);
	//	rej_normals.setInputTarget<PointXYZRGBNormal> (tgt);
	//	rej_normals.setTargetNormals<PointXYZRGBNormal, PointXYZRGBNormal> (tgt);
	//	rej_normals.setInputCorrespondences (remaining_correspondences_temp);
	//	rej_normals.getCorrespondences (remaining_correspondences);
}

/************************************************************************************/
/************************************************************************************/
/************************************************************************************/
void Registration::findCorrespondences (const PCNormalPtr &src,
		const PCNormalPtr &tgt,
		Correspondences &all_correspondences)
{
	// estimate correspondences using perpendicular distance to vector as the similarity parameter
	//pcl::registration::CorrespondenceEstimationNormalShooting<PointXYZRGBNormal, PointXYZRGBNormal, PointXYZRGBNormal> est;
	//pcl::registration::CorrespondenceEstimation<PointT, PointT> est;

	// estimate correspondences using the cos(theta) as the similarity parameter
	pcl::registration::CorrespondenceEstimationBackProjection<PointXYZRGBNormal, PointXYZRGBNormal, PointXYZRGBNormal> est;
	est.setInputSource (src);
	est.setInputTarget (tgt);

	est.setSourceNormals (src);
	est.setTargetNormals (tgt); //Used on BackProjectrion
	est.setKSearch (10); //Also try with 30

	// Reciprocal determines if the correspondence chosen is reciprocal.
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
	// SVD transformation estimation consult "Linear Least-Squares Optimization for Point-to-Plane ICP Surface Registration"
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
	if (!vis->updateCorrespondences<PointXYZRGBNormal> (src, tgt, *correspondences, 1))
	{
		vis->addCorrespondences<PointXYZRGBNormal> (src, tgt, *correspondences, 1, "correspondences");
		vis->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "correspondences");
		//vis->setShapeRenderingProperties (PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "correspondences");
	}
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

	//Normal estimation consult "Adaptive Neighborhood Selection for Real-Time Surface
	//NormalEstimation from Organized Point Cloud Data Using Integral Images"
	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
	ne.setMaxDepthChangeFactor(0.01f);
	ne.setNormalSmoothingSize(10.0f);
	ne.setInputCloud(cloud_in);

	ne.compute(*cloud_normals);
	pcl::concatenateFields (*cloud_in, *cloud_normals, *cloud_normals_pf);

	//	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	//	pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGB> handler_cloud(cloud_in);
	//	viewer.addPointCloud<PointXYZRGB>(cloud_in,handler_cloud,"cloud");
	//	viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(cloud_in, cloud_normals,100,0.05f,"Normalss");
	//
	//	while(!viewer.wasStopped())
	//	viewer.spinOnce();

	//Downsample images to increase processing speed
	if (_downsample)
		Modelization::planeDetection::voxel_filter(cloud_normals_pf,_leaf_size,cloud_out);
	else
		cloud_out=*cloud_normals_pf;

	//		pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	//		int v1,v2;
	//		pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGB> handler_cloud(cloud_in);
	//		pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGBNormal> handler_cloud2(cloud_out.makeShared());
	//		viewer.createViewPort(0,0,0.5,1,v1);
	//		viewer.createViewPort(0.5,0,1,1,v2);
	//		viewer.addPointCloud<PointXYZRGB>(cloud_in,handler_cloud,"cloud",v1);
	//		viewer.addPointCloud<PointXYZRGBNormal>(cloud_out.makeShared(),handler_cloud2,"cloud2",v2);
	//		while(!viewer.wasStopped())
	//		viewer.spinOnce();

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
	CorrespondencesPtr all_correspondences (new Correspondences),
			good_correspondences (new Correspondences);
	PCNormalPtr output (new PCNormal);

	*output = *src;

	int iterations = 0;
	// Convergence criteria class stops the loop
	pcl::registration::DefaultConvergenceCriteria<double> converged (iterations, transform, *good_correspondences);

	// ICP loop
	do
	{
		// Find correspondences
		timer.tic();
		findCorrespondences (output, tgt, *all_correspondences);
		//cout<<"Find Correspondances "<<": "<<timer.toc()<<" Miliseconds"<<endl;

		//		Visualize first iteration all correspondances
		//		if(iterations==0)
		//			visualizeCorrespondances(output,tgt,output,tgt,all_correspondences);
//		PCL_DEBUG ("Number of correspondences found: %d\n", all_correspondences->size ());

		if (rejection)
		{
			// Reject correspondences
			timer.tic();
			rejectBadCorrespondences (all_correspondences, output, tgt, *good_correspondences);
			//cout<<"Reject Correspondances "<<": "<<timer.toc()<<" Miliseconds"<<endl;
//			PCL_DEBUG ("Number of correspondences remaining after rejection: %d\n", good_correspondences->size ());
		}
		else
			timer.tic();
		*good_correspondences = *all_correspondences;

		//				Visualize first iteration good correspondances
		//		if(iterations==0)
		//					visualizeCorrespondances(output,tgt,output,tgt,good_correspondences);

		// Find transformation
		findTransformation (output, tgt, good_correspondences, transform);
		//cout<<"Find Transformation "<<": "<<timer.toc()<<" Miliseconds"<<endl;
		// Obtain the final transformation
		final_transform = transform * final_transform;

		// Transform the data
		timer.tic();
		transformPointCloudWithNormals (*src, *output, final_transform.cast<float> ());
		//cout<<"Transform Cloud "<<": "<<timer.toc()<<" Miliseconds"<<endl;
		// Check if convergence has been reached
		++iterations;

	}
	while (!converged);
	cout<< "Took "<<iterations<<" to converge"<<endl;
	//	cout<<"Converged "<<converged<<endl;
	//	view (src, tgt, good_correspondences);
	//	view (output, tgt, good_correspondences);
	transform = final_transform;
}

/************************************************************************************/
/************************************************************************************/
/************************************************************************************/
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

/************************************************************************************/
/************************************************************************************/
/************************************************************************************/
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

/************************************************************************************/
/************************************************************************************/
/************************************************************************************/
bool Registration::verifyCorrespondences(const PCNormalPtr &src
		, const PCNormalPtr &tgt)
{
	CorrespondencesPtr all_correspondences (new Correspondences),
			good_correspondences (new Correspondences);
	findCorrespondences (src, tgt, *all_correspondences);
	if (rejection)
	{
		// Reject correspondences
		rejectBadCorrespondences (all_correspondences, src, tgt, *good_correspondences);
	}
	else
		*good_correspondences = *all_correspondences;
	printf ("Number of correspondences from verify: %ld\n", good_correspondences->size ());
	if(good_correspondences->size()<=minThreshold)
		return true;
	return false;
}

/************************************************************************************/
/************************************************************************************/
/************************************************************************************/
void Registration::checkforKeyframe(const PCXYZRGBPtr &fullCloud
		, const PCNormalPtr &cloud
		, transformPtr &transform){

	// cout <<"inside function "<<endl<<*(transform)<<endl;
	// check for the very first frame and set it as key frame
	if(Keyframes.size()==0){
		Keyframes.push_back(cloud);
		KeyTransform.push_back(transform);
		KeyframeFull.push_back(fullCloud);
		accTransform.reset(new Eigen::Matrix4d);
		*accTransform = identityMatrix;
		//		cout<<"First key frame"<<endl;
	}
	else{
		//Add the recent transformation matrix to the last matrices since the last key frame
		*accTransform = *accTransform**transform;
		//Check for a new keyframe
		cout<<"Keyframe number : "<<Keyframes.size()<<endl;
		if(verifyCorrespondences(Keyframes[Keyframes.size() - 1],cloud)){ //todo:function to check threshold;
			//if so add it and reset the transform matrix
			Keyframes.push_back(cloud);
			KeyTransform.push_back(accTransform);
			KeyframeFull.push_back(fullCloud);
			accTransform.reset(new Eigen::Matrix4d);
			*accTransform = identityMatrix;
			cout<<"Key frame no: "<<Keyframes.size()<<endl;
		}
		//		cout<<"Stored transform"<<endl<<*(KeyTransform[KeyTransform.size()-1])<<endl;
	}

}

/************************************************************************************/
/************************************************************************************/
/************************************************************************************/
void Registration::getView(PCXYZRGBPtr &result)
{
	//If first call to the function, initialize the fullcloud image.
	gettingView = true;
	size_t size = Keyframes.size();
	if(viewframes==0 && size>0)
		fullCloud.reset(new PCXYZRGB);

	if(viewframes == size){
		gettingView = false;
		return;
	}

	Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
	std::size_t i;
	for(i=0;i<viewframes;i++)
	{
		//		cout <<"inside function: transfer no: "<<i<<endl<<*(KeyTransform[0])<<endl<<endl;
		transform*=*(KeyTransform[i]);
	}
	//		cout<<"inside function total transform previous"<<transform<<endl;
	//run for the newly acquired keyframes since last function call
	for(;viewframes<size;viewframes++){
		//		cout<<viewframes<<"  Actual Transform"<<endl<<*(KeyTransform[KeyTransform.size()-1])<<endl;
		PCXYZRGBPtr temp(new PCXYZRGB);
		PCXYZRGBPtr temp2(new PCXYZRGB);
		transform*=*(KeyTransform[viewframes]);
		//		cout<<"Applied Transform"<<endl<<transform<<endl;
		transformPointCloud (*(KeyframeFull[viewframes]), *temp, transform.cast<float> ());
		Modelization::planeDetection::voxel_filter(temp,0.005,*temp2);
		*fullCloud+=*temp2;
	}
	Modelization::planeDetection::voxel_filter(fullCloud,0.01,*result);
	cout<<"Number of keyframes : "<<size<<endl;
	viewReady = true;
	gettingView = false;
}

} /* namespace Modelization */
