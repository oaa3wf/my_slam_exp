/* This file is part of RGBDSLAM.
 * 
 * RGBDSLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * RGBDSLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with RGBDSLAM.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "my_slam/parameter_server.h"

//Documentation see header file
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/ros/conversions.h"
#include <pcl/common/distances.h>
#include <visualization_msgs/Marker.h>
#include <pcl/io/io.h>
#include <pcl/io/impl/pcd_io.hpp>
#include "pcl_ros/transforms.h"
#include "my_slam/openni_listener.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <sstream>
#include <algorithm> // std::min
#include <string>
#include <cv.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Core>
#include "my_slam/node.h"
#include "my_slam/misc.h"


//#include "opencv2/features2d/precomp.hpp"
#include "opencv2/features2d/features2d.hpp"
#ifdef CV_NONFREE
#include "opencv2/nonfree/features2d.hpp"
#endif
#include "opencv2/imgproc/imgproc.hpp"
#include "my_slam/aorb.h"
#include <cassert>
#include <iostream>
#include <algorithm>

//For rosbag reading

#include <boost/foreach.hpp>

//for comparison with ground truth from mocap and movable cameras on robots
#include <tf/transform_listener.h>

typedef message_filters::Subscriber<sensor_msgs::Image> image_sub_type;      
typedef message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo_sub_type;      
typedef message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_type;      
typedef message_filters::Subscriber<nav_msgs::Odometry> odom_sub_type;

//typedef pcl::PointCloud<pcl::PointXYZRGB> pointcloud_type;

using namespace cv;


//Use Grid or Dynamic or GridDynamic as prefix of FAST, SIFT, SURF or AORB
cv::FeatureDetector* createDetector(const std::string& detectorName){
  //For anything but SIFTGPU
  cv::FeatureDetector* detAdj = NULL;

  //ROS_INFO_STREAM("Using " << detectorName << " keypoint detector.");
  if( detectorName == "SIFTGPU" ) {
    return NULL;// Will not be used
  } 
  else if(detectorName == "FAST") {
     detAdj = new FastFeatureDetector(20); //new DetectorAdjuster("FAST", 20);
  }
#ifdef CV_NONFREE
  else if(detectorName == "SURF" || detectorName == "SURF128") {
     detAdj = new SurfFeatureDetector(200); //new DetectorAdjuster("SURF", 200);
  }
  else if(detectorName == "SIFT") {
     detAdj = new SiftFeatureDetector(0 /*max_features*/, 3 /*default lvls/octave*/, 0.04); //DetectorAdjuster("SIFT", 0.04, 0.0001);
  }
#else
  else if(detectorName == "SURF" || detectorName == "SURF128" || detectorName == "SIFT") 
  {
     //ROS_ERROR("OpenCV non-free functionality (%s) not built in.", detectorName.c_str());
     //ROS_ERROR("To enable non-free functionality build with CV_NONFREE set.");
		 std::cerr << "OpenCV non-free functionality is not built in." << std::endl;
     //ROS_WARN("Using ORB feature detection instead.");
     //ParameterServer::instance()->set("feature_detector_type", std::string("ORB"));
     detAdj = new AorbFeatureDetector(10000, 1.2, 8, 15, 0, 2, 0, 31, static_cast<int>(20)); //DetectorAdjuster("AORB", 20);
  }
#endif
  else if(detectorName == "ORB") {
     detAdj = new AorbFeatureDetector(10000, 1.2, 8, 15, 0, 2, 0, 31, static_cast<int>(20)); //new DetectorAdjuster("AORB", 20);
  } 
  else {
    //ROS_ERROR("Unsupported Keypoint Detector. Using ORB as fallback.");
		std::cerr << "Unsupported Keypoint Detector. Using ORB as fallback." <<std::endl;
    return createDetector("ORB");
  }
  assert(detAdj != NULL && "No valid detector aduster");

  //ParameterServer* params = ParameterServer::instance();
  //bool gridWrap = (params->get<int>("detector_grid_resolution") > 1);
  //bool dynaWrap = (params->get<int>("adjuster_max_iterations") > 0);

  //if(dynaWrap && gridWrap){
    //return adjustedGridWrapper(detAdj);
  //}
  //else if(dynaWrap){
    //int min = params->get<int>("max_keypoints");
    //int max = min * 1.5; //params->get<int>("max_keypoints");
    //return adjusterWrapper(detAdj, min, max);
  //}
  return detAdj;
}

cv::DescriptorExtractor* createDescriptorExtractor(const std::string& descriptorType) 
{
    DescriptorExtractor* extractor = 0;
    if(descriptorType == "ORB") {
        extractor = new OrbDescriptorExtractor();
    }
#ifdef CV_NONFREE
    else if(descriptorType == "SIFT") {
        extractor = new SiftDescriptorExtractor();/*( double magnification=SIFT::DescriptorParams::GET_DEFAULT_MAGNIFICATION(), bool isNormalize=true, bool recalculateAngles=true, int nOctaves=SIFT::CommonParams::DEFAULT_NOCTAVES, int nOctaveLayers=SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS, int firstOctave=SIFT::CommonParams::DEFAULT_FIRST_OCTAVE, int angleMode=SIFT::CommonParams::FIRST_ANGLE )*/
    }
    else if(descriptorType == "SURF") {
        extractor = new SurfDescriptorExtractor();/*( int nOctaves=4, int nOctaveLayers=2, bool extended=false )*/
    }
    else if(descriptorType == "SURF128") {
        extractor = new SurfDescriptorExtractor();/*( int nOctaves=4, int nOctaveLayers=2, bool extended=false )*/
        extractor->set("extended", 1);
    }
#else
    else if(descriptorType == "SURF128" || descriptorType == "SIFT" || descriptorType == "SURF") 
    {
        //ROS_ERROR("OpenCV non-free functionality (%s) not built in.", descriptorType.c_str());
        //ROS_ERROR("To enable non-free functionality build with CV_NONFREE set.");
        //ROS_WARN("Using ORB feature detection instead.");
				std::cerr << "OpenCV non-free functionality is not built in." <<std::endl;
        //ParameterServer::instance()->set("feature_extractor_type", std::string("ORB"));
        return createDescriptorExtractor("ORB");
    }
#endif
    else if(descriptorType == "BRIEF") {
        extractor = new BriefDescriptorExtractor();
    }
    else if(descriptorType == "BRISK") {
        extractor = new cv::BRISK();/*brisk default: (int thresh=30, int octaves=3, float patternScale=1.0f)*/
    }
    else if(descriptorType == "FREAK") {
        extractor = new cv::FREAK();
    }
    else if(descriptorType == "SIFTGPU") {
      ROS_DEBUG("%s is to be used as extractor, creating ORB descriptor extractor as fallback.", descriptorType.c_str());
      return createDescriptorExtractor("ORB");
    }
    else {
      ROS_ERROR("No valid descriptor-matcher-type given: %s. Using ORB", descriptorType.c_str());
      return createDescriptorExtractor("ORB");
    }
    assert(extractor != 0 && "No extractor could be created");
    return extractor;
}


void OpenNIListener::visualize_images(cv::Mat visual_image, cv::Mat depth_image){
  if(ParameterServer::instance()->get<bool>("visualize_mono_depth_overlay")){
    cv::Mat visual_edges = cv::Mat( visual_image.rows, visual_image.cols, CV_8UC1); 
    cv::Mat depth_edges  = cv::Mat( depth_image.rows, depth_image.cols, CV_8UC1); 
    overlay_edges(visual_image, depth_image, visual_edges, depth_edges);
    Q_EMIT newDepthImage(cvMat2QImage(depth_image,depth_image, depth_image+visual_edges, 1)); //show registration by edge overlay
    cv::Mat monochrome; 
    if(visual_image.type() != CV_8UC1) {
      monochrome = cv::Mat( visual_image.rows, visual_image.cols, CV_8UC1); 
      cv::cvtColor(visual_image, monochrome, CV_RGB2GRAY);
    } else {
      monochrome = visual_image; 
    }
    Q_EMIT newVisualImage(cvMat2QImage(monochrome, monochrome + depth_edges, monochrome, 0)); //visual_idx=0
  } else { // No overlay
    Q_EMIT newVisualImage(cvMat2QImage(visual_image, 0)); //visual_idx=0
    Q_EMIT newDepthImage(cvMat2QImage(depth_image, 1)); 
  }
}


OpenNIListener::OpenNIListener(GraphManager* graph_mgr)
: graph_mgr_(graph_mgr),
  stereo_sync_(NULL), kinect_sync_(NULL), no_cloud_sync_(NULL),
  visua_sub_(NULL), depth_sub_(NULL), cloud_sub_(NULL),
  depth_mono8_img_(cv::Mat()),
  data_id_(0),
  num_processed_(0),
  image_encoding_("rgb8")
{
  
  //detector_ = createDetector("ORB");
  //extractor_ = createDescriptorExtractor("FREAK");
  //setupSubscribers();


	ParameterServer* ps = ParameterServer::instance();
  if(ps->get<bool>("encoding_bgr")){
    image_encoding_ = "bgr8";//used in conversion to qimage. exact value does not matter, just not rgb8
  }
  detector_ = createDetector(ps->get<std::string>("feature_detector_type"));
  extractor_ = createDescriptorExtractor(ps->get<std::string>("feature_extractor_type"));
  setupSubscribers();
}


void OpenNIListener::setupSubscribers(){
  //ParameterServer* ps = ParameterServer::instance();
  //int q = 50;
	ParameterServer* ps = ParameterServer::instance();
  int q = ps->get<int>("subscriber_queue_size");
  ros::NodeHandle nh;
	graph_mgr_->marker_pub_ = nh.advertise<visualization_msgs::Marker>("/rgbdslam/pose_graph_markers",0);
  tflistener_ = new tf::TransformListener(nh);
  if(true){
	
		/**
    std::string visua_tpc = "/camera/color/image_raw";
    std::string depth_tpc = "/camera/depth/image_raw";
    std::string cinfo_tpc = "/camera/depth/camera_info";
    std::string cloud_tpc = "/camera/depth/points";
    std::string widev_tpc = "";
    std::string widec_tpc = "";
		**/


		std::string visua_tpc = ps->get<std::string>("topic_image_mono");
    std::string depth_tpc = ps->get<std::string>("topic_image_depth");
    std::string cinfo_tpc = ps->get<std::string>("camera_info_topic");
    std::string cloud_tpc = ps->get<std::string>("topic_points");
    std::string widev_tpc = ps->get<std::string>("wide_topic");
    std::string widec_tpc = ps->get<std::string>("wide_cloud_topic");

    //All information from Kinect
    if(!visua_tpc.empty() && !depth_tpc.empty() && !cloud_tpc.empty())
    {   
        visua_sub_ = new image_sub_type(nh, visua_tpc, q);
        depth_sub_ = new image_sub_type (nh, depth_tpc, q);
        cloud_sub_ = new pc_sub_type (nh, cloud_tpc, q);  
        kinect_sync_ = new message_filters::Synchronizer<KinectSyncPolicy>(KinectSyncPolicy(q),  *visua_sub_, *depth_sub_, *cloud_sub_),
        kinect_sync_->registerCallback(boost::bind(&OpenNIListener::kinectCallback, this, _1, _2, _3));
        //ROS_INFO_STREAM_NAMED("OpenNIListener", "Listening to " << visua_tpc << ", " << depth_tpc << " and " << cloud_tpc);
				std::cout << "OpenNIListener, Listening to " << visua_tpc << ", " << depth_tpc << " and " << cloud_tpc <<std::endl;
    } 
    //No cloud, but visual image and depth
    else if(!visua_tpc.empty() && !depth_tpc.empty() && !cinfo_tpc.empty() && cloud_tpc.empty())
    {   
        visua_sub_ = new image_sub_type(nh, visua_tpc, q);
        depth_sub_ = new image_sub_type(nh, depth_tpc, q);
        cinfo_sub_ = new cinfo_sub_type(nh, cinfo_tpc, q);
        no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(q),  *visua_sub_, *depth_sub_, *cinfo_sub_);
        no_cloud_sync_->registerCallback(boost::bind(&OpenNIListener::noCloudCallback, this, _1, _2, _3));
        //ROS_INFO_STREAM_NAMED("OpenNIListener", "Listening to " << visua_tpc << " and " << depth_tpc);
				std::cout << "OpenNIListener, Listening to " << visua_tpc << " and " << depth_tpc << std::endl;
    } 

    //All information from stereo                                               
    if(!widec_tpc.empty() && !widev_tpc.empty())
    {   
      visua_sub_ = new image_sub_type(nh, widev_tpc, q);
      cloud_sub_ = new pc_sub_type(nh, widec_tpc, q);
      stereo_sync_ = new message_filters::Synchronizer<StereoSyncPolicy>(StereoSyncPolicy(q), *visua_sub_, *cloud_sub_);
      stereo_sync_->registerCallback(boost::bind(&OpenNIListener::stereoCallback, this, _1, _2));
      //ROS_INFO_STREAM_NAMED("OpenNIListener", "Listening to " << widev_tpc << " and " << widec_tpc );
			std::cout << "OpenNIListener, Listening to " << widev_tpc << " and " << widec_tpc <<std::endl;
    } 

		/**
    if(ps->get<bool>("use_robot_odom")){
    	odom_sub_= new odom_sub_type(nh, ps->get<std::string>("odometry_tpc"), 3);
      ROS_INFO_STREAM_NAMED("OpenNIListener", "Listening to odometry on " << ps->get<std::string>("odometry_tpc"));
    	odom_sub_->registerCallback(boost::bind(&OpenNIListener::odomCallback,this,_1));
    }
		**/
  } 
  else {
    std::cerr<< "Couldn't suscribe to the right things" << std::endl;
  }
}


static void calculateDepthMask(cv::Mat_<uchar>& depth_img, const pointcloud_type::Ptr point_cloud)
{
    //calculate depthMask
    float value;
    for(unsigned int y = 0; y < depth_img.rows; y++){
        for(unsigned int x = 0; x < depth_img.cols; x++){
            value = point_cloud->at(x,y).z; 
            if(value != value){//isnan
                depth_img.at<uchar>(y,x) = 0;
            }else{
                depth_img.at<uchar>(y,x) = static_cast<uchar>(value*50.0); //Full white at ~5 meter
            }
        }
    }
}


void OpenNIListener::stereoCallback(const sensor_msgs::ImageConstPtr& visual_img_msg, 
                                    const sensor_msgs::PointCloud2ConstPtr& point_cloud)
{
    //ScopedTimer s(__FUNCTION__);
    //ROS_INFO_NAMED("OpenNIListener", "Received data from stereo cam");
    //ROS_WARN_ONCE_NAMED("eval", "First RGBD-Data Received");
    
		ParameterServer* ps = ParameterServer::instance();
    if(++data_id_ < ps->get<int>("skip_first_n_frames") 
       || data_id_ % ps->get<int>("data_skip_step") != 0){ 
      ROS_INFO_THROTTLE_NAMED(1, "OpenNIListener", "Skipping Frame %i because of data_skip_step setting (this msg is only shown once a sec)", data_id_);
      if(ps->get<bool>("use_gui")){//Show the image, even if not using it
        cv::Mat visual_img =  cv_bridge::toCvCopy(visual_img_msg)->image;
        Q_EMIT newVisualImage(cvMat2QImage(visual_img, 0)); //visual_idx=0
      }
      return;
    }


    //calculate depthMask
    pointcloud_type::Ptr pc_col(new pointcloud_type());//will belong to node
    pcl::fromROSMsg(*point_cloud,*pc_col);
    cv::Mat_<uchar> depth_img(visual_img_msg->height, visual_img_msg->width);
    calculateDepthMask(depth_img, pc_col);

    //Get images into OpenCV format
    //sensor_msgs::CvBridge bridge;
    cv::Mat visual_img =  cv_bridge::toCvCopy(visual_img_msg, "mono8")->image;
    if(visual_img.rows != depth_img.rows ||
       visual_img.cols != depth_img.cols ||
       point_cloud->width != (uint32_t) visual_img.cols ||
       point_cloud->height != (uint32_t) visual_img.rows){
      //ROS_ERROR("PointCloud, depth and visual image differ in size! Ignoring Data");
			std::cerr<< "PointCloud, depth and visual image differ in size! Ignoring Data" <<std::endl;
      return;
    }

    cameraCallback(visual_img, pc_col, depth_img);
}


OpenNIListener::~OpenNIListener(){
  delete tflistener_;
}

void OpenNIListener::noCloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                                      const sensor_msgs::ImageConstPtr& depth_img_msg,
                                      const sensor_msgs::CameraInfoConstPtr& cam_info_msg) 
{
  //ScopedTimer s(__FUNCTION__);
  //ROS_WARN_ONCE_NAMED("eval", "First RGBD-Data Received");
  //ROS_DEBUG("Received data from kinect");
  ParameterServer* ps = ParameterServer::instance();

	if(++data_id_ < ps->get<int>("skip_first_n_frames") 
     || data_id_ % ps->get<int>("data_skip_step") != 0)
  { 
  // If only a subset of frames are used, skip computations but visualize if gui is running
    ROS_INFO_THROTTLE_NAMED(1, "OpenNIListener", "Skipping Frame %i because of data_skip_step setting (this msg is only shown once a sec)", data_id_);
    if(ps->get<bool>("use_gui")){//Show the image, even if not using it
      //cv::Mat depth_float_img = cv_bridge::toCvCopy(depth_img_msg)->image;
      cv::Mat visual_img =  cv_bridge::toCvCopy(visual_img_msg)->image;
      //if(visual_img.rows != depth_float_img.rows || 
      //   visual_img.cols != depth_float_img.cols){
      //  ROS_ERROR("depth and visual image differ in size! Ignoring Data");
      //  return;
      //}
      //depthToCV8UC1(depth_float_img, depth_mono8_img_); //float can't be visualized or used as mask in float format TODO: reprogram keypoint detector to use float values with nan to mask
      //image_encoding_ = visual_img_msg->encoding;
      //Q_EMIT newVisualImage(cvMat2QImage(visual_img, 0)); //visual_idx=0
      //Q_EMIT newDepthImage (cvMat2QImage(depth_mono8_img_,1));//overwrites last cvMat2QImage
    }
    return;
  }

  //Convert images to OpenCV format
  //sensor_msgs::CvBridge bridge;
  //cv::Mat depth_float_img = bridge.imgMsgToCv(depth_img_msg);
  //cv::Mat visual_img =  bridge.imgMsgToCv(visual_img_msg);
  cv::Mat depth_float_img = cv_bridge::toCvCopy(depth_img_msg)->image;
  //const cv::Mat& depth_float_img_big = cv_bridge::toCvShare(depth_img_msg)->image;
  cv::Mat visual_img;

  //ROS_INFO_STREAM("Encoding: " << visual_img_msg->encoding);
  visual_img =  cv_bridge::toCvCopy(visual_img_msg)->image;
  
  //const cv::Mat& visual_img_big =  cv_bridge::toCvShare(visual_img_msg)->image;
  //cv::Size newsize(320, 240);
  //cv::Mat visual_img(newsize, visual_img_big.type()), depth_float_img(newsize, depth_float_img_big.type());
  //cv::resize(visual_img_big, visual_img, newsize);
  //cv::resize(depth_float_img_big, depth_float_img, newsize);
  if(visual_img.rows != depth_float_img.rows || 
     visual_img.cols != depth_float_img.cols){
    //ROS_WARN("depth and visual image differ in size! Rescaling Depth Data");
     cv::resize(depth_float_img, depth_float_img, visual_img.size(), 0,0,cv::INTER_NEAREST);
    //return;
  }
  image_encoding_ = visual_img_msg->encoding;

  depthToCV8UC1(depth_float_img, depth_mono8_img_); //float can't be visualized or used as mask in float format TODO: reprogram keypoint detector to use float values with nan to mask

  //if(asyncFrameDrop(depth_img_msg->header.stamp, visual_img_msg->header.stamp)) 
    //return;

  noCloudCameraCallback(visual_img, depth_float_img, depth_mono8_img_, visual_img_msg->header, cam_info_msg);
}


void OpenNIListener::kinectCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                                     const sensor_msgs::ImageConstPtr& depth_img_msg,   
                                     const sensor_msgs::PointCloud2ConstPtr& point_cloud) 
{
  /// \callgraph
  //ScopedTimer s(__FUNCTION__);
  //ROS_DEBUG("Received data from kinect");
  //ROS_WARN_ONCE_NAMED("eval", "First RGBD-Data Received");
  ParameterServer* ps = ParameterServer::instance();

	
  if(++data_id_ < ps->get<int>("skip_first_n_frames") 
     || data_id_ % ps->get<int>("data_skip_step") != 0)
  { 
  // If only a subset of frames are used, skip computations but visualize if gui is running
    ROS_INFO_THROTTLE_NAMED(1, "OpenNIListener", "Skipping Frame %i because of data_skip_step setting (this msg is only shown once a sec)", data_id_);
    if(ps->get<bool>("use_gui")){//Show the image, even if not using it
      //cv::Mat depth_float_img = cv_bridge::toCvCopy(depth_img_msg)->image;
      cv::Mat visual_img =  cv_bridge::toCvCopy(visual_img_msg)->image;
      //if(visual_img.rows != depth_float_img.rows || 
      //   visual_img.cols != depth_float_img.cols){
      //  ROS_ERROR("depth and visual image differ in size! Ignoring Data");
      //  return;
      //}
      //depthToCV8UC1(depth_float_img, depth_mono8_img_); //float can't be visualized or used as mask in float format TODO: reprogram keypoint detector to use float values with nan to mask
      //image_encoding_ = visual_img_msg->encoding;
      //Q_EMIT newVisualImage(cvMat2QImage(visual_img, 0)); //visual_idx=0
      //Q_EMIT newDepthImage (cvMat2QImage(depth_mono8_img_,1));//overwrites last cvMat2QImage
    }
    return;
  }


  //Get images into OpenCV format
  //sensor_msgs::CvBridge bridge;
  //cv::Mat depth_float_img = bridge.imgMsgToCv(depth_img_msg);
  //cv::Mat visual_img =  bridge.imgMsgToCv(visual_img_msg);
  cv::Mat depth_float_img = cv_bridge::toCvCopy(depth_img_msg)->image;
  cv::Mat visual_img =  cv_bridge::toCvCopy(visual_img_msg)->image;
  if(visual_img.rows != depth_float_img.rows || 
     visual_img.cols != depth_float_img.cols ||
     point_cloud->width != (uint32_t) visual_img.cols ||
     point_cloud->height != (uint32_t) visual_img.rows){
     //ROS_ERROR("PointCloud, depth and visual image differ in size! Ignoring Data");
		 std::cerr << "PointCloud, depth and visual image differ in size! Ignoring Data" <<std::endl;
    return;
  }
  image_encoding_ = visual_img_msg->encoding;
  depthToCV8UC1(depth_float_img, depth_mono8_img_); //float can't be visualized or used as mask in float format TODO: reprogram keypoint detector to use float values with nan to mask

  //if(asyncFrameDrop(depth_img_msg->header.stamp, visual_img_msg->header.stamp)) 
    //return;



  pointcloud_type::Ptr pc_col(new pointcloud_type());//will belong to node
  pcl::fromROSMsg(*point_cloud,*pc_col);
  cameraCallback(visual_img, pc_col, depth_mono8_img_);
}




void OpenNIListener::cameraCallback(cv::Mat visual_img, 
                                    pointcloud_type::Ptr point_cloud, 
                                    cv::Mat depth_mono8_img)
{
  //ScopedTimer s(__FUNCTION__);
  //ROS_WARN_COND(point_cloud ==NULL, "Nullpointer for pointcloud");
  

  //######### Main Work: create new node ##############################################################
  //Q_EMIT setGUIStatus("Computing Keypoints and Features");
  Node* node_ptr = new Node(visual_img, detector_, extractor_, point_cloud, depth_mono8_img);

  //retrieveTransformations(pcl_conversions::fromPCL(point_cloud->header), node_ptr);
  callProcessing(visual_img, node_ptr);
}






void OpenNIListener::noCloudCameraCallback(cv::Mat visual_img, 
                                           cv::Mat depth, 
                                           cv::Mat depth_mono8_img,
                                           std_msgs::Header depth_header,
                                           const sensor_msgs::CameraInfoConstPtr& cam_info)
{
  
  //ScopedTimer s(__FUNCTION__);
  //######### Main Work: create new node ##############################################################
  //Q_EMIT setGUIStatus("Computing Keypoints and Features");
  //Node* node_ptr = new Node(visual_img, depth, depth_mono8_img, cam_info, depth_header, detector_, extractor_);

  //retrieveTransformations(depth_header, node_ptr);//Retrieve the transform between the lens and the base-link at capturing time;
  //callProcessing(visual_img, node_ptr);
}



//Call function either regularly or as background thread
void OpenNIListener::callProcessing(cv::Mat visual_img, Node* node_ptr)
{
  if(!future_.isFinished()){
    //ScopedTimer s("New Node is ready, waiting for graph manager.");
    future_.waitForFinished(); //Wait if GraphManager ist still computing. 
  }

  //update for visualization of the feature flow
  visualization_img_ = visual_img; //No copy
  visualization_depth_mono8_img_ = depth_mono8_img_;//No copy
  //With this define, processNode runs in the background and after finishing visualizes the results
  //Thus another Callback can be started in the meantime, to create a new node in parallel
  if(ParameterServer::instance()->get<bool>("concurrent_node_construction")) {
    //ROS_DEBUG("Processing Node in parallel to the construction of the next node");
		if(ParameterServer::instance()->get<bool>("use_gui")){
      //visual_img points to the data received in the callback - which might be deallocated after the callback returns. 
      //This will happen before visualization if processNode is running in background, therefore the data needs to be cloned
      visualization_img_ = visual_img.clone();
      visualization_depth_mono8_img_ = depth_mono8_img_.clone();
      visualize_images(visualization_img_, depth_mono8_img_);
    }
    future_ = QtConcurrent::run(this, &OpenNIListener::processNode, node_ptr); 
  }
  else { //Non-concurrent
    processNode(node_ptr);//regular function call
  }
}


void OpenNIListener::processNode(Node* new_node)
{
  //ScopedTimer s(__FUNCTION__);
  //Q_EMIT setGUIStatus("Adding Node to Graph");
  bool has_been_added = graph_mgr_->addNode(new_node);
  ++num_processed_;
  //Q_EMIT setGUIInfo2(QString("Frames processed: ")+QString::number(num_processed_));

/**
  ///ODOMETRY
  if(has_been_added && !ps->get<std::string>("odom_frame_name").empty()){
    ROS_INFO_NAMED("OpenNIListener", "Verifying Odometry Information");
    ros::Time latest_odom_time;
    std::string odom_frame  = ps->get<std::string>("odom_frame_name");
    std::string base_frame  = ps->get<std::string>("base_frame_name");
    std::string error_msg;
    int ev = tflistener_->getLatestCommonTime(odom_frame, base_frame, latest_odom_time, &error_msg); 
    if(ev == tf::NO_ERROR){
      graph_mgr_->addOdometry(latest_odom_time, tflistener_);
    } else {
      ROS_WARN_STREAM("Couldn't get time of latest tf transform between " << odom_frame << " and " << base_frame << ": " << error_msg);
    }
  }
**/

  //######### Visualization code  #############################################

  if(!has_been_added) {
    delete new_node; 
    new_node = NULL;
  }
}


/**
void OpenNIListener::togglePause(){
  pause_ = !pause_;
  ROS_INFO_NAMED("OpenNIListener", "Pause toggled to: %s", pause_? "true":"false");
  if(pause_) Q_EMIT setGUIStatus("Processing Thread Paused");
  else Q_EMIT setGUIStatus("Processing Thread Running");
}
**/


/// Create a QImage from image. The QImage stores its data in the rgba_buffers_ indexed by idx (reused/overwritten each call)
QImage OpenNIListener::cvMat2QImage(const cv::Mat& channel1, const cv::Mat& channel2, const cv::Mat& channel3, unsigned int idx)
{
  if(rgba_buffers_.size() <= idx){
      rgba_buffers_.resize(idx+1);
  }
  if(channel2.rows != channel1.rows || channel2.cols != channel1.cols ||
     channel3.rows != channel1.rows || channel3.cols != channel1.cols){
     //ROS_ERROR("Image channels to be combined differ in size");
		 std::cerr << "Image channels to be combined differ in size" <<std::endl;
  }
  if(channel1.rows != rgba_buffers_[idx].rows || channel1.cols != rgba_buffers_[idx].cols){
    //ROS_DEBUG("Created new rgba_buffer with index %i", idx);
    rgba_buffers_[idx] = cv::Mat( channel1.rows, channel1.cols, CV_8UC4); 
    //printMatrixInfo(rgba_buffers_[idx]);
  }
  cv::Mat mono_channel1 = channel1; 
  if(channel1.type() != CV_8UC1) {
    mono_channel1 = cv::Mat( channel1.rows, channel1.cols, CV_8UC1); 
    cv::cvtColor(channel1, mono_channel1, CV_RGB2GRAY);
  }
  std::vector<cv::Mat> input;
  cv::Mat alpha( channel1.rows, channel1.cols, CV_8UC1, cv::Scalar(255)); //TODO this could be buffered for performance
  input.push_back(mono_channel1);
  input.push_back(channel2);
  input.push_back(channel3);
  input.push_back(alpha);
  cv::merge(input, rgba_buffers_[idx]);
  //printMatrixInfo(rgba_buffers_[idx]);
  return QImage((unsigned char *)(rgba_buffers_[idx].data), 
                rgba_buffers_[idx].cols, rgba_buffers_[idx].rows, 
                rgba_buffers_[idx].step, QImage::Format_RGB32 );
}


/// Create a QImage from image. The QImage stores its data in the rgba_buffers_ indexed by idx (reused/overwritten each call)
QImage OpenNIListener::cvMat2QImage(const cv::Mat& image, unsigned int idx)
{
  //ROS_DEBUG_STREAM("Converting Matrix of type " << openCVCode2String(image.type()) << " to RGBA");
  if(rgba_buffers_.size() <= idx){
      rgba_buffers_.resize(idx+1);
  }
  //ROS_DEBUG_STREAM("Target Matrix is of type " << openCVCode2String(rgba_buffers_[idx].type()));
  if(image.rows != rgba_buffers_[idx].rows || image.cols != rgba_buffers_[idx].cols){
    //ROS_DEBUG("Created new rgba_buffer with index %i", idx);
    rgba_buffers_[idx] = cv::Mat( image.rows, image.cols, CV_8UC4); 
    //printMatrixInfo(rgba_buffers_[idx], "for QImage Buffering");
  }
  char red_idx = 0, green_idx = 1, blue_idx = 2;
  if(image_encoding_.compare("rgb8") == 0) { red_idx = 2; blue_idx = 0; }
  cv::Mat alpha( image.rows, image.cols, CV_8UC1, cv::Scalar(255)); //TODO this could be buffered for performance
  cv::Mat in[] = { image, alpha };
  // rgba[0] -> bgr[2], rgba[1] -> bgr[1],
  // rgba[2] -> bgr[0], rgba[3] -> alpha[0]

  if(image.type() == CV_8UC1){
    int from_to[] = { 0,0,  0,1,  0,2,  1,3 };
    mixChannels( in , 2, &rgba_buffers_[idx], 1, from_to, 4 );
  } else {
    int from_to[] = { red_idx,0,  green_idx,1,  blue_idx,2,  3,3 }; //BGR+A -> RGBA
    mixChannels( in , 2, &rgba_buffers_[idx], 1, from_to, 4 );
  }
  //printMatrixInfo(rgba_buffers_[idx], "for QImage Buffering");
  return QImage((unsigned char *)(rgba_buffers_[idx].data), 
                rgba_buffers_[idx].cols, rgba_buffers_[idx].rows, 
                rgba_buffers_[idx].step, QImage::Format_RGB32 );
}

/**
//! Callback for the robot odometry
void OpenNIListener::odomCallback(const nav_msgs::OdometryConstPtr& odom_msg){
  tf::Transform tfTransf;
  tf::poseMsgToTF (odom_msg->pose.pose, tfTransf);
  tf::StampedTransform stTransf(tfTransf, odom_msg->header.stamp, odom_msg->header.frame_id, odom_msg->child_frame_id); 
  printTransform("Odometry Transformation", stTransf);
  tflistener_->setTransform(stTransf);
  //Is done now after creation of Node
	//graph_mgr_->addOdometry(odom_msg->header.stamp,tflistener_);
}
**/

