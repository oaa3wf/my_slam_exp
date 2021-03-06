#ifndef MY_SLAM_NODE_H
#define MY_SLAM_NODE_H

#include "ros/ros.h"
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <Eigen/Core>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_datatypes.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <Eigen/StdVector>
#include "my_slam/matching_result.h" 

typedef std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > std_vector_of_eigen_vector4f;

class Node{
public:
	Node(const cv::Mat visual,
			const cv::Mat& detection_mask,
			cv::Ptr<cv::FeatureDetector> detector,
			cv::Ptr<cv::DescriptorExtractor> extractor,
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud);

	Node(const cv::Mat visual,
           cv::Ptr<cv::FeatureDetector> detector,
           cv::Ptr<cv::DescriptorExtractor> extractor,
           pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
           const cv::Mat detection_mask);

	Node(){};
	~Node();
	

	//Compare nodes and give transformation between them
	MatchingResult matchNodePair(const Node* older_node);


		//! return the 3D projection of valid keypoints using information from the point cloud and remove invalid keypoints (NaN depth) 
	void projectTo3D(std::vector<cv::KeyPoint>& feature_locations_2d,  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& feature_locations_3d,const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr point_cloud);


  ///Transform, e.g., from kinematics
  void setBase2PointsTransform(tf::StampedTransform& b2p);


  ///Transform, e.g., from kinematics
  tf::StampedTransform getBase2PointsTransform() const;


  //!Fills "matches" and returns ratio of "good" features 
  //!in the sense of distinction via the "nn_distance_ratio" setting (see parameter server)
	unsigned int featureMatching(const Node* other, std::vector<cv::DMatch>* matches)const;

	bool getRelativeTransformationTo(const Node* earlier_node,
                                       std::vector<cv::DMatch>* initial_matches,
                                       Eigen::Matrix4f& resulting_transformation,
                                       float& rmse, 
                                       std::vector<cv::DMatch>& matches)const;

	bool getRelativeTransformationPCL(const Node* earlier_node,
                                       std::vector<cv::DMatch>* initial_matches,
                                       Eigen::Matrix4f& resulting_transformation,
                                       float& rmse, 
                                       std::vector<cv::DMatch>& matches) const;



	void computeInliersAndError(const std::vector<cv::DMatch> & initial_matches,
                              const Eigen::Matrix4f& transformation,
                              const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& origins,
                              const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& targets,
                              size_t min_inliers, //break if this can't be reached
                              std::vector<cv::DMatch>& new_inliers, //pure output var
                              double& mean_error, //pure output var //std::vector<double>& errors,
                              double squaredMaxInlierDistInM = 0.0009)const; //output var;

	int id_;         //<number of camera nodes in the graph when the node was added
	int seq_id_;      //<number of images that have been processed (even if they were not added)
	int vertex_id_;   //<id of the corresponding vertex in the g2o graph
	bool valid_tf_estimate_;      //<Flags whether the data of this node should be considered for postprocessing steps, e.g., visualization, trajectory, map creation
	bool matchable_;        //< Flags whether the data for matching is (still) available
	ros::Time timestamp_; 		//< Time of the data belonging to the node
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_col;
	
	///descriptor definitions
	cv::Mat feature_descriptors_;         

  ///backprojected 3d descriptor locations relative to cam position in homogeneous coordinates (last dimension is 1.0)
	std_vector_of_eigen_vector4f feature_locations_3d_;  

  ///Where in the image are the descriptors
	std::vector<cv::KeyPoint> feature_locations_2d_; 

	sensor_msgs::CameraInfo cam_info_; 

	int initial_node_matches_;

  tf::StampedTransform base2points_; //!<contains the transformation from the base (defined on param server) to the point_cloud

	std::string base_frame_name;
	std::string feature_extractor_type_;
	std::string matcher_type_;

	int min_matches_;
	float max_dist_for_inliers_;
	int ransac_iterations_;
	cv::Mat m_img;



	
};

#endif
