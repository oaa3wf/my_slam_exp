#include "my_slam/node.h"
#include "my_slam/misc.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <std_msgs/Header.h>
#include <pcl/PCLHeader.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transformation_from_correspondences.h>

#include <Eigen/Core>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
//#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/visualization/pcl_visualizer.h>

#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>


typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;



///Randomly choose <sample_size> of the matches
std::vector<cv::DMatch> sample_matches_prefer_by_distance(unsigned int sample_size, std::vector<cv::DMatch>& matches_with_depth)
{
    //Sample ids to pick matches lateron (because they are unique and the
    //DMatch operator< overload breaks uniqueness of the Matches if they have the
    //exact same distance, e.g., 0.0)
    std::set<std::vector<cv::DMatch>::size_type> sampled_ids;
    int safety_net = 0;
    while(sampled_ids.size() < sample_size && matches_with_depth.size() >= sample_size){
      //generate a set of samples. Using a set solves the problem of drawing a sample more than once
      int id1 = rand() % matches_with_depth.size();
      int id2 = rand() % matches_with_depth.size();
      if(id1 > id2) id1 = id2; //use smaller one => increases chance for lower id
      sampled_ids.insert(id1);
      if(++safety_net > 10000){ //ROS_ERROR("Infinite Sampling"); 
				break; } 
    }

    //Given the ids, construct the resulting vector
    std::vector<cv::DMatch> sampled_matches;
    sampled_matches.reserve(sampled_ids.size());
    BOOST_FOREACH(std::vector<cv::DMatch>::size_type id, sampled_ids){
      sampled_matches.push_back(matches_with_depth[id]);
    }
    return sampled_matches;
}





int bruteForceSearchORB(const uint64_t* v, const uint64_t* search_array, const unsigned int& size, int& result_index){
  //constexpr unsigned int howmany64bitwords = 4;//32*8/64;
	/**
  const unsigned int howmany64bitwords = 4;//32*8/64;
  assert(search_array && "Nullpointer in bruteForceSearchORB");
  result_index = -1;//impossible
  int min_distance = 1 + 256;//More than maximum distance
  for(unsigned int i = 0; i < size-1; i+=1, search_array+=4){
    int hamming_distance_i = hamming_distance_orb32x8_popcountll(v, search_array);
    if(hamming_distance_i < min_distance){
      min_distance = hamming_distance_i;
      result_index = i;
    }
  } 
  return min_distance;
	**/
	return 5;
}


Eigen::Matrix4f getTransformFromMatches(const Node* newer_node, const Node* earlier_node,const std::vector<cv::DMatch>& matches,bool& valid, const float max_dist_m) 
{
  pcl::TransformationFromCorrespondences tfc;
  valid = true;
  std::vector<Eigen::Vector3f> t, f;
  float weight = 1.0;

  BOOST_FOREACH(const cv::DMatch& m, matches)
  {
    Eigen::Vector3f from = newer_node->feature_locations_3d_[m.queryIdx].head<3>();
    Eigen::Vector3f to = earlier_node->feature_locations_3d_[m.trainIdx].head<3>();
    if(isnan(from(2)) || isnan(to(2)))
      continue;

    weight = 1.0/(from(2) * to(2));

    if(false){//is that code useful?
      //Validate that 3D distances are corresponding
      if (max_dist_m > 0) {  //storing is only necessary, if max_dist is given
        if(f.size() >= 1)
        {
          float delta_f = (from - f.back()).squaredNorm();//distance to the previous query point
          float delta_t = (to   - t.back()).squaredNorm();//distance from one to the next train point

          if ( abs(delta_f-delta_t) > max_dist_m * max_dist_m ) {
            valid = false;
            return Eigen::Matrix4f();
          }
        }
        f.push_back(from);
        t.push_back(to);    
      }
    }

    tfc.add(from, to, weight);// 1.0/(to(2)*to(2)));//the further, the less weight b/c of quadratic accuracy decay
  }


  // get relative movement from samples
  return tfc.getTransformation().matrix();

}


Node::Node(const cv::Mat visual,
		   const cv::Mat& detection_mask,
           cv::Ptr<cv::FeatureDetector> detector,
           cv::Ptr<cv::DescriptorExtractor> extractor,
           pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud) : 
  id_(-1), seq_id_(-1), vertex_id_(-1), valid_tf_estimate_(true), matchable_(true),
  timestamp_(point_cloud->header.stamp),
  pc_col(point_cloud),
  initial_node_matches_(0)
{
 
  //ROS_INFO_STREAM("Construction of Node with " << ps->get<std::string>("feature_detector_type") << " Features");
  //ScopedTimer s("Node Constructor");

	base_frame_name = "base_frame";
	base2points_ = tf::StampedTransform(tf::Transform::getIdentity(), pcl_conversions::fromPCL(point_cloud->header).stamp,base_frame_name, point_cloud->header.frame_id);

	feature_extractor_type_ = "ORB";
	matcher_type_ = "BRUTEFORCE_OPENCV";
	min_matches_ = 20;
	max_dist_for_inliers_ = 0.5;
	ransac_iterations_ = 100;
	
  cv::Mat gray_img; 
  if(visual.type() == CV_8UC3){ cv::cvtColor(visual, gray_img, CV_RGB2GRAY); } 
  else { gray_img = visual; }
	m_img = gray_img;

	//2D keypoint detection
    // s("Feature Detection");
    //ROS_FATAL_COND(detector.empty(), "No valid detector!");
    detector->detect( gray_img, feature_locations_2d_, detection_mask);// fill 2d locations


 	//3D keypoint detection
    projectTo3D(feature_locations_2d_, feature_locations_3d_, pc_col); //takes less than 0.01 sec
    // projectTo3d need a dense cloud to use the points.at(px.x,px.y)-Call
    //ScopedTimer s("Feature Extraction");
	//2D keypoint extraction
    extractor->compute(gray_img, feature_locations_2d_, feature_descriptors_); //fill feature_descriptors_ with information 
 

 
	//check to see that everything is alright
    assert(feature_locations_2d_.size() == feature_locations_3d_.size());
    assert(feature_locations_3d_.size() == (unsigned int)feature_descriptors_.rows); 
    //feature_matching_stats_.resize(feature_locations_2d_.size(), 0);
    //ROS_INFO_NAMED("statistics", "Feature Count of Node:\t%d", (int)feature_locations_2d_.size());
    /* Now inside the projection method:
    size_t max_keyp = ps->get<int>("max_keypoints");
    if(feature_locations_2d_.size() > max_keyp) {
      feature_locations_2d_.resize(max_keyp);
      feature_locations_3d_.resize(max_keyp);
      feature_descriptors_ = feature_descriptors_.rowRange(0,max_keyp);
    }
    */
    assert(feature_locations_2d_.size() == feature_locations_3d_.size());
    assert(feature_locations_3d_.size() == (unsigned int)feature_descriptors_.rows); 


/**
  if((!ps->get<bool>("use_glwidget") ||
      !ps->get<bool>("use_gui")) &&
     !ps->get<bool>("store_pointclouds") &&
     !ps->get<int>("emm__skip_step"))
  {
    ROS_WARN("Clearing out points");
    this->clearPointCloud();
  } else if(ps->get<double>("voxelfilter_size") > 0.0) {
    //Let it be voxelfiltered with the call to clearPointCloud
    //*
    double vfs = ps->get<double>("voxelfilter_size");
    pcl::VoxelGrid<point_type> sor;
    sor.setLeafSize(vfs,vfs,vfs);
    pointcloud_type::ConstPtr const_cloud_ptr = boost::make_shared<pointcloud_type> (*pc_col);                                                                 
    sor.setInputCloud (const_cloud_ptr);
    sor.filter (*pc_col);
    //
**/
 
}

Node::Node(const cv::Mat visual,
           cv::Ptr<cv::FeatureDetector> detector,
           cv::Ptr<cv::DescriptorExtractor> extractor,
           pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
           const cv::Mat detection_mask) : id_(-1), seq_id_(-1), vertex_id_(-1), valid_tf_estimate_(true), matchable_(true),
  timestamp_(point_cloud->header.stamp),
  pc_col(point_cloud),
  initial_node_matches_(0)
{
  //cv::namedWindow("matches");
  //ParameterServer* ps = ParameterServer::instance();

  //ROS_INFO_STREAM("Construction of Node with " << ps->get<std::string>("feature_detector_type") << " Features");
  //ScopedTimer s("Node Constructor");
	base_frame_name = "base_frame";
	base2points_ = tf::StampedTransform(tf::Transform::getIdentity(), pcl_conversions::fromPCL(point_cloud->header).stamp,base_frame_name, point_cloud->header.frame_id);

	
	feature_extractor_type_ = "FREAK";
	matcher_type_ = "BRUTEFORCE_OPENCV";
	min_matches_ = 20;
	max_dist_for_inliers_ = 3.00;
	ransac_iterations_ = 100;
	
  cv::Mat gray_img; 
  if(visual.type() == CV_8UC3){ cvtColor(visual, gray_img, CV_RGB2GRAY); } 
  else { gray_img = visual; }
	m_img = gray_img;



  if(true)//feature_detector_type_ != "GICP")
  {
    //ScopedTimer s("Feature Detection");
    //ROS_FATAL_COND(detector.empty(), "No valid detector!");
    detector->detect( gray_img, feature_locations_2d_, detection_mask);// fill 2d locations
  }

  if(true )//feature_detector_type_ != "GICP")
  {
    projectTo3D(feature_locations_2d_, feature_locations_3d_, pc_col); //takes less than 0.01 sec
    // projectTo3d need a dense cloud to use the points.at(px.x,px.y)-Call
    //ScopedTimer s("Feature Extraction");
		//std::cout<< "Feature 2d size before extraction: "<< feature_locations_2d_.size()<<" , 3D: " << feature_locations_3d_.size() <<std::endl;
    extractor->compute(gray_img, feature_locations_2d_, feature_descriptors_); //fill feature_descriptors_ with information 
    projectTo3D(feature_locations_2d_, feature_locations_3d_, pc_col); //takes less than 0.01 sec
		std::cout<< "Feature 2d size after extraction: "<< feature_locations_2d_.size()<<" , 3D: " << feature_locations_3d_.size() <<std::endl;
  }

  if(true )//feature_detector_type_ != "GICP")
  {
    assert(feature_locations_2d_.size() == feature_locations_3d_.size());
    assert(feature_locations_3d_.size() == (unsigned int)feature_descriptors_.rows); 
    //feature_matching_stats_.resize(feature_locations_2d_.size(), 0);
    //ROS_INFO_NAMED("statistics", "Feature Count of Node:\t%d", (int)feature_locations_2d_.size());
    /* Now inside the projection method:
    size_t max_keyp = ps->get<int>("max_keypoints");
    if(feature_locations_2d_.size() > max_keyp) {
      feature_locations_2d_.resize(max_keyp);
      feature_locations_3d_.resize(max_keyp);
      feature_descriptors_ = feature_descriptors_.rowRange(0,max_keyp);
    }
    */
    assert(feature_locations_2d_.size() == feature_locations_3d_.size());
    assert(feature_locations_3d_.size() == (unsigned int)feature_descriptors_.rows); 
  }
/**
  if(ps->get<bool>("use_root_sift") &&
     (ps->get<std::string>("feature_extractor_type") == "SIFTGPU" ||
      ps->get<std::string>("feature_extractor_type") == "SURF" ||
      ps->get<std::string>("feature_extractor_type") == "GFTT" ||
      ps->get<std::string>("feature_extractor_type") == "SIFT")){
    squareroot_descriptor_space(feature_descriptors_);
  }
**/
}

Node::~Node(){
}


void Node::projectTo3D(std::vector<cv::KeyPoint>& feature_locations_2d,std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& feature_locations_3d, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr point_cloud)
{
  //ScopedTimer s(__FUNCTION__);

  size_t max_keyp =600; //ParameterServer::instance()->get<int>("max_keypoints");
  double maximum_depth = 1000000000000;//ParameterServer::instance()->get<double>("maximum_depth");
  cv::Point2f p2d;

  if(feature_locations_3d.size()){
    //ROS_INFO("There is already 3D Information in the FrameInfo, clearing it");
    feature_locations_3d.clear();
  }

  for(unsigned int i = 0; i < feature_locations_2d.size(); /*increment at end of loop*/){
    p2d = feature_locations_2d[i].pt;
    if (p2d.x >= point_cloud->width || p2d.x < 0 ||
        p2d.y >= point_cloud->height || p2d.y < 0 ||
        std::isnan(p2d.x) || std::isnan(p2d.y)){ //TODO: Unclear why points should be outside the image or be NaN
      //ROS_WARN_STREAM("Ignoring invalid keypoint: " << p2d); //Does it happen at all? If not, remove this code block
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }

    pcl::PointXYZRGB p3d = point_cloud->at((int) p2d.x,(int) p2d.y);

    // Check for invalid measurements
    if ( (p3d.z > maximum_depth) || isnan(p3d.x) || isnan(p3d.y) || isnan(p3d.z))
    {
      //ROS_DEBUG_NAMED(__FILE__, "Feature %d has been extracted at NaN depth. Omitting", i);
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }

    feature_locations_3d.push_back(Eigen::Vector4f(p3d.x, p3d.y, p3d.z, 1.0));
    i++; //Only increment if no element is removed from vector
    if(feature_locations_3d.size() >= max_keyp) break;
  }
	//std::cout<< point_cloud->isOrganized() <<std::endl;
	std::cout<< "Feature 2d size before: "<< feature_locations_2d.size()<<" , 3D: " << feature_locations_3d.size() <<std::endl;
  //ROS_INFO("Feature 2d size: %zu, 3D: %zu", feature_locations_2d.size(), feature_locations_3d.size());
  feature_locations_2d.resize(feature_locations_3d.size());
  //ROS_INFO("Feature 2d size: %zu, 3D: %zu", feature_locations_2d.size(), feature_locations_3d.size());
	std::cout<< "Feature 2d size after: "<< feature_locations_2d.size()<<" , 3D: " << feature_locations_3d.size() <<std::endl;
}

MatchingResult Node::matchNodePair(const Node* older_node)
{
  MatchingResult mr;
  try{
    bool found_transformation = false;
/**
    if(ParameterServer::instance()->get<int>("max_connections") > 0 &&
       initial_node_matches_ > ParameterServer::instance()->get<int>("max_connections")) 
      return mr; //enough is enough
**/
    const unsigned int min_matches = (unsigned int) min_matches_;// minimal number of feature correspondences to be a valid candidate for a link

    this->featureMatching(older_node, &mr.all_matches); 

    double ransac_quality = 0;
   // ROS_DEBUG_NAMED(__FILE__, "found %i inital matches",(int) mr.all_matches.size());
    if (mr.all_matches.size() < min_matches){
        //ROS_INFO("Too few inliers between %i and %i for RANSAC method. Only %i correspondences to begin with.",
						//older_node->id_,this->id_,(int)mr.all_matches.size());
				std::cout<< "Too few inliers between " <<  older_node->id_ <<  " and " << this->id_ << " for RANSAC method. Only " << (int)mr.all_matches.size() << " correspondences to begin with." <<std::endl;
                 
    } 
    else {//All good for feature based transformation estimation
        found_transformation = getRelativeTransformationTo(older_node,&mr.all_matches, mr.ransac_trafo, mr.rmse, mr.inlier_matches);//getRelativeTransformationPCL(older_node,&mr.all_matches, mr.ransac_trafo, mr.rmse, mr.inlier_matches); 
        //Statistics  
        float nn_ratio = 0.0;
        if(found_transformation){
          //double w = 1.0 + (double)mr.inlier_matches.size()-(double)min_matches;///(double)mr.all_matches.size();
          for(unsigned int i = 0; i < mr.inlier_matches.size(); i++){
            nn_ratio += mr.inlier_matches[i].distance;
          }
          nn_ratio /= mr.inlier_matches.size();
          //ParameterServer::instance()->set("nn_distance_ratio", static_cast<double>(nn_ratio + 0.2));
          mr.final_trafo = mr.ransac_trafo;
          mr.edge.informationMatrix =   Eigen::Matrix<double,6,6>::Identity()*(mr.inlier_matches.size()/(mr.rmse*mr.rmse)); //TODO: What do we do about the information matrix? Scale with inlier_count. Should rmse be integrated?)

          mr.edge.id1 = older_node->id_;//and we have a valid transformation
          mr.edge.id2 = this->id_; //since there are enough matching features,
          mr.edge.transform = mr.final_trafo.cast<double>();//we insert an edge between the frames
          
          found_transformation = true;
         

         // ROS_INFO("RANSAC found a %s transformation with %d inliers matches with average ratio %f", found_transformation? "valid" : "invalid", (int) mr.inlier_matches.size(), nn_ratio);

        } else {
          for(unsigned int i = 0; i < mr.all_matches.size(); i++){
            nn_ratio += mr.all_matches[i].distance;
          }
          nn_ratio /= mr.all_matches.size();
          ROS_INFO("RANSAC found no valid trafo, but had initially %d feature matches with average ratio %f",(int) mr.all_matches.size(), nn_ratio);

/**
#ifdef USE_PCL_ICP
          if(((int)this->id_ - (int)older_node->id_) <= 1){ //Apply icp only for adjacent frames, as the initial guess needs to be in the global minimum
            mr.icp_trafo = Eigen::Matrix4f::Identity();
            int max_count = ParameterServer::instance()->get<int>("gicp_max_cloud_size");
            pointcloud_type::Ptr tmp1(new pointcloud_type());
            pointcloud_type::Ptr tmp2(new pointcloud_type());
            filterCloud(*pc_col, *tmp1, max_count);
            filterCloud(*(older_node->pc_col), *tmp2, max_count);
            mr.icp_trafo = icpAlignment(tmp2, tmp1, mr.icp_trafo);
            //pairwiseObservationLikelihood(this, older_node, mr);
            //double icp_quality;
            found_transformation = true; //observation_criterion_met(mr.inlier_points, mr.outlier_points, mr.occluded_points + mr.inlier_points + mr.outlier_points, icp_quality);
            if(found_transformation){
              ROS_ERROR("Found trafo via icp");
              mr.final_trafo = mr.icp_trafo;
              mr.edge.id1 = older_node->id_;//and we have a valid transformation
              mr.edge.id2 = this->id_; //since there are enough matching features,
              mr.edge.transform = mr.final_trafo.cast<double>();//we insert an edge between the frames
              std::cout << std::endl << mr.final_trafo << std::endl;
            }
          }
#endif
**/
        }
    } 

 

    if(found_transformation) {
        //ROS_INFO("Returning Valid Edge");
        //++initial_node_matches_; //trafo is accepted
    }
    else {
        mr.edge.id1 = -1;
        mr.edge.id2 = -1;
    }
  }
  catch (std::exception e){
    //ROS_ERROR("Caught Exception in comparison of Nodes %i and %i: %s", this->id_, older_node->id_, e.what());
		std::cout<<"Caught Exception in comparison of Nodes "<< this->id_ << " and " << older_node->id_ << e.what() <<std::endl; 
		
  }

  return mr;
}

void Node::setBase2PointsTransform(tf::StampedTransform& b2p){
    base2points_ = b2p;
}

	
tf::StampedTransform Node::getBase2PointsTransform() const {
    return base2points_;
}


//TODO: This function seems to be resistant to parallelization probably due to knnSearch
unsigned int Node::featureMatching(const Node* other, std::vector<cv::DMatch>* matches) const 
{
  //ScopedTimer s(__FUNCTION__);
  assert(matches->size()==0);
  // number of neighbours found (two, to compare the best matches for distinctness
  const int k = 2;
  //unsigned int one_nearest_neighbour = 0, two_nearest_neighbours = 0;

  // number of neighbors found (has to be two, see l. 57)
  double sum_distances = 0.0;
  //ParameterServer* ps = ParameterServer::instance();
  //const int min_kp = ps->get<int> ("min_keypoints");

  //using siftgpu, if available and wanted
	/**
  if(ps->get<std::string>("feature_detector_type") == "GICP"){
    return 0;
  }

#ifdef USE_SIFT_GPU
  if (ps->get<std::string> ("matcher_type") == "SIFTGPU") {
    siftgpu_mutex.lock();
    sum_distances = SiftGPUWrapper::getInstance()->match(siftgpu_descriptors, feature_descriptors_.rows, other->siftgpu_descriptors, other->feature_descriptors_.rows, matches);
    siftgpu_mutex.unlock();
  }
  else
#endif
	**/
  //using BruteForceMatcher for ORB features
  if (matcher_type_ != "BRUTEFORCE_OPENCV" && feature_extractor_type_ == "ORB")
  {
    //ROS_INFO_COND(ps->get<std::string>("matcher_type") == "FLANN", "You specified FLANN with ORB, this is SLOW! Using BRUTEFORCE instead.");
    cv::Ptr<cv::DescriptorMatcher> matcher;
    if(feature_extractor_type_ == "ORB"){
        //ScopedTimer s("My bruteforce Search", false, true);
        uint64_t* query_value =  reinterpret_cast<uint64_t*>(this->feature_descriptors_.data);
        uint64_t* search_array = reinterpret_cast<uint64_t*>(other->feature_descriptors_.data);
        for(unsigned int i = 0; i < this->feature_locations_2d_.size(); ++i, query_value += 4){//ORB feature = 32*8bit = 4*64bit
          int result_index = -1;
          int hd = bruteForceSearchORB(query_value, search_array, other->feature_locations_2d_.size(), result_index);
          if(hd >= 128) continue;//not more than half of the bits matching: Random
          cv::DMatch match(i, result_index, hd /256.0 + (float)rand()/(1000.0*RAND_MAX));
          matches->push_back(match);
        }
    }
    //else//any bruteforce
    if (matcher_type_ == "BRUTEFORCE_OPENCV" && feature_extractor_type_ == "ORB")
    {
      //ScopedTimer s("OpenCV bruteforce Search", false, true);
      std::string brute_force_type("BruteForce-HammingLUT"); //L2 per default
      matcher = cv::DescriptorMatcher::create(brute_force_type);
      std::vector< std::vector<cv::DMatch> > bruteForceMatches;
      matcher->knnMatch(feature_descriptors_, other->feature_descriptors_, bruteForceMatches, k);
      double max_dist_ratio_fac = 0.9;
      //if ((int)bruteForceMatches.size() < min_kp) max_dist_ratio_fac = 1.0; //if necessary use possibly bad descriptors
      srand((long)std::clock());
      std::set<int> train_indices;
      matches->clear();
      matches->reserve(bruteForceMatches.size());
      for(unsigned int i = 0; i < bruteForceMatches.size(); i++) {
          cv::DMatch m1 = bruteForceMatches[i][0];
          cv::DMatch m2 = bruteForceMatches[i][1];
          float dist_ratio_fac = m1.distance / m2.distance;
          if (dist_ratio_fac < max_dist_ratio_fac) {//this check seems crucial to matching quality
              int train_idx = m1.trainIdx;
              if(train_indices.count(train_idx) > 0)
                continue; //FIXME: Keep better
                
              train_indices.insert(train_idx);
              sum_distances += m1.distance;
              m1.distance = dist_ratio_fac + (float)rand()/(1000.0*RAND_MAX); //add a small random offset to the distance, since later the dmatches are inserted to a set, which omits duplicates and the duplicates are found via the less-than function, which works on the distance. Therefore we need to avoid equal distances, which happens very often for ORB
              matches->push_back(m1);
          } 

      }
    }
    //matcher->match(feature_descriptors_, other->feature_descriptors_, *matches);
  } 

	  else if (feature_extractor_type_ == "FREAK")
  {
		
		cv::BruteForceMatcher<cv::Hamming> matcher;
   
    //ScopedTimer s("My FREAK search", false, true);
		matcher.match(feature_descriptors_, other->feature_descriptors_, *matches);
		//std::cout << "match size :" << (*matches).size() << std::endl; *************************************

		//cv::Mat imgMatch;
    //cv::drawMatches(this->m_img, this->feature_locations_2d_, other->m_img, other->feature_locations_2d_, *matches, imgMatch);

    //cv::namedWindow("matches", CV_WINDOW_KEEPRATIO);
    //cv::imshow("matches", imgMatch);
    //cv::waitKey(0);
 
  }
	/**
  else if (ps->get<std::string>("matcher_type") == "FLANN") // && ps->get<std::string>("feature_extractor_type") != "ORB")
  {
    ScopedTimer s("OpenCV FLANN Search", false, true);
    int start_feature = 0; //feature_descriptors_.rows - std::min(2*ps->get<int>("max_matches"), feature_descriptors_.rows);//FIXME: search only for best keypoints (in SIFTGPU they are at the back)
    int num_features = feature_descriptors_.rows;                               //compute features per chunk
    { //old block of (removed) search for matches chunkwise
      // compare
      // http://opencv-cocoa.googlecode.com/svn/trunk/samples/c/find_obj.cpp
      cv::Mat indices(num_features-start_feature, k, CV_32S);
      cv::Mat dists(num_features-start_feature, k, CV_32F);
      cv::Mat relevantDescriptors = feature_descriptors_.rowRange(start_feature, num_features);

      // get the best two neighbors
      {
        ScopedTimer s("FLANN KNN-search");
        /* 64: The number of times the tree(s) in the index should be
         * recursively traversed. A higher value for this parameter would give
         * better search precision, but also take more time. If automatic
         * configuration was used when the index was created, the number of
         * checks required to achieve the specified precision was also
         * computed, in which case this parameter is ignored.
         */
/**
        other->knnSearch(relevantDescriptors, indices, dists, k, cv::flann::SearchParams(16));
      }

      cv::DMatch match;
      double avg_ratio = 0.0;
      double max_dist_ratio_fac = ps->get<double>("nn_distance_ratio");
      std::set<int> train_indices;
      matches->clear();
      matches->reserve(indices.rows);
      for(int i = 0; i < indices.rows; ++i) {
        float dist_ratio_fac =  dists.at<float>(2*i) / dists.at<float>(2*i + 1);
        avg_ratio += dist_ratio_fac;
        //if (indices.rows < min_kp) dist_ratio_fac = 1.0; //if necessary use possibly bad descriptors
        if (max_dist_ratio_fac > dist_ratio_fac) {
          int train_idx = indices.at<int>(2 * i);
          if(train_indices.count(train_idx) > 0)
            continue; //FIXME: Keep better
            
          train_indices.insert(train_idx);
          match.queryIdx = i+start_feature;
          match.trainIdx = train_idx;
          match.distance = dist_ratio_fac; //dists_ptr[2 * i];
          sum_distances += match.distance;

          assert(match.trainIdx < other->feature_descriptors_.rows);
          assert(match.queryIdx < feature_descriptors_.rows);
          matches->push_back(match);
        }
      }
      
      ROS_INFO("Feature Matches between Nodes %3d (%4d features) and %3d (%4d features) (features %d to %d of first node):\t%4d. Percentage: %f%%, Avg NN Ratio: %f",
                this->id_, (int)this->feature_locations_2d_.size(), other->id_, (int)other->feature_locations_2d_.size(), start_feature, num_features, 
                (int)matches->size(), (100.0*matches->size())/((float)num_features-start_feature), avg_ratio / (num_features-start_feature));

    }//for
  }
  else {
      ROS_FATAL_STREAM("Cannot match features:\nNo valid combination for " <<
                       "matcher_type ("           << ps->get<std::string>("matcher_type") << ") and " <<
                       "feature_extractor_type (" << ps->get<std::string>("feature_extractor_type") << ") chosen.");
  }

  keepStrongestMatches(ps->get<int>("max_matches"), matches);

  ROS_DEBUG_NAMED("statistics", "count_matrix(%3d, %3d) =  %4d;",
                  this->id_+1, other->id_+1, (int)matches->size());
  ROS_DEBUG_NAMED("statistics", "dista_matrix(%3d, %3d) =  %f;",
                 this->id_+1, other->id_+1, sum_distances/ (float)matches->size());
  ROS_DEBUG_NAMED("statistics", "Feature Matches between Nodes %3d (%4d features) and %3d (%4d features):\t%4d",
                  this->id_, (int)this->feature_locations_2d_.size(),
                  other->id_, (int)other->feature_locations_2d_.size(),
                  (int)matches->size());


	**/

  //ROS_INFO("matches size: %i, rows: %i", (int) matches->size(), feature_descriptors_.rows);

  //assert(one_nearest_neighbour+two_nearest_neighbours > 0);
  //return static_cast<float>(one_nearest_neighbour) / static_cast<float>(one_nearest_neighbour+two_nearest_neighbours);
  return matches->size();
}

///Find transformation with largest support, RANSAC style.
///Return false if no transformation can be found
bool Node::getRelativeTransformationTo(const Node* earlier_node,
                                       std::vector<cv::DMatch>* initial_matches,
                                       Eigen::Matrix4f& resulting_transformation,
                                       float& rmse, 
                                       std::vector<cv::DMatch>& matches) const
{
  //ScopedTimer s(__FUNCTION__);
  //static const bool allow_features_without_depth = ParameterServer::instance()->get<bool>("allow_features_without_depth");
  //VALIDATION
  assert(initial_matches != NULL);
  
  //std::stringstream nodesstringstream; nodesstringstream << "Nodes " << (this->id_) << "<->" << (earlier_node->id_);
  //std::string nodesstring = nodesstringstream.str();
  if(initial_matches->size() <= min_matches_){
    //ROS_INFO("%s: Only %d feature matches between %d and %d (minimal: %i)",nodesstring.c_str(), (int)initial_matches->size() , this->id_, earlier_node->id_, ParameterServer::instance()->get<int>("min_matches"));
		std::cout << "Too few matches, Only " << (int)initial_matches->size() <<  " feature matches between " <<  this->id_ <<" and "  << earlier_node->id_ <<std::endl;
    return false;
  }

  //PREPARATION
  unsigned int min_inlier_threshold = int(initial_matches->size()*0.75);
  unsigned int min_inlier_thrzeshold = (unsigned int) min_matches_;
  if(min_inlier_threshold > 0.75 * initial_matches->size()){
    //FIXME: Evaluate whether beneficial
    //ROS_INFO("Lowering min_inlier_threshold from %d to %d, because there are only %d matches to begin with", min_inlier_threshold, (int) (0.75 * initial_matches->size()), (int)initial_matches->size());
		std::cout <<"Lowering min_inlier_threshold " << std::endl;
    min_inlier_threshold = 0.75 * initial_matches->size();
  }

  double inlier_error; //all squared errors
  srand((long)std::clock());
  
  // a point is an inlier if it's no more than max_dist_m m from its partner apart
  const float max_dist_m = max_dist_for_inliers_;
  const int ransac_iterations = ransac_iterations_;
  //std::vector<double> dummy;

  // initialize result values of all iterations 
  matches.clear();
  resulting_transformation = Eigen::Matrix4f::Identity();
  rmse = 1e6;
  unsigned int valid_iterations = 0;//, best_inlier_cnt = 0;
  const unsigned int sample_size = 4;// chose this many randomly from the correspondences:
  bool valid_tf = false; // valid is false iff the sampled points clearly aren't inliers themself 

  std::vector<cv::DMatch>* matches_with_depth = initial_matches;
  
  std::sort(matches_with_depth->begin(), matches_with_depth->end()); //sort by distance, which is the nn_ratio

  int real_iterations = 0;
  for(int n = 0; (n < ransac_iterations && matches_with_depth->size() >= sample_size); n++) //Without the minimum number of matches, the transformation can not be computed as usual TODO: implement monocular motion est
  {
    //Initialize Results of refinement
    double refined_error = 1e6;
    std::vector<cv::DMatch> refined_matches; 
    std::vector<cv::DMatch> inlier = sample_matches_prefer_by_distance(sample_size, *matches_with_depth); //initialization with random samples 
    //std::vector<cv::DMatch> inlier = sample_matches(sample_size, *matches_with_depth); //initialization with random samples 
    Eigen::Matrix4f refined_transformation = Eigen::Matrix4f::Identity();

    real_iterations++;
    for(int refinements = 1; refinements < 20 /*got stuck?*/; refinements++) 
    {
        Eigen::Matrix4f transformation = getTransformFromMatches(this, earlier_node, inlier,valid_tf,max_dist_m);
        //Eigen::Matrix4f transformation = getTransformFromMatchesUmeyama(this, earlier_node, inlier,valid_tf);
        if (!valid_tf || transformation!=transformation)  //Trafo Contains NaN?
          break; // valid_tf is false iff the sampled points aren't inliers themself 

        //test which features are inliers 
        computeInliersAndError(*initial_matches, transformation, 
                               this->feature_locations_3d_, //this->feature_depth_stats_, 
                               earlier_node->feature_locations_3d_, //earlier_node->feature_depth_stats_, 
                               std::max(min_inlier_threshold, static_cast<unsigned int>(refined_matches.size())), //break if no chance to reach this amount of inliers
                               inlier, inlier_error, max_dist_m*max_dist_m); 
        
        if(inlier.size() < min_inlier_threshold || inlier_error > max_dist_m){
          //ROS_DEBUG("Skipped iteration: inliers: %i (min %i), inlier_error: %.2f (max %.2f)", (int)inlier.size(), (int) min_inlier_threshold,  inlier_error*100, max_dist_m*100);
          break; //hopeless case
        }

        //superior to before?
        if (inlier.size() >= refined_matches.size() && inlier_error <= refined_error) {
          size_t prev_num_inliers = refined_matches.size();
          assert(inlier_error>=0);
          refined_transformation = transformation;
          refined_matches = inlier;
          refined_error = inlier_error;
          if(inlier.size() == prev_num_inliers) break; //only error improved -> no change would happen next iteration
        }
        else break;
    }  //END REFINEMENTS
    //Successful Iteration?
    if(refined_matches.size() > 0){ //Valid?
        valid_iterations++;
       // ROS_DEBUG("Valid iteration: inliers/matches: %lu/%lu (min %u), refined error: %.2f (max %.2f), global error: %.2f", 
                //refined_matches.size(), matches.size(), min_inlier_threshold,  refined_error, max_dist_m, rmse);

        //Acceptable && superior to previous iterations?
        if (refined_error <= rmse &&  
            refined_matches.size() >= matches.size() && 
            refined_matches.size() >= min_inlier_threshold)
        {
          //ROS_DEBUG("%s: Improvment in iteration %d: inliers: %i (min %i), inlier_error: %.2f (max %.2f)",nodesstring.c_str(), real_iterations, (int)refined_matches.size(), (int) min_inlier_threshold,  refined_error, max_dist_m);
          rmse = refined_error;
          resulting_transformation = refined_transformation;
          matches.assign(refined_matches.begin(), refined_matches.end());
          //Performance hacks:
          if (refined_matches.size() > initial_matches->size()*0.5) n+=10;///Iterations with more than half of the initial_matches inlying, count tenfold
          if (refined_matches.size() > initial_matches->size()*0.75) n+=10;///Iterations with more than 3/4 of the initial_matches inlying, count twentyfold
          if (refined_matches.size() > initial_matches->size()*0.8) break; ///Can this get better anyhow?
        }
    }
  } //iterations
  if(valid_iterations == 0) // maybe no depth. Try identity?
  { 
    //IDENTITYTEST
    //ROS_INFO("Last Resort: Trying identity as hypothesis");
		std::cout << "Last Resort: Trying identity as hypothesis" <<std::endl;
    //1 ransac iteration with identity
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();//hypothesis
    std::vector<cv::DMatch> inlier; //result
    //test which samples are inliers 
    computeInliersAndError(*initial_matches, transformation, 
                           this->feature_locations_3d_, //this->feature_depth_stats_, 
                           earlier_node->feature_locations_3d_, //earlier_node->feature_depth_stats_, 
                           min_inlier_threshold, //break if no chance to reach this amount of inliers
                           inlier, inlier_error, max_dist_m*max_dist_m); 
    
    //superior to before?
    if (inlier.size() > min_inlier_threshold && inlier_error < max_dist_m) {
      assert(inlier_error>=0);
      resulting_transformation = transformation;
      matches.assign(inlier.begin(), inlier.end());
      rmse = inlier_error;
      valid_iterations++;
      //ROS_INFO("No-Motion guess for %i<->%i: inliers: %i (min %i), inlier_error: %.2f (max %.2f)", this->id_, earlier_node->id_, (int)matches.size(), (int) min_inlier_threshold,  rmse, max_dist_m);
    }
  } //END IDENTITY AS GUESS
  //ROS_INFO("%i good iterations (from %i), inlier pct %i, inlier cnt: %i, error (MHD): %.2f",valid_iterations, ransac_iterations, (int) (matches.size()*1.0/initial_matches->size()*100),(int) matches.size(),rmse);
  
  //ROS_INFO_STREAM("Transformation estimated:\n" << resulting_transformation);
  



  //G2O Refinement (minimize mahalanobis distance, include depthless features in optimization)
  //Optimize transform based on latest inliers (in "matches") and initial guess (in "resulting_transformation")
	/**
  const int g2o_iterations = ParameterServer::instance()->get<int>( "g2o_transformation_refinement");
  if(g2o_iterations > 0 && matches.size() > min_inlier_threshold)//Do not do this if RANSAC didn't succeed. Can't handle outliers
  {
    Eigen::Matrix4f transformation = resulting_transformation;//current hypothesis
    getTransformFromMatchesG2O(earlier_node, this,matches, transformation, g2o_iterations);
    std::vector<cv::DMatch> inlier; //result

    //Evaluate the new transformation
    computeInliersAndError(*initial_matches, transformation, 
                           this->feature_locations_3d_, //this->feature_depth_stats_, 
                           earlier_node->feature_locations_3d_, //earlier_node->feature_depth_stats_, 
                           matches.size(), //minimum to reach
                           inlier,inlier_error, //Output!
                           max_dist_m*max_dist_m); 
    ROS_INFO_STREAM(nodesstring << ": g2o transformation estimated to Node " << earlier_node->id_ << ":\n" << transformation);
    //superior in inliers or equal inliers and better rmse?
    if (inlier.size() >= matches.size() || inlier.size() >= min_inlier_threshold && inlier_error < rmse) {
      //if More inliers -> Refine with them included
      if (inlier.size() > matches.size()) {
        //Refine using the new inliers
        getTransformFromMatchesG2O(earlier_node, this,inlier, transformation, g2o_iterations);
        computeInliersAndError(*initial_matches, transformation, 
                               this->feature_locations_3d_, //this->feature_depth_stats_, 
                               earlier_node->feature_locations_3d_, //earlier_node->feature_depth_stats_, 
                               inlier.size(), //minimum to reach
                               inlier,inlier_error, max_dist_m*max_dist_m); 
      }
      ROS_INFO("G2o optimization result for %i<->%i: inliers: %i (min %i), inlier_error: %.2f (max %.2f)", this->id_, earlier_node->id_, (int)inlier.size(), (int) min_inlier_threshold,  inlier_error, max_dist_m);
      //Again superier? Then use the new result
      if (inlier.size() >= matches.size()) 
      {
        ROS_INFO_STREAM("Refined transformation estimate" << earlier_node->id_ << ":\n" << transformation);
        assert(inlier_error>=0);
        resulting_transformation = transformation;
        matches.assign(inlier.begin(), inlier.end());
        rmse = inlier_error;
        valid_iterations++;
        ROS_INFO("G2o refinement optimization result for %i<->%i: inliers: %i (min %i), inlier_error: %.2f (max %.2f)", this->id_, earlier_node->id_, (int)matches.size(), (int) min_inlier_threshold,  rmse, max_dist_m);
      }
    }
    else {
      ROS_INFO("G2O optimization of RANSAC for %s rejected: G2O inliers: %i (min %i), inlier_error: %.2f (max %.2f) RANSAC inls: %i (min %i), inlier_error: %.2f", nodesstring.c_str(), (int)inlier.size(), (int) min_inlier_threshold,  inlier_error, max_dist_m, (int)matches.size(), (int) min_inlier_threshold,  rmse);
    }
  }
  
**/


 // ROS_INFO("%s: %i good iterations (from %i), inlier pct %i, inlier cnt: %i, error (MHD): %.2f",nodesstring.c_str(), valid_iterations, ransac_iterations, (int) (matches.size()*1.0/initial_matches->size()*100),(int) matches.size(),rmse);
  // ROS_INFO("best overall: inlier: %i, error: %.2f",best_inlier_invalid, best_error_invalid*100);

  bool enough_absolute = matches.size() >= min_inlier_threshold;
  return enough_absolute;
}




void Node::computeInliersAndError(const std::vector<cv::DMatch> & all_matches, const Eigen::Matrix4f& transformation4f,
                                  const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& origins,
                                  const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& earlier,
                                  size_t min_inliers, //break if this can't be reached
                                  std::vector<cv::DMatch>& inliers, //pure output var
                                  double& return_mean_error,//pure output var: rms-mahalanobis-distance
                                  //std::vector<double>& errors,
                                  double squaredMaxInlierDistInM) const
{ 
  inliers.clear();
  assert(all_matches.size() > 0);
  inliers.reserve(all_matches.size());
  //errors.clear();
  const size_t all_matches_size = all_matches.size();
  double mean_error = 0.0;
  Eigen::Matrix4d transformation4d = transformation4f.cast<double>();

//parallelization is detrimental here
//#pragma omp parallel for reduction (+: mean_error)
  for(int i=0; i < all_matches_size; ++i)
  //BOOST_FOREACH(const cv::DMatch& m, all_matches)
  {
    const cv::DMatch& m = all_matches[i];
    const Eigen::Vector4f& origin = origins[m.queryIdx];
    const Eigen::Vector4f& target = earlier[m.trainIdx];
		//std::cout << "origin :" << origin << std::endl;
		//std::cout << "target :" << target << std::endl;
		/**
    if(origin(2) == 0.0 || target(2) == 0.0){ //does NOT trigger on NaN
			 std::cout<< "Ignoring match cause of NANs" <<std::endl;
       continue;
    }
		**/
    //double mahal_dist = errorFunction2(origin, target, transformation4d);
		Eigen::Vector4f m_temp = target-transformation4f*origin;//(target - transformation4d*origin);
		double mahal_dist = (double) m_temp.squaredNorm();
    if(mahal_dist > squaredMaxInlierDistInM){
		  //std::cout<< "Ignoring match cause of error too large" <<std::endl; *************************
      continue; //ignore outliers
			
    }
    if(!(mahal_dist >= 0.0)){
      //ROS_WARN_STREAM("Mahalanobis_ML_Error: "<<mahal_dist);

      //ROS_WARN_STREAM("Transformation for error !>= 0:\n" << transformation4d << "Matches: " << all_matches.size());
			std::cout << "Mahalanobis_ML_Error: " << mahal_dist <<std::endl;
      continue;
    }
    mean_error += mahal_dist;
//#pragma omp critical
    inliers.push_back(m); //include inlier
  }


  if (inliers.size()<3){ //at least the samples should be inliers
    //ROS_DEBUG("No inliers at all in %d matches!", (int)all_matches.size()); // only warn if this checks for all initial matches
		std::cout <<"No inliers at all in "<<(int)all_matches.size()  <<" matches "<<std::endl;
    return_mean_error = 1e9;
  } else {
    mean_error /= inliers.size();
    return_mean_error = sqrt(mean_error);
  }

}

bool Node::getRelativeTransformationPCL(const Node* earlier_node,
                                       std::vector<cv::DMatch>* initial_matches,
                                       Eigen::Matrix4f& resulting_transformation,
                                       float& rmse, 
                                       std::vector<cv::DMatch>& matches) const
{


  // Point clouds
  PointCloudT::Ptr object (new PointCloudT);
  PointCloudT::Ptr object_aligned (new PointCloudT);
  PointCloudT::Ptr scene (new PointCloudT);
  FeatureCloudT::Ptr object_features (new FeatureCloudT);
  FeatureCloudT::Ptr scene_features (new FeatureCloudT);
	pcl::copyPointCloud(*(this->pc_col),*object);
	pcl::copyPointCloud(*(earlier_node->pc_col),*scene);
	
  

  
  // Downsample
  //pcl::console::print_highlight ("Downsampling...\n");
  pcl::VoxelGrid<PointNT> grid;
  const float leaf = 0.05f;
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (object);
  grid.filter (*object);
  grid.setInputCloud (scene);
  grid.filter (*scene);
  
  // Estimate normals for scene
  //pcl::console::print_highlight ("Estimating scene normals...\n");
  pcl::NormalEstimationOMP<PointNT,PointNT> nest;
  nest.setRadiusSearch (0.01);
  nest.setInputCloud (scene);
  nest.compute (*scene);
  
  // Estimate features
  //pcl::console::print_highlight ("Estimating features...\n");
  FeatureEstimationT fest;
  fest.setRadiusSearch (0.025);
  fest.setInputCloud (object);
  fest.setInputNormals (object);
  fest.compute (*object_features);
  fest.setInputCloud (scene);
  fest.setInputNormals (scene);
  fest.compute (*scene_features);
  
  // Perform alignment
  pcl::console::print_highlight ("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
  align.setInputSource (object);
  align.setSourceFeatures (object_features);
  align.setInputTarget (scene);
  align.setTargetFeatures (scene_features);
  align.setMaximumIterations (10000); // Number of RANSAC iterations
  align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness (5); // Number of nearest features to use
  align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
  align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
  {
    //pcl::ScopeTime t("Alignment");
    align.align (*object_aligned);
  }

	std::vector<cv::DMatch>* matches_with_depth = initial_matches;
	double inlier_error;
    const unsigned int sample_size = 3;// chose this many randomly from the correspondences:
	std::vector<cv::DMatch> inlier = sample_matches_prefer_by_distance(sample_size, *matches_with_depth); 
	const float max_dist_m = max_dist_for_inliers_;
    unsigned int min_inlier_threshold = (unsigned int) min_matches_;

	if (align.hasConverged ())
  {
		pcl::console::print_highlight ("Alignment converged \n");
		resulting_transformation = align.getFinalTransformation();
		std::cout<< "resulting transformation : " << resulting_transformation << std::endl;
		/**
		computeInliersAndError(*initial_matches, resulting_transformation, 
                           this->feature_locations_3d_, //this->feature_depth_stats_, 
                           earlier_node->feature_locations_3d_, //earlier_node->feature_depth_stats_, 
                           min_inlier_threshold, //break if no chance to reach this amount of inliers
                           inlier, inlier_error, max_dist_m*max_dist_m); 
		**/
		
	
	}
	else
	{
		pcl::console::print_highlight ("Alignment did not converged \n");
		std::cout << "Last Resort: Trying identity as hypothesis" <<std::endl;
    //1 ransac iteration with identity
    resulting_transformation = Eigen::Matrix4f::Identity();//hypothesis
		

	}
	
	computeInliersAndError(*initial_matches, resulting_transformation, 
                           this->feature_locations_3d_, //this->feature_depth_stats_, 
                           earlier_node->feature_locations_3d_, //earlier_node->feature_depth_stats_, 
                           min_inlier_threshold, //break if no chance to reach this amount of inliers
                           inlier, inlier_error, max_dist_m*max_dist_m);


			assert(inlier_error>=0);
      matches.assign(inlier.begin(), inlier.end());
      rmse = inlier_error;
			std::cout<< "matches :" << matches.size() <<std::endl;
 
	return (matches.size() >= min_inlier_threshold);

}

