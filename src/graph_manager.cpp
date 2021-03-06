#include "my_slam/graph_manager.h"

#include <visualization_msgs/Marker.h>

#include "g2o/types/slam3d/se3quat.h"
#include "g2o/types/slam3d/edge_se3.h"

#include "g2o/core/optimizable_graph.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/core/optimization_algorithm_dogleg.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "my_slam/misc.h"

#include <QThread>
#include <qtconcurrentrun.h>
#include <QtConcurrentMap> 

#include <boost/foreach.hpp>
#include <std_msgs/Header.h>
#include <pcl/PCLHeader.h>
#include <pcl_conversions/pcl_conversions.h>

typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 3> >  SlamBlockSolver;
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearCSparseSolver;
typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearCholmodSolver;
typedef g2o::LinearSolverPCG<SlamBlockSolver::PoseMatrixType> SlamLinearPCGSolver;
typedef g2o::LinearSolverDense<SlamBlockSolver::PoseMatrixType> SlamLinearDenseSolver;

void updateInlierFeatures(const MatchingResult& mr, Node* new_node, Node* old_node)
{
/**
  BOOST_FOREACH(const cv::DMatch& match, mr.inlier_matches){
    assert(new_node->feature_matching_stats_.size() > match.queryIdx);
    assert(old_node->feature_matching_stats_.size() > match.trainIdx);
    unsigned char& new_flag = new_node->feature_matching_stats_[match.queryIdx];
    if(new_flag < 255) ++new_flag;
    unsigned char& old_flag = old_node->feature_matching_stats_[match.trainIdx];
    if(old_flag < 255) ++old_flag;

  }
**/
}


void fixationOfVertices(std::string strategy, g2o::SparseOptimizer* optimizer,std::map<int, Node* >& graph, g2o::HyperGraph::VertexSet& camera_vertices,int earliest_loop_closure_node)
{
    //Fixation strategies
    if (strategy == "previous" && graph.size() > 2) {
      optimizer->setFixed(camera_vertices, false);
      optimizer->vertex(graph[graph.size() - 2]->vertex_id_)->setFixed(true);
    }
    else if (strategy == "largest_loop"){
      //std::stringstream ss; ss << "Nodes in or outside loop: ";
      for (std::map<int, Node*>::iterator it=graph.begin(); it!=graph.end(); ++it){
        Node* mynode = it->second;
        //Even before oldest matched node?
        bool is_outside_largest_loop =  mynode->id_ < earliest_loop_closure_node;
        //ss << mynode->id_ << (is_outside_largest_loop ? "o, " : "i, ");
        optimizer->vertex(mynode->vertex_id_)->setFixed(is_outside_largest_loop);
      }
      //ROS_INFO("%s", ss.str().c_str());
    }
    else if (strategy == "first") {
      optimizer->setFixed(camera_vertices, false);
      optimizer->vertex(graph[0]->vertex_id_)->setFixed(true);
    }
}
/**
GraphManager::GraphManager() :optimizer_(NULL),
next_seq_id(0), next_vertex_id(0),
optimizer_skip_step_(1),min_matches_(20)
{

	backend = "pcg";
	pose_relative_to_ = "inaffected";



  createOptimizer();
	//optimizer_->initSolver(6, 10);
  ros::NodeHandle nh;
	 

  
  //marker_pub_ = nh.advertise<visualization_msgs::Marker>("/rgbdslam/pose_graph_markers",0);
  computed_motion_ = tf::Transform::getIdentity();
  init_base_pose_  = tf::Transform::getIdentity();
  //std::string fixed_frame = ParameterServer::instance()->get<std::string>("fixed_frame_name");
  //std::string base_frame  = ParameterServer::instance()->get<std::string>("base_frame_name");

  std::string fixed_frame = "fixed_frame_";
  std::string base_frame  ="base_frame_";
    
  latest_transform_cache_ = tf::StampedTransform(tf::Transform::getIdentity(), ros::Time::now(), base_frame, fixed_frame);
  //timer_ = nh.createTimer(ros::Duration(0.1), &GraphManager::broadcastLatestTransform, this);

	computed_motion_ = tf::Transform::getIdentity();
  init_base_pose_  = tf::Transform::getIdentity();
  //std::string fixed_frame = "fixed_frame_";
  //std::string base_frame  = "base_frame_";

	
}
**/

GraphManager::~GraphManager() {
  //TODO: delete all Nodes
    //for (unsigned int i = 0; i < optimizer_->vertices().size(); ++i) {
    //Q_FOREACH(Node* node, graph_) { delete node; }
    BOOST_FOREACH(GraphNodeType entry, graph_) { 
      delete entry.second; 
    }
    graph_.clear();
    QMutexLocker locker(&optimizer_mutex_);
    QMutexLocker locker2(&optimization_mutex_);
		//delete optimizer2_;
		//delete slamInterface;
    //delete (optimizer_); optimizer_=NULL; //FIXME: this leads to a double free corruption. Bug in g2o?
    //ransac_marker_pub_.shutdown();
    //whole_cloud_pub_.shutdown();
    //marker_pub_.shutdown();
    //batch_cloud_pub_.shutdown();

}


GraphManager::GraphManager(g2o::G2oSlamInterface* m_interface, g2o::SparseOptimizerIncremental* m_optimizer) :optimizer_(NULL),
next_seq_id(0), next_vertex_id(0),
optimizer_skip_step_(1),min_matches_(20)
{

	backend = "pcg";
	pose_relative_to_ = "inaffected";



  createOptimizer();
  ros::NodeHandle nh;
	 

  
  //marker_pub_ = nh.advertise<visualization_msgs::Marker>("/rgbdslam/pose_graph_markers",0);
  computed_motion_ = tf::Transform::getIdentity();
  init_base_pose_  = tf::Transform::getIdentity();
  //std::string fixed_frame = ParameterServer::instance()->get<std::string>("fixed_frame_name");
  //std::string base_frame  = ParameterServer::instance()->get<std::string>("base_frame_name");

  std::string fixed_frame = "fixed_frame_";
  std::string base_frame  ="base_frame_";
    
  latest_transform_cache_ = tf::StampedTransform(tf::Transform::getIdentity(), ros::Time::now(), base_frame, fixed_frame);
  //timer_ = nh.createTimer(ros::Duration(0.1), &GraphManager::broadcastLatestTransform, this);

	computed_motion_ = tf::Transform::getIdentity();
  init_base_pose_  = tf::Transform::getIdentity();
  //std::string fixed_frame = "fixed_frame_";
  //std::string base_frame  = "base_frame_";

	assert(optimizer2_==NULL);

	optimizer2_ = m_optimizer;
	optimizer2_->setVerbose(false);
  //optimizer.setForceStopFlag(&hasToStop);
  optimizer2_->vizWithGnuplot = true;
	

	
	assert(slamInterface==NULL);
  slamInterface = m_interface;
  slamInterface->setUpdateGraphEachN(10);
  slamInterface->setBatchSolveEachN(100);


	slamInterface->addNode("", 0, 6, std::vector<double>());

 
    


}

void GraphManager::createOptimizer()
{
  QMutexLocker locker(&optimizer_mutex_);
  QMutexLocker locker2(&optimization_mutex_);
  // allocating the optimizer
  
		if(optimizer_ != NULL){
		  for(g2o::SparseOptimizer::VertexIDMap::iterator it = optimizer_->vertices().begin(); it != optimizer_->vertices().end(); it++)
		  {
		    it->second->edges().clear();
		  }
		  for(EdgeSet::iterator it = optimizer_->edges().begin(); it != optimizer_->edges().end(); it++)
		  {
		    //delete *it;
		  }
		  optimizer_->edges().clear();
		  optimizer_->vertices().clear();
		}
		delete optimizer_; 
		optimizer_ = new g2o::SparseOptimizer();
		optimizer_->setVerbose(false);
 
		SlamBlockSolver* solver = NULL;
    if(backend == "cholmod" || backend == "auto"){
      SlamLinearCholmodSolver* linearSolver = new SlamLinearCholmodSolver();
      linearSolver->setBlockOrdering(false);
      solver = new SlamBlockSolver(linearSolver);
      //current_backend_ = "cholmod";
    }
    else if(backend == "csparse"){
      SlamLinearCSparseSolver* linearSolver = new SlamLinearCSparseSolver();
      linearSolver->setBlockOrdering(false);
      solver = new SlamBlockSolver(linearSolver);
      //current_backend_ = "csparse";
    }
    else if(backend == "dense"){
      SlamLinearDenseSolver* linearSolver = new SlamLinearDenseSolver();
      solver = new SlamBlockSolver(linearSolver);
      //current_backend_ = "dense";
    }
    else if(backend == "pcg"){
      SlamLinearPCGSolver* linearSolver = new SlamLinearPCGSolver();
      solver = new SlamBlockSolver(linearSolver);
      //current_backend_ = "pcg";
    }
    else {
      ROS_ERROR("Bad Parameter for g2o Solver backend: %s. User cholmod, csparse or pcg", backend.c_str());
      ROS_INFO("Falling Back to Cholmod Solver");
      SlamLinearCholmodSolver* linearSolver = new SlamLinearCholmodSolver();
      linearSolver->setBlockOrdering(false);
      solver = new SlamBlockSolver(linearSolver);
      //current_backend_ = "cholmod";
    }
    //optimizer_->setSolver(solver);
    g2o::OptimizationAlgorithmLevenberg * algo = new g2o::OptimizationAlgorithmLevenberg(solver);
    //g2o::OptimizationAlgorithmDogleg * algo = new g2o::OptimizationAlgorithmDogleg(solver);
    optimizer_->setAlgorithm(algo);

}

bool GraphManager::addNode(Node* new_node)
{

if (new_node->feature_locations_2d_.size() < min_matches_) {
      //ROS_WARN("Skipping node because it has only %zu features (minimum is %d)",new_node->feature_locations_2d_.size(), ps->get<int>("min_matches______"));
      return false;
  }

  //First Node, so only build its index, insert into storage and add a
  //vertex at the origin, of which the position is very certain
  if (graph_.size()==0){
      firstNode(new_node);
      return true;
  }

//All other nodes are processed in the following
  QMatrix4x4 motion_estimate;///Output:contains the best-yet of the pairwise motion estimates for the current node
  bool edge_to_last_keyframe_found = false;
  bool found_match = nodeComparisons(new_node, motion_estimate, edge_to_last_keyframe_found);

	if (found_match) 
  { //Success

		//This needs to be done before rendering, so deleting the cloud always works
    graph_[new_node->id_] = new_node; //Node->id_ == Graph_ Index


		 if(optimizer_skip_step_ > 0 && 
          (camera_vertices.size() % optimizer_skip_step_) == 0)
      { 
        optimizeGraph();
      }


			visualizeGraphEdges();
      visualizeGraphNodes();



	}
	else{

		if(graph_.size() == 1){//if there is only one node which has less features, replace it by the new one
      //ROS_WARN("Choosing new initial node, because it has more features");
			std::cout<<"Choosing new initial node, because it has more features"<<std::endl;
      if(new_node->feature_locations_2d_.size() > graph_[0]->feature_locations_2d_.size()){
        this->resetGraph();
        //process_node_runs_ = false;
        firstNode(new_node);
        return true;
      }
    } else { //delete new_node; //is now  done by auto_ptr
      //ROS_WARN("Did not add as Node");
			std::cout<<"Did not add as Node"<<std::endl;
			//delete new_node;
			//new_node = NULL;
    }


		
	}


//Q_EMIT setGUIInfo(message.sprintf("%s, Camera Pose Graph Size: %iN/%iE, Duration: %f, Inliers:%5i",// &chi;<sup>2</sup>: %f", 
       //found_match ? "Added" : "Ignored", (int)camera_vertices.size(), (int)cam_cam_edges_.size(), s.elapsed(), (int)curr_best_result_.inlier_matches.size()));//,


			 return found_match;
}


void GraphManager::firstNode(Node* new_node) 
{
    //Q_EMIT renderableOctomap(&co_server_);
    //set the node id only if the node is actually added to the graph
    //needs to be done here as the graph size can change inside this function
    new_node->id_ = graph_.size();
    new_node->seq_id_ = next_seq_id++; // allways incremented, even if node is not added
    init_base_pose_ =  new_node->getBase2PointsTransform();//identity if no MoCap available
    //printTransform("Ground Truth Transform for First Node", init_base_pose_);
    //new_node->buildFlannIndex(); // create index so that next nodes can use it
    g2o::VertexSE3* reference_pose = new g2o::VertexSE3;

    new_node->vertex_id_ = next_vertex_id++;
    graph_[new_node->id_] = new_node;
    reference_pose->setId(new_node->vertex_id_);

    camera_vertices.insert(reference_pose);

    //ROS_INFO("Adding initial node with id %i and seq %i, v_id: %i", new_node->id_, new_node->seq_id_, new_node->vertex_id_);
		Eigen::Quaterniond eigen_quat = Eigen::Quaterniond::Identity();
  	Eigen::Vector3d translation(0.0,0.0,0.0);
    g2o::SE3Quat g2o_ref_se3(eigen_quat, translation);
    reference_pose->setEstimate(g2o_ref_se3);
    reference_pose->setFixed(true);//fix at origin
    optimizer_mutex_.lock();
    optimizer_->addVertex(reference_pose); 
    optimizer_mutex_.unlock();
    //QString message;
    //Q_EMIT setGUIInfo(message.sprintf("Added first node with %i keypoints to the graph", (int)new_node->feature_locations_2d_.size()));
    //pointcloud_type::Ptr the_pc(new_node->pc_col); //this would delete the cloud after the_pc gets out of scope
    //QMatrix4x4 latest_transform = g2o2QMatrix(g2o_ref_se3);


   //process_node_runs_ = false;
}


bool GraphManager::nodeComparisons(Node* new_node, 
                                   QMatrix4x4& curr_motion_estimate,
                                   bool& edge_to_keyframe)///Output:contains the best-yet of the pairwise motion estimates for the current node
{
    /// \callergraph
    //ScopedTimer s(__FUNCTION__);
    //process_node_runs_ = true;

    //ParameterServer* ps = ParameterServer::instance();
    int num_keypoints = (int)new_node->feature_locations_2d_.size();
    if (num_keypoints < min_matches_)
    {
        //ROS_INFO("Found only %i features on image, node is not included", num_keypoints);
				std::cout <<"Found only " << num_keypoints << " features on image, node is not included"<<std::endl;
        //process_node_runs_ = false;
        return false;
    }

    //setting of the node id needs to be done here as the graph size can change inside this method
    new_node->id_ = graph_.size();
    new_node->seq_id_ = next_seq_id++; // allways incremented, even if node is not added


 
		unsigned int num_edges_before = cam_cam_edges_.size();
    int sequentially_previous_id = graph_.rbegin()->second->id_; 
    //int best_match_candidate_id = sequentially_previous_id; 
    MatchingResult mr;
    int prev_best = mr.edge.id1;
    curr_best_result_ = mr;

    //Initial Comparison ######################################################################
    bool predecessor_matched = false;
    //First check if trafo to last frame is big
    //Node* prev_frame = graph_[graph_.size()-1];
    Node* prev_frame = graph_[graph_.size()-1];
    //ROS_INFO("Comparing new node (%i) with previous node %i", new_node->id_, prev_frame->id_);
		std::cout << "Comparing new node "<<new_node->id_<< " with previous node "<<prev_frame->id_ <<std::endl;
    mr = new_node->matchNodePair(prev_frame);
    //ROS_INFO("Node comparison result: %s", mr.toString());
    if(mr.edge.id1 >= 0 && mr.edge.id2 >= 0) {//Found trafo
      ros::Time time1 = pcl_conversions::fromPCL(prev_frame->pc_col->header).stamp;
      ros::Time time2 = pcl_conversions::fromPCL(new_node->pc_col->header).stamp;
      ros::Duration delta_time =  time2 - time1;
      if(!isBigTrafo(mr.edge.transform) || !isSmallTrafo(mr.edge.transform, delta_time.toSec())){ //Found trafo, but bad trafo (too small to big)
          //ROS_WARN("Transformation not within bounds. Did not add as Node");
          curr_best_result_ = mr;
          return false;
      } else { //Good Transformation
				std::cout << "got a good trafo" <<std::endl;
        //ROS_DEBUG_STREAM("Information Matrix for Edge (" << mr.edge.id1 << "<->" << mr.edge.id2 << ") \n" << mr.edge.informationMatrix);
        if (addEdgeToG2O(mr.edge, prev_frame, new_node,  true, true, curr_motion_estimate)) 
        {
          graph_[new_node->id_] = new_node; //Needs to be added
      }
      predecessor_matched = true;
    }
		}
    else {
      //ROS_WARN("Found no transformation to predecessor (edge ids are negative)");
			std::cout<< "Found no transformation to predecessor (edge ids are negative)" <<std::endl;
    // No transformation to predecessor. No other choice than try other candidates. This is done below 
    }
  //end: Initial Comparison ######################################################################


	 //Eigen::Matrix4f ransac_trafo, final_trafo;
    QList<int> vertices_to_comp;
    int  seq_cand = 10; //minus one, because the first predecessor has already been checked
    int geod_cand = 0;
    int samp_cand = 0;
    if(predecessor_matched){
      //vertices_to_comp = getPotentialEdgeTargetsWithDijkstra(new_node, seq_cand, geod_cand, samp_cand, curr_best_result_.edge.id1); 
			  vertices_to_comp = getPotentialEdgeTargetsWithDijkstra(new_node, seq_cand, geod_cand, samp_cand, sequentially_previous_id, true); 
    } else {
      vertices_to_comp = getPotentialEdgeTargetsWithDijkstra(new_node, seq_cand, geod_cand, samp_cand, sequentially_previous_id, true); 
    }
    if(prev_best >= 0 && !vertices_to_comp.contains(prev_best)){
      vertices_to_comp.append(prev_best);//Test: definitely reuse best (currently: the oldest) matched node from last
    }

    QList<const Node* > nodes_to_comp;//only necessary for parallel computation

    //MAIN LOOP: Compare node pairs ######################################################################
    if (true) 
    {
        std::stringstream ss;
        for (int id_of_id = (int) vertices_to_comp.size() - 1; id_of_id >= 0; id_of_id--) {
            //First compile a qlist of the nodes to be compared, then run the comparisons in parallel,
            //collecting a qlist of the results (using the blocking version of mapped).
            nodes_to_comp.push_front(graph_[vertices_to_comp[id_of_id]]); 
            ss << vertices_to_comp[id_of_id] << ", ";
        }
        ROS_INFO_STREAM("Nodes to compare: " << ss);
        QThreadPool* qtp = QThreadPool::globalInstance();
        ROS_INFO("Running node comparisons in parallel in %i (of %i) available threads", qtp->maxThreadCount() - qtp->activeThreadCount(), qtp->maxThreadCount());
        if (qtp->maxThreadCount() - qtp->activeThreadCount() == 1) {
            //Never seen this problem...
            ROS_WARN("Few Threads Remaining: Increasing maxThreadCount to %i", qtp->maxThreadCount()+1);
            qtp->setMaxThreadCount(qtp->maxThreadCount() + 1);
        }
        QList<MatchingResult> results = QtConcurrent::blockingMapped(nodes_to_comp, boost::bind(&Node::matchNodePair, new_node, _1));

        for (int i = 0; i < results.size(); i++) 
        {
            MatchingResult& mr = results[i];
            //ROS_INFO("Result of comparison %d: %s", i, mr.toString());*******************
            if (mr.edge.id1 >= 0 ) {
              //ROS_INFO("new node has id %i", new_node->id_);
              assert(graph_[mr.edge.id1]);

              ros::Duration delta_time = new_node->timestamp_ - graph_[mr.edge.id1]->timestamp_;
              if (isSmallTrafo(mr.edge.transform, delta_time.toSec()) &&
                  addEdgeToG2O(mr.edge,graph_[mr.edge.id1],new_node, isBigTrafo(mr.edge.transform), mr.inlier_matches.size() > curr_best_result_.inlier_matches.size(), curr_motion_estimate))
                { 
                  graph_[new_node->id_] = new_node; //Needs to be added
                  if(mr.edge.id1 == mr.edge.id2-1 ) {//older == newer-1
                    predecessor_matched = true;
                  }

                  updateInlierFeatures(mr, new_node, graph_[mr.edge.id1]);
                  graph_[mr.edge.id1]->valid_tf_estimate_ = true;
                  //ROS_INFO("Added Edge between %i and %i. Inliers: %i",mr.edge.id1,mr.edge.id2,(int) mr.inlier_matches.size());*******************************
                  if (mr.inlier_matches.size() > curr_best_result_.inlier_matches.size()) {
                  //if (curr_best_result_.edge.id1 == -1 || sqrTransNorm(mr.final_trafo) < sqrTransNorm(curr_best_result_.final_trafo)) {
                    curr_best_result_ = mr;
                  }
                  //if(keyframe_ids_.contains(mr.edge.id1)) edge_to_keyframe = true;
                }
                else{
                  ROS_WARN("Rejected edge from %d to %d", mr.edge.id1, mr.edge.id2);
                }
            }
        }
    } 

    //END OF MAIN LOOP: Compare node pairs ######################################################################
    bool found_trafo = (cam_cam_edges_.size() != num_edges_before);
    //bool valid_odometry = !ps->get<std::string>("odom_frame_name").empty();// || odom_tf_old.frame_id_ == "missing_odometry" || odom_tf_new.frame_id_ == "missing_odometry"; 


/**
    bool keep_anyway = (ps->get<bool>("keep_all_nodes") || 
                        (((int)new_node->feature_locations_3d_.size() > ps->get<int>("min_matches")) 
                         && ps->get<bool>("keep_good_nodes")));
    ros::Duration delta_time = new_node->header_.stamp - graph_[sequentially_previous_id]->header_.stamp;
    float time_delta_sec = fabs(delta_time.toSec());
    ROS_WARN_COND(time_delta_sec >= 0.1, "Time jump (time delta: %.2f)", time_delta_sec);

    //If no trafo is found, only keep if a parameter says so or odometry is available. 
    //Otherwise only add a constant position edge, if the predecessor wasn't matched and its timestamp is nearby
    if((!found_trafo && valid_odometry) || 
       ((!found_trafo && keep_anyway) || 
        (!predecessor_matched && time_delta_sec < 0.1))) //FIXME: Add parameter for constant position assumption and time_delta
    { 
      LoadedEdge3D odom_edge;

      odom_edge.id1 = sequentially_previous_id;
      odom_edge.id2 = new_node->id_;
      odom_edge.transform.setIdentity();
      curr_motion_estimate = eigenTF2QMatrix(odom_edge.transform);
      odom_edge.informationMatrix = Eigen::Matrix<double,6,6>::Zero(); 
      ROS_WARN("No valid (sequential) transformation between %d and %d: Using constant position assumption.", odom_edge.id1, odom_edge.id2);
      odom_edge.informationMatrix = Eigen::Matrix<double,6,6>::Identity() / time_delta_sec;//e-9; 
      addEdgeToG2O(odom_edge,graph_[sequentially_previous_id],new_node, true,true, curr_motion_estimate);
      graph_[new_node->id_] = new_node; //Needs to be added
      new_node->valid_tf_estimate_ = false; //Don't use for postprocessing, rendering etc
      MatchingResult mr;
      mr.edge = odom_edge;
      curr_best_result_ = mr;
    }
**/

	return cam_cam_edges_.size() > num_edges_before;

}


bool GraphManager::addEdgeToG2O(const LoadedEdge3D& edge,
                                Node* n1, Node* n2,  
                                bool largeEdge, bool set_estimate, 
                                QMatrix4x4& motion_estimate) //Pure output
{
    //ScopedTimer s(__FUNCTION__);
    assert(n1);
    assert(n2);
    assert(n1->id_ == edge.id1);
    assert(n2->id_ == edge.id2);

    QMutexLocker locker(&optimizer_mutex_);
    g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(n1->vertex_id_));
    g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(n2->vertex_id_));

    // at least one vertex has to be created, assert that the transformation
    // is large enough to avoid to many vertices on the same spot
    if (!v1 || !v2){
        if (!largeEdge) {
            //ROS_INFO("Edge to new vertex is too short, vertex will not be inserted");
						std::cout<< "Edge to new vertex is too short, vertex will not be inserted" <<std::endl;
            return false; 
        }
    }

    if(!v1 && !v2){
      //ROS_ERROR("Missing both vertices: %i, %i, cannot create edge", edge.id1, edge.id2);
			std::cout << "Missing both vertices: "<< edge.id1 << " , " <<  edge.id2 << " , " << "cannot create edge" <<std::endl;
      return false;
    }
    else if (!v1 && v2) {
        v1 = new g2o::VertexSE3;
        assert(v1);
        int v_id = next_vertex_id++;
        v1->setId(v_id);

        n1->vertex_id_ = v_id; // save vertex id in node so that it can find its vertex
        v1->setEstimate(v2->estimate() * edge.transform.inverse());
				camera_vertices.insert(v1);
        optimizer_->addVertex(v1); 
        motion_estimate = eigenTF2QMatrix(v1->estimate()); 
       // ROS_WARN("Creating previous id. This is unexpected by the programmer");
    }
    else if (!v2 && v1) {
        v2 = new g2o::VertexSE3;
        assert(v2);
        int v_id = next_vertex_id++;
        v2->setId(v_id);
        n2->vertex_id_ = v_id;
        v2->setEstimate(v1->estimate() * edge.transform);
				camera_vertices.insert(v2);
        optimizer_->addVertex(v2); 
        motion_estimate = eigenTF2QMatrix(v2->estimate()); 
    }
    else if(set_estimate){
        v2->setEstimate(v1->estimate() * edge.transform);
        motion_estimate = eigenTF2QMatrix(v2->estimate()); 
    }
    g2o::EdgeSE3* g2o_edge = new g2o::EdgeSE3;
    g2o_edge->vertices()[0] = v1;
    g2o_edge->vertices()[1] = v2;
    Eigen::Isometry3d meancopy(edge.transform); 
    g2o_edge->setMeasurement(meancopy);
    //Change setting from which mahal distance the robust kernel is used: robust_kernel_.setDelta(1.0);
    g2o_edge->setRobustKernel(&robust_kernel_);
    // g2o_edge->setInverseMeasurement(edge.trannsform.inverse());
    g2o_edge->setInformation(edge.informationMatrix);
    optimizer_->addEdge(g2o_edge);
    //ROS_DEBUG_STREAM("Added Edge ("<< edge.id1 << "-" << edge.id2 << ") to Optimizer:\n" << edge.transform << "\nInformation Matrix:\n" << edge.informationMatrix);
	  cam_cam_edges_.insert(g2o_edge);
	
		Eigen::Matrix<double, 7, 1> B = g2o::internal::toVectorQT(meancopy);
		
		std::vector<double> A(edge.informationMatrix.data(), edge.informationMatrix.data() + edge.informationMatrix.rows() * edge.informationMatrix.cols());
		std::vector<double> C(B.data(), B.data() + B.rows() * B.cols());

		slamInterface->addEdge("", 0, 6, edge.id1, edge.id2,C,A);
	
    return true;
}


double GraphManager::optimizeGraph(double break_criterion, bool nonthreaded){
  if(!nonthreaded) {
    //ROS_DEBUG("Optimization done in Thread");
    QtConcurrent::run(this, &GraphManager::optimizeGraphImpl, break_criterion); 
    return -1.0;
  }
  else { //Non-concurrent
    return optimizeGraphImpl(break_criterion);//regular function call
  }
}

double GraphManager::optimizeGraphImpl(double break_criterion)
{
  //ScopedTimer s(__FUNCTION__, false, true); // not only for logging
  //ParameterServer* ps = ParameterServer::instance();
  double stop_cond = 0.010;
  //ROS_WARN_NAMED("eval", "Loop Closures: %u, Sequential Edges: %u", loop_closures_edges, sequential_edges);
  //ROS_WARN("Starting Optimization");
  double chi2 = std::numeric_limits<double>::max();
  if(!optimization_mutex_.tryLock(2/*milliseconds*/))
  {
    //ROS_INFO("Attempted Graph Optimization, but it is already running. Skipping.");
		std::cout << "Attempted Graph Optimization, but it is already running. Skipping." <<std::endl;
    return -1.0;
  }
  else //Got the lock
  {
    optimizer_mutex_.lock();
    if(optimizer_->vertices().size() == 0){
      //ROS_ERROR("Attempted Graph Optimization on an empty graph!");
			std::cout<< "Attempted Graph Optimization on an empty graph!" <<std::endl;
      return -1.0;
    }

    fixationOfVertices("first", optimizer_, graph_, camera_vertices, 0); 


   if (pose_relative_to_ == "inaffected") {
     //ScopedTimer s2("Optimizer Initialization (inaffected)");
     g2o::HyperDijkstra hypdij(optimizer_);
     g2o::UniformCostFunction cost_function;
     g2o::VertexSE3* new_vertex = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_[graph_.size()-1]->vertex_id_));
     hypdij.shortestPaths(new_vertex,&cost_function,4);
     g2o::HyperGraph::VertexSet& vs = hypdij.visited();
     optimizer_->initializeOptimization(vs);
   } 

    {
      //ScopedTimer s2("Optimizer Initialization");
      optimizer_->initializeOptimization(cam_cam_edges_);
    }

   // ROS_WARN_NAMED("eval", "Optimization with %zu cams, %zu nodes and %zu edges in the graph", graph_.size(), optimizer_->vertices().size(), optimizer_->edges().size());
    //Q_EMIT iamBusy(1, "Optimizing Graph", 0); 
    int currentIt = 0;
    //Optimize certain number of iterations
    if(stop_cond >= 1.0){ 
	
	
      do {
        currentIt += optimizer_->optimize(ceil(stop_cond / 10));//optimize in maximally 10 steps
      } while(ros::ok() && currentIt < stop_cond && currentIt > 0); //the latter avoids infinite looping if there's nothing to optimize
      optimizer_->computeActiveErrors();
      chi2 = optimizer_->chi2();
    } 
    //Optimize to convergence
    else { 

			std::cerr << "vertices" << optimizer2_->vertices().size() <<std::endl;
			g2o::G2oSlamInterface::SolveResult  solverState= slamInterface->solve();

			std::string res = "";		
			
			if(solverState == 0){
				res = "SOLVED";

			}
			else if(solverState == 1){
				res = "SOLVED_BATCH";
		

			}
			else if(solverState == 2){
				res = "NOOP";



			}
			else if(solverState == 3){
				res = "ERROR";


			}
			std::cerr<< res <<std::endl;
      double prev_chi2;
      do {
        prev_chi2 = chi2; //chi2 is numeric_limits::max() in first iteration
        currentIt += optimizer_->optimize(5);//optimize 5 iterations per step
        optimizer_->computeActiveErrors();
        chi2 = optimizer_->chi2();
      } while(chi2/prev_chi2 < (1.0 - stop_cond));//e.g.  999/1000 < (1.0 - 0.01) => 0.999 < 0.99
    }

    //ROS_WARN_STREAM_NAMED("eval", "G2O Statistics: " << std::setprecision(15) << camera_vertices.size() 
                         // << " cameras, " << optimizer_->edges().size() << " edges. " << chi2
                          //<< " ; chi2 "<< ", Iterations: " << currentIt);
    optimizer_mutex_.unlock();
    optimization_mutex_.unlock();
  }

  // Q_EMIT progress(1, "Optimizing Graph", 1); 

  //ROS_INFO("Finished G2O optimization after %d iterations", i);
  //optimizer_->save((bagfile_name + "_g2o-optimizer-save-file-after").c_str());

  //ROS_INFO("A: last cam: %i", last_added_cam_vertex_id());
	std::cout<< "A: last cam: " << last_added_cam_vertex_id() <<std::endl;

  QMutexLocker locker(&optimizer_mutex_);
  if (pose_relative_to_ == "inaffected") {
    optimizer_->setFixed(camera_vertices, true);
  }
  else {
    optimizer_->setFixed(camera_vertices, false);
  }
  g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(last_added_cam_vertex_id()));
  //ROS_INFO("Sending Transform for Vertex ID: %d", new_node
  computed_motion_ =  eigenTransf2TF(v->estimate());
  Node* newest_node = graph_[graph_.size()-1];
  latest_transform_cache_ = stampedTransformInWorldFrame(newest_node, computed_motion_);
  //printTransform("Computed final transform", latest_transform_cache_);
  //broadcastTransform(latest_transform_cache_);


 

  //ROS_WARN_STREAM_NAMED("eval", "Optimizer Runtime; "<< s.elapsed() <<" s");
  return chi2; 
}



/**
void GraphManager::broadcastTransform(const tf::StampedTransform& stamped_transform) 
{
    br_.sendTransform(stamped_transform);
    if(graph_.size() > 0){
      Node* current_node = graph_.at(graph_.size() - 1);
      if(current_node && current_node->header_.stamp.toSec() == stamped_transform.stamp_.toSec()){
        publishCloud(current_node, current_node->header_.stamp, online_cloud_pub_);
      } else {
        ROS_WARN("Timestamp of transform does not match node");
      }
    }
}
**/

tf::StampedTransform GraphManager::stampedTransformInWorldFrame(const Node* node, const tf::Transform& computed_motion) const 
{
    std::string fixed_frame = "fixed_frame_";
    std::string base_frame  ="base_frame_";
    if(base_frame.empty()){ //if there is no base frame defined, use frame of sensor data
      base_frame = node->pc_col->header.frame_id;
    }
    const tf::StampedTransform& base2points = node->getBase2PointsTransform();//get pose of base w.r.t current pc at capture time

    tf::Transform world2base = init_base_pose_*base2points*computed_motion*base2points.inverse();
    //printTransform("World->Base", world2base);

    return tf::StampedTransform(world2base.inverse(), base2points.stamp_, base_frame, fixed_frame);
}

void GraphManager::resetGraph(){


    next_seq_id = next_vertex_id = 0;
    cam_cam_edges_.clear();
    createOptimizer();

    //Q_FOREACH(Node* node, graph_) { delete node; }
    BOOST_FOREACH(GraphNodeType entry, graph_){ delete entry.second; entry.second = NULL; }
    //for(unsigned int i = 0; i < graph_.size(); delete graph_[i++]);//No body
    graph_.clear();

}




void GraphManager::visualizeGraphEdges() const {
    //ScopedTimer s(__FUNCTION__);

    if (marker_pub_.getNumSubscribers() > 0){ //no visualization for nobody
        ROS_WARN("Sending RVIZ Marker");
        visualization_msgs::Marker edges_marker;
        edges_marker.header.frame_id = "/camera_rgb_optical_frame"; //TODO: Should be a meaningfull fixed frame with known relative pose to the camera
        edges_marker.header.stamp = ros::Time::now();
        edges_marker.ns = "camera_pose_graph"; // Set the namespace and id for this marker.  This serves to create a unique ID
        edges_marker.id = 0;    // Any marker sent with the same namespace and id will overwrite the old one

        edges_marker.type = visualization_msgs::Marker::LINE_LIST;
        edges_marker.action = visualization_msgs::Marker::ADD; // Set the marker action.  Options are ADD and DELETE
        edges_marker.frame_locked = true; //rviz automatically retransforms the markers into the frame every update cycle
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        edges_marker.scale.x = 0.005; //line width
				//edges_marker.scale.y = 0.05;
				//edges_marker.scale.z = 0.05;
        //Global pose (used to transform all points)
        edges_marker.pose.position.x = 0;
        edges_marker.pose.position.y = 0;
        edges_marker.pose.position.z = 0;
        edges_marker.pose.orientation.x = 0.0;
        edges_marker.pose.orientation.y = 0.0;
        edges_marker.pose.orientation.z = 0.0;
        edges_marker.pose.orientation.w = 1.0;
        // Set the color -- be sure to set alpha to something non-zero!
        edges_marker.color.r = 1.0f;
        edges_marker.color.g = 1.0f;
        edges_marker.color.b = 1.0f;
        edges_marker.color.a = 1.0f;//looks smoother
        geometry_msgs::Point point; //start and endpoint for each line segment
        g2o::VertexSE3* v1,* v2; //used in loop
        EdgeSet::iterator edge_iter = cam_cam_edges_.begin();
        int counter = 0;
        for(;edge_iter != cam_cam_edges_.end(); edge_iter++, counter++) {
            g2o::EdgeSE3* myedge = dynamic_cast<g2o::EdgeSE3*>(*edge_iter);
            std::vector<g2o::HyperGraph::Vertex*>& myvertices = myedge->vertices();
            v1 = dynamic_cast<g2o::VertexSE3*>(myvertices.at(1));
            v2 = dynamic_cast<g2o::VertexSE3*>(myvertices.at(0));

            point.x = v1->estimate().translation().x();
            point.y = v1->estimate().translation().y();
            point.z = v1->estimate().translation().z();
            edges_marker.points.push_back(point);
            
            point.x = v2->estimate().translation().x();
            point.y = v2->estimate().translation().y();
            point.z = v2->estimate().translation().z();
            edges_marker.points.push_back(point);
        }

        marker_pub_.publish (edges_marker);
        ROS_INFO("published %d graph edges", counter);
    }

}

void GraphManager::visualizeGraphNodes() const {
    //ScopedTimer s(__FUNCTION__);

    if (marker_pub_.getNumSubscribers() > 0){ //don't visualize, if nobody's looking
        visualization_msgs::Marker nodes_marker;
        nodes_marker.header.frame_id = "/camera_rgb_optical_frame"; //TODO: Should be a meaningfull fixed frame with known relative pose to the camera
        nodes_marker.header.stamp = ros::Time::now();
        nodes_marker.ns = "camera_pose_graph"; // Set the namespace and id for this marker.  This serves to create a unique ID
        nodes_marker.id = 1;    // Any marker sent with the same namespace and id will overwrite the old one


        nodes_marker.type = visualization_msgs::Marker::LINE_LIST;
        nodes_marker.action = visualization_msgs::Marker::ADD; // Set the marker action.  Options are ADD and DELETE
        nodes_marker.frame_locked = true; //rviz automatically retransforms the markers into the frame every update cycle
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        nodes_marker.scale.x = 0.005;
				//nodes_marker.scale.y = 0.02;
				//nodes_marker.scale.z = 0.02;
        //Global pose (used to transform all points) //TODO: is this the default pose anyway?
        nodes_marker.pose.position.x = 0;
        nodes_marker.pose.position.y = 0;
        nodes_marker.pose.position.z = 0;
        nodes_marker.pose.orientation.x = 0.0;
        nodes_marker.pose.orientation.y = 0.0;
        nodes_marker.pose.orientation.z = 0.0;
        nodes_marker.pose.orientation.w = 1.0;
        // Set the color -- be sure to set alpha to something non-zero!
        nodes_marker.color.r = 1.0f;
        nodes_marker.color.g = 0.0f;
        nodes_marker.color.b = 0.0f;
        nodes_marker.color.a = 1.0f;


        geometry_msgs::Point tail; //same startpoint for each line segment
        geometry_msgs::Point tip;  //different endpoint for each line segment
        std_msgs::ColorRGBA arrow_color_red  ;  //red x axis
        arrow_color_red.r = 1.0;
        arrow_color_red.a = 1.0;
        std_msgs::ColorRGBA arrow_color_green;  //green y axis
        arrow_color_green.g = 1.0;
        arrow_color_green.a = 1.0;
        std_msgs::ColorRGBA arrow_color_blue ;  //blue z axis
        arrow_color_blue.b = 1.0;
        arrow_color_blue.a = 1.0;
        Eigen::Vector3d origin(0.0,0.0,0.0);
        Eigen::Vector3d x_axis(0.2,0.0,0.0); //20cm long axis for the first (almost fixed) node
        Eigen::Vector3d y_axis(0.0,0.2,0.0);
        Eigen::Vector3d z_axis(0.0,0.0,0.2);
        Eigen::Vector3d tmp; //the transformed endpoints
        int counter = 0;
        g2o::VertexSE3* v; //used in loop
  for (g2o::HyperGraph::VertexSet::iterator it = camera_vertices.begin(); it != camera_vertices.end(); ++it){

   // VertexIDMap::iterator vertex_iter = optimizer_->vertices().begin();
   // for(/*see above*/; vertex_iter != optimizer_->vertices().end(); vertex_iter++, counter++) {

   v = dynamic_cast<g2o::VertexSE3* >(*it);
            //v->estimate().rotation().x()+ v->estimate().rotation().y()+ v->estimate().rotation().z()+ v->estimate().rotation().w();
            tmp = v->estimate() * origin;
            tail.x = tmp.x();
            tail.y = tmp.y();
            tail.z = tmp.z();
            //Endpoints X-Axis
            nodes_marker.points.push_back(tail);
            nodes_marker.colors.push_back(arrow_color_red);
            tmp = v->estimate() * x_axis;
            tip.x  = tmp.x();
            tip.y  = tmp.y();
            tip.z  = tmp.z();
            nodes_marker.points.push_back(tip);
            nodes_marker.colors.push_back(arrow_color_red);
            //Endpoints Y-Axis
            nodes_marker.points.push_back(tail);
            nodes_marker.colors.push_back(arrow_color_green);
            tmp = v->estimate() * y_axis;
            tip.x  = tmp.x();
            tip.y  = tmp.y();
            tip.z  = tmp.z();
            nodes_marker.points.push_back(tip);
            nodes_marker.colors.push_back(arrow_color_green);
            //Endpoints Z-Axis
            nodes_marker.points.push_back(tail);
            nodes_marker.colors.push_back(arrow_color_blue);
            tmp = v->estimate() * z_axis;
            tip.x  = tmp.x();
            tip.y  = tmp.y();
            tip.z  = tmp.z();
            nodes_marker.points.push_back(tip);
            nodes_marker.colors.push_back(arrow_color_blue);
            //shorten all nodes after the first one
            x_axis.x() = 0.1;
            y_axis.y() = 0.1;
            z_axis.z() = 0.1;
        }

        marker_pub_.publish (nodes_marker);
        ROS_INFO("published %d graph nodes", counter);
    }

}

QList<int> GraphManager::getPotentialEdgeTargetsWithDijkstra(const Node* new_node, int sequential_targets, int geodesic_targets, int sampled_targets, int predecessor_id, bool include_predecessor)
{
    QList<int> ids_to_link_to; //return value
    if(predecessor_id < 0) predecessor_id = graph_.size()-1;
    //Prepare output
    std::stringstream ss;
    ss << "Node ID's to compare with candidate for node " << graph_.size() << ". Sequential: ";

   if((int)camera_vertices.size() <= sequential_targets+geodesic_targets+sampled_targets ||
      camera_vertices.size() <= 1)
    { //if less prev nodes available than targets requestet, just use all
      sequential_targets = sequential_targets+geodesic_targets+sampled_targets;
      geodesic_targets = 0;
      sampled_targets = 0;
      predecessor_id = graph_.size()-1;
    }

    if(sequential_targets > 0){
      //all the sequential targets (will be checked last)
      for (int i=1; i < sequential_targets+1 && predecessor_id-i >= 0; i++) { 
          ids_to_link_to.push_back(predecessor_id-i); 
          ss << ids_to_link_to.back() << ", " ; 
      }
    }


    if(geodesic_targets > 0){
      g2o::HyperDijkstra hypdij(optimizer_);
      g2o::UniformCostFunction cost_function;
      g2o::VertexSE3* prev_vertex = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_[predecessor_id]->vertex_id_));
      hypdij.shortestPaths(prev_vertex,&cost_function,ParameterServer::instance()->get<int>("geodesic_depth"));
      g2o::HyperGraph::VertexSet& vs = hypdij.visited();

      //Need to convert vertex_id to node id
      std::map<int, int> vertex_id_to_node_id;
      for (graph_it it = graph_.begin(); it !=graph_.end(); ++it){
            Node *node = it->second;
            vertex_id_to_node_id[node->vertex_id_] = node->id_;
            //ROS_WARN("ID Pair: %d, %d", node->vertex_id_, node->id_);
      }

			//creating minimum spanning tree minus sequential candidates I believe
      //Geodesic Neighbours except sequential
      std::map<int,int> neighbour_indices; //maps neighbour ids to their weights in sampling
      int sum_of_weights=0;
      for (g2o::HyperGraph::VertexSet::iterator vit=vs.begin(); vit!=vs.end(); vit++) { //FIXME: Mix of vertex id and graph node (with features) id
        int vid = (*vit)->id();
        //ROS_WARN("Vertex ID: %d", vid);
        int id = 0;
        try{
          id = vertex_id_to_node_id.at(vid);
        }
        catch (std::exception e){//Catch exceptions: Unexpected problems shouldn't crash the application
          ROS_ERROR("Vertex ID %d has no corresponding node", vid);
          ROS_ERROR("Map Content:");
          for(std::map<int,int>::const_iterator it = vertex_id_to_node_id.begin(); it != vertex_id_to_node_id.end(); it++){
            ROS_ERROR("Node ID %d: Vertex ID %d", it->first, it->second);
          }
          for(g2o::SparseOptimizer::VertexIDMap::iterator it = optimizer_->vertices().begin(); it != optimizer_->vertices().end(); it++)
          {
            ROS_ERROR("Vertex ID %d", it->first);
          }
        }
        if(!graph_.at(id)->matchable_) continue;
        if(id < predecessor_id-sequential_targets || (id > predecessor_id && id <= (int)graph_.size()-1)){ //Geodesic Neighbours except sequential 
            int weight = abs(predecessor_id-id);
            neighbour_indices[id] = weight; //higher probability to be drawn if far away
            sum_of_weights += weight;
        }
      }
			
			// searching for candidates from minimum spanning tree I believe
      //Sample targets from graph-neighbours
      ss << "Dijkstra: ";
      while(ids_to_link_to.size() < sequential_targets+geodesic_targets && neighbour_indices.size() != 0){ 
        int random_pick = rand() % sum_of_weights;
        ROS_DEBUG("Pick: %d/%d", random_pick, sum_of_weights);
        int weight_so_far = 0;
        for(std::map<int,int>::iterator map_it = neighbour_indices.begin(); map_it != neighbour_indices.end(); map_it++ ){
          weight_so_far += map_it->second;
          ROS_DEBUG("Checking: %d, %d, %d", map_it->first, map_it-> second, weight_so_far);
          if(weight_so_far > random_pick){//found the selected one
            int sampled_id = map_it->first;
            ids_to_link_to.push_front(sampled_id);
            ss << ids_to_link_to.front() << ", " ; 
            sum_of_weights -= map_it->second;
            ROS_DEBUG("Taking ID: %d, decreasing sum of weights to %d", map_it->first, sum_of_weights);
            neighbour_indices.erase(map_it);
            ROS_ERROR_COND(sum_of_weights<0, "Sum of weights should never be zero");
            break;
          }
          ROS_DEBUG("Skipping ID: %d", map_it->first);
        }//for
      }
    }
    
		//for large loop closures
    if(sampled_targets > 0){
      ss << "Random Sampling: ";
      std::vector<int> non_neighbour_indices;//initially holds all, then neighbours are deleted
      non_neighbour_indices.reserve(graph_.size());
      for (QList<int>::iterator it = keyframe_ids_.begin(); it != keyframe_ids_.end(); it++){
        if(ids_to_link_to.contains(*it) == 0 && graph_.at(*it)->matchable_){
          non_neighbour_indices.push_back(*it); 
        }
      }

      //Sample targets from non-neighbours (search new loops)
      while(ids_to_link_to.size() < geodesic_targets+sampled_targets+sequential_targets && non_neighbour_indices.size() != 0){ 
          int index_of_v_id = rand() % non_neighbour_indices.size();
          int sampled_id = non_neighbour_indices[index_of_v_id];
          non_neighbour_indices[index_of_v_id] = non_neighbour_indices.back(); //copy last id to position of the used id
          non_neighbour_indices.resize(non_neighbour_indices.size()-1); //drop last id
          ids_to_link_to.push_front(sampled_id);
          ss << ids_to_link_to.front() << ", " ; 
      }
    }

    if(include_predecessor){
      ids_to_link_to.push_back(predecessor_id); 
      ss << predecessor_id;
    }
    ROS_INFO("%s", ss.str().c_str());
		
    return ids_to_link_to; //only compare to first frame
}

