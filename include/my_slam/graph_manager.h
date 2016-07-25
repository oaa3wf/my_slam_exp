#ifndef MY_SLAM_GRAPH_MANAGER_H
#define MY_SLAM_GRAPH_MAHAGER_H

#include "my_slam/node.h"
#include "my_slam/parameter_server.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/core/sparse_optimizer.h"
//#include "g2o/types/slam3d/parameter_camera.h"
//#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/core/hyper_dijkstra.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/examples/interactive_slam/g2o_incremental/graph_optimizer_sparse_incremental.h"
#include "g2o/examples/interactive_slam/g2o_interactive/g2o_slam_interface.h"



#include <tf/transform_broadcaster.h>


#include <QMatrix4x4>
#include <QMutex>
#include <QObject>

#include "my_slam/parameter_server.h"


//typedef g2o::HyperGraph::VertexSet::iterator Vset_it;
///Type to iterate over graph map
typedef std::map<int, Node* >::iterator graph_it;
typedef std::set<g2o::HyperGraph::Edge*> EdgeSet;
typedef g2o::HyperGraph::EdgeSet::iterator EdgeSet_it;
typedef std::map<int, Node* >::iterator graph_it;

class GraphManager : public QObject {
		Q_OBJECT
		Q_SIGNALS:
///Connect to this signal to get the transformation matrix from the last frame as QString
      void newTransformationMatrix(QString);
      void sendFinished();
      void setGUIInfo(QString message);
      void setGUIStatus(QString message);
      void setPointCloud(pointcloud_type * pc, QMatrix4x4 transformation);
      void setFeatures(const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >*);
      void updateTransforms(QList<QMatrix4x4>* transformations);
      void setGUIInfo2(QString message);
      void setGraphEdges(const QList<QPair<int, int> >* edge_list);
      void deleteLastNode();
      void resetGLViewer();
      void setGraph(const g2o::OptimizableGraph*);
      void iamBusy(int id, const char* message, int max);
      void progress(int id, const char* message, int val);
      //void renderableOctomap(Renderable* r);

	public:
	//GraphManager();
  GraphManager(g2o::G2oSlamInterface* m_interface, g2o::SparseOptimizerIncremental* m_optimizer);
  ~GraphManager();

    //! Add new node to the graph.
    /// Node will be included, if a valid transformation to one of the former nodes
    /// can be found. If appropriate, the graph is optimized
    /// graphmanager owns newNode after this call. Do no delete the object
    /// \callergraph
    bool addNode(Node* new_node); 

    //! Try to compute transformations to previous nodes
    /// getPotentialEdgeTargetsWithDijkstra is used to select
    /// previous nodes to match against, then the comparison
    /// of nodes is called, possibly in parallel.
    /// \callergraph
    bool nodeComparisons(Node* new_node, QMatrix4x4& curr_motion_estimate,bool& edge_to_keyframe);///Output:contains the best-yet of the pairwise motion estimates for the current node


    double optimizeGraph(double iter = -1, bool nonthreaded=false);

		double optimizeGraphImpl(double max_iter);


    ///Adds an visual-feature-correspondence edge to the optimizer
    bool addEdgeToG2O(const LoadedEdge3D& edge, Node* n1, Node* n2, bool good_edge, bool set_estimate, QMatrix4x4& motion_estimate);


    void createOptimizer();


    void resetGraph();

    ///Add a keyframe to the list (and log keyframes)
    void addKeyframe(int id);

		typedef std::pair<int, Node*> GraphNodeType;


    int last_added_cam_vertex_id(){
      return graph_[graph_.size()-1]->vertex_id_;
    };


    /// Adds the first node
    void firstNode(Node* new_node);


    //RVIZ visualization stuff (also in graph_mgr_io.cpp)
    ///Send markers to visualize the graph edges (cam transforms) in rviz (if somebody subscribed)
    void visualizeGraphEdges() const;
    ///Send markers to visualize the graph nodes (cam positions) in rviz (if somebody subscribed)
    void visualizeGraphNodes() const;

		void visualizeClouds();

    ///Compute the transform between the fixed frame (usually the initial position of the cam) 
    /// and the node from the motion (given in sensor frame)
    tf::StampedTransform stampedTransformInWorldFrame(const Node* node, 
                                                      const tf::Transform& computed_motion) const;


		QList<int> getPotentialEdgeTargetsWithDijkstra(const Node* new_node, int sequential_targets, int geodesic_targets, int sampled_targets, int predecessor_id, bool include_predecessor);

		QList<QMatrix4x4>* getAllPosesAsMatrixList() const;


    //!This mutex restricts access to the optimizer's data structures
    QMutex optimizer_mutex_;
    //!This mutex restricts the optimization computation itself
    QMutex optimization_mutex_; 

    mutable tf::TransformBroadcaster br_;
    tf::Transform  init_base_pose_;
    tf::Transform computed_motion_; ///<transformation of the last frame to the first frame (assuming the first one is fixed)
    tf::StampedTransform latest_transform_cache_;//base_frame -> optical_frame 
    g2o::RobustKernelHuber robust_kernel_;
    g2o::SparseOptimizer* optimizer_;
		g2o::SparseOptimizerIncremental* optimizer2_;
		g2o::G2oSlamInterface* slamInterface;
    unsigned int next_seq_id;
    unsigned int next_vertex_id;
    std::map<int, Node* > graph_;
    int optimizer_skip_step_;
    int min_matches_;
    std::string backend;
    std::string pose_relative_to_;
    ///will contain the motion to the best matching node
    MatchingResult curr_best_result_; 
    ///Pose vertices (in camera coordinate system)
    g2o::HyperGraph::VertexSet camera_vertices;
    ///"Regular" edges from camera to camera as found form feature correspondeces
    g2o::HyperGraph::EdgeSet cam_cam_edges_;
		ros::Publisher marker_pub_; 
		ros::Publisher cloud_pub_;
		unsigned int geodesic_depth_;
		QList<int> keyframe_ids_;//Keyframes are added, if no previous keyframe was matched
	  int earliest_loop_closure_node_;
		int predecessor_candidates_;
		unsigned int loop_closures_edges, sequential_edges;

};

#endif
