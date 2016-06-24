#include "my_slam/node.h"
//#include "my_slam/graph_manager.h"
#include "my_slam/openni_listener.h"
#include "g2o/examples/interactive_slam/g2o_incremental/graph_optimizer_sparse_incremental.h"
#include "g2o/examples/interactive_slam/g2o_interactive/g2o_slam_interface.h"

#include "ros/ros.h"

int main(int argc, char** argv)
{

	ros::init(argc, argv, "my_slam");
	g2o::SparseOptimizerIncremental m_optimizer;
	m_optimizer.setVerbose(false);
	m_optimizer.vizWithGnuplot = true;
	
	g2o::G2oSlamInterface m_slamInterface(&m_optimizer);
	m_slamInterface.setUpdateGraphEachN(10);
  m_slamInterface.setBatchSolveEachN(10);

	GraphManager graph_mgr(&m_slamInterface, &m_optimizer);
	//GraphManager graph_mgr;
  //Instantiate the kinect image listener
	OpenNIListener listener(&graph_mgr);

	while(ros::ok()){
		
		ros::spinOnce();

	}

 
 	//ros::NodeHandle nh;


return 0;
}
