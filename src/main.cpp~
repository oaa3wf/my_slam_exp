#include "my_slam/node.h"
//#include "my_slam/graph_manager.h"
#include "my_slam/openni_listener.h"
#include "g2o/examples/interactive_slam/g2o_incremental/graph_optimizer_sparse_incremental.h"
#include "g2o/examples/interactive_slam/g2o_interactive/g2o_slam_interface.h"

#include "my_slam/qtros.h"
#include <QApplication>
#include <QObject>
#include "my_slam/qt_gui.h"
#include "my_slam/glviewer.h"
#include <Eigen/Core>
#include "my_slam/parameter_server.h"

#include "ros/ros.h"



//TODO:
//better potential-edge-selection through flann or place recognition
//Better separation of function, communication, parameters and gui
//Better integration of the calibration data
//Correct implementation of odometry
//Multi-Camera-fusion

class Renderable; //Fwd decl for signal renderableOctomap
///Connect Signals and Slots for the ui control
void ui_connections(QObject* ui, GraphManager* graph_mgr, OpenNIListener* listener)
{
  Qt::ConnectionType ctype = Qt::AutoConnection;
  if (ParameterServer::instance()->get<bool>("concurrent_io")) 
    ctype = Qt::DirectConnection;
/**
  QObject::connect(ui, SIGNAL(reset()), graph_mgr, SLOT(reset()), ctype);
  QObject::connect(ui, SIGNAL(optimizeGraph()), graph_mgr, SLOT(optimizeGraph()), ctype);
  QObject::connect(ui, SIGNAL(togglePause()), listener, SLOT(togglePause()), ctype);
  QObject::connect(ui, SIGNAL(getOneFrame()), listener, SLOT(getOneFrame()), ctype);
  QObject::connect(ui, SIGNAL(deleteLastFrame()), graph_mgr, SLOT(deleteLastFrame()), ctype);
  QObject::connect(ui, SIGNAL(sendAllClouds()), graph_mgr, SLOT(sendAllClouds()), ctype);
  QObject::connect(ui, SIGNAL(saveAllClouds(QString)), graph_mgr, SLOT(saveAllClouds(QString)), ctype);
  QObject::connect(ui, SIGNAL(saveOctomapSig(QString)), graph_mgr, SLOT(saveOctomap(QString)), ctype);
  QObject::connect(ui, SIGNAL(saveAllFeatures(QString)), graph_mgr, SLOT(saveAllFeatures(QString)), ctype);
  QObject::connect(ui, SIGNAL(saveIndividualClouds(QString)), graph_mgr, SLOT(saveIndividualClouds(QString)), ctype);
  QObject::connect(ui, SIGNAL(saveTrajectory(QString)), graph_mgr, SLOT(saveTrajectory(QString)), ctype);
  QObject::connect(ui, SIGNAL(toggleMapping(bool)), graph_mgr, SLOT(toggleMapping(bool)), ctype);
  QObject::connect(ui, SIGNAL(saveG2OGraph(QString)), graph_mgr, SLOT(saveG2OGraph(QString)), ctype);
**/
  QObject::connect(ui, SIGNAL(togglePause()), listener, SLOT(togglePause()), ctype);
  QObject::connect(ui, SIGNAL(toggleMapping(bool)), graph_mgr, SLOT(toggleMapping(bool)), ctype);
}

///Connect Signals and Slots only relevant for the graphical interface
void gui_connections(Graphical_UI* gui, GraphManager* graph_mgr, OpenNIListener* listener)
{

    QObject::connect(listener,  SIGNAL(newVisualImage(QImage)), gui, SLOT(setVisualImage(QImage)));
    QObject::connect(listener,  SIGNAL(newFeatureFlowImage(QImage)), gui, SLOT(setFeatureFlowImage(QImage)));
    QObject::connect(listener,  SIGNAL(newFeatureImage(QImage)), gui, SLOT(setFeatureImage(QImage)));
    QObject::connect(listener,  SIGNAL(newDepthImage(QImage)), gui, SLOT(setDepthImage(QImage)));
/**
    QObject::connect(graph_mgr, SIGNAL(sendFinished()), gui, SLOT(sendFinished()));
    QObject::connect(graph_mgr, SIGNAL(iamBusy(int, const char*, int)), gui, SLOT(showBusy(int, const char*, int)));
    QObject::connect(graph_mgr, SIGNAL(progress(int, const char*, int)), gui, SLOT(setBusy(int, const char*, int)));
    QObject::connect(listener, SIGNAL(iamBusy(int, const char*, int)), gui, SLOT(showBusy(int, const char*, int)));
    QObject::connect(listener, SIGNAL(progress(int, const char*, int)), gui, SLOT(setBusy(int, const char*, int)));
    QObject::connect(graph_mgr, SIGNAL(setGUIInfo(QString)), gui, SLOT(setInfo(QString)));
    QObject::connect(graph_mgr, SIGNAL(setGUIStatus(QString)), gui, SLOT(setStatus(QString)));
    QObject::connect(gui, SIGNAL(printEdgeErrors(QString)), graph_mgr, SLOT(printEdgeErrors(QString)));
    QObject::connect(gui, SIGNAL(pruneEdgesWithErrorAbove(float)), graph_mgr, SLOT(pruneEdgesWithErrorAbove(float)));
    QObject::connect(gui, SIGNAL(clearClouds()), graph_mgr, SLOT(clearPointClouds()));
    QObject::connect(gui, SIGNAL(occupancyFilterClouds()), graph_mgr, SLOT(occupancyFilterClouds()));
    QObject::connect(gui, SIGNAL(saveBagfile(QString)), graph_mgr, SLOT(saveBagfileAsync(QString)));
    QObject::connect(gui, SIGNAL(openPCDFiles(QStringList)), listener, SLOT(loadPCDFiles(QStringList)));
    QObject::connect(gui, SIGNAL(openBagFile(QString)), listener, SLOT(loadBagFileFromGUI(QString)));
**/
    if (ParameterServer::instance()->get<bool>("use_glwidget") && gui->getGLViewer() != NULL) {
      GLViewer* glv = gui->getGLViewer();
	    QObject::connect(graph_mgr, SIGNAL(setPointCloud(pointcloud_type *, QMatrix4x4)), glv, SLOT(addPointCloud(pointcloud_type *, QMatrix4x4)), Qt::BlockingQueuedConnection ); //Needs to block, otherwise the opengl list compilation makes the app unresponsive. This effectively throttles the processing rate though
      typedef const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >* cnst_ft_vectors;
	    QObject::connect(graph_mgr, SIGNAL(setFeatures(const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >*)), glv, SLOT(addFeatures(const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >*))); //, Qt::DirectConnection);
	    QObject::connect(graph_mgr, SIGNAL(setGraphEdges(const QList<QPair<int, int> >*)), glv, SLOT(setEdges(const QList<QPair<int, int> >*)));
	    QObject::connect(graph_mgr, SIGNAL(updateTransforms(QList<QMatrix4x4>*)), glv, SLOT(updateTransforms(QList<QMatrix4x4>*)));
      QObject::connect(graph_mgr, SIGNAL(deleteLastNode()), glv, SLOT(deleteLastNode()));
	    QObject::connect(graph_mgr, SIGNAL(resetGLViewer()),  glv, SLOT(reset()));
	    QObject::connect(graph_mgr, SIGNAL(renderableOctomap(Renderable*)),  glv, SLOT(setRenderable(Renderable*)));
      //QObject::connect(glv, SIGNAL(clickedPosition(float,float,float)), graph_mgr, SLOT(filterNodesByPosition(float,float,float)));
      if(!ParameterServer::instance()->get<bool>("store_pointclouds")) {
          QObject::connect(glv, SIGNAL(cloudRendered(pointcloud_type *)), graph_mgr, SLOT(clearPointCloud(pointcloud_type const *))); // 
      } else if(ParameterServer::instance()->get<double>("voxelfilter_size") > 0.0) {
          QObject::connect(glv, SIGNAL(cloudRendered(pointcloud_type *)), graph_mgr, SLOT(reducePointCloud(pointcloud_type const *))); // 
      }
    }
    QObject::connect(listener, SIGNAL(setGUIInfo(QString)), gui, SLOT(setInfo(QString)));
    QObject::connect(listener, SIGNAL(setGUIStatus(QString)), gui, SLOT(setStatus(QString)));
    QObject::connect(listener, SIGNAL(setGUIInfo2(QString)), gui, SLOT(setInfo2(QString)));
    QObject::connect(graph_mgr, SIGNAL(setGUIInfo2(QString)), gui, SLOT(setInfo2(QString)));
}

int main(int argc, char** argv)
{


  //create thread object, to run the ros event processing loop in parallel to the qt loop
  QtROS qtRos(argc, argv, "my_slam"); //ros node name & namespace
	//ros::init(argc, argv, "my_slam");

	//Depending an use_gui on the Parameter Server, a gui- or a headless application is used
  QApplication app(argc, argv, ParameterServer::instance()->get<bool>("use_gui")); 

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


  Graphical_UI* gui = NULL;
	if (app.type() == QApplication::GuiClient){
      gui = new Graphical_UI();
      gui->show();
      gui_connections(gui, &graph_mgr, &listener);
      ui_connections(gui, &graph_mgr, &listener);//common connections for the user interfaces
  } else {
      ROS_WARN("Running without graphical user interface! See README or wiki page for how to interact with RGBDSLAM.");
  }
  //Create Ros service interface with or without gui
  //RosUi ui("rgbdslam"); //ui namespace for service calls
  //ui_connections(&ui, &graph_mgr, &listener);//common connections for the user interfaces

  //If one thread receives a exit signal from the user, signal the other thread to quit too
  QObject::connect(&app, SIGNAL(aboutToQuit()), &qtRos, SLOT(quitNow()));
  QObject::connect(&qtRos, SIGNAL(rosQuits()), &app, SLOT(quit()));

  qtRos.start();// Run main loop.
  app.exec();
/**
	while(ros::ok()){
		
		ros::spinOnce();

	}
**/
 
 	//ros::NodeHandle nh;


return 0;
}
