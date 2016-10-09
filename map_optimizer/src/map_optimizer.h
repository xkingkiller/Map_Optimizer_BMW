#ifndef MAP_OPTIMIZER_H_
#define MAP_OPTIMIZER_H_
#include "ros/ros.h"
#include <tf/transform_listener.h>

#include "gmapping/gridfastslam/gridslamprocessor.h"

//#include "g2o/core/sparse_optimizer.h"
//#include "g2o/types/slam2d/vertex_se2.h"
#include "sensor_msgs/PointCloud.h"
#include "laser_geometry/laser_geometry.h"
#include "map_optimizer_msg/OdomSelectList.h"
#include "map_optimizer_msg/SelectMode.h"

#include "graph_manager.h"

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <map>
#include <vector>

class MapOptimizer
{
	//using TNode = GMapping::GridSlamProcessor::TNode;
	typedef GMapping::GridSlamProcessor::TNode TNode;

  public:
	MapOptimizer();
    ~MapOptimizer();

    void init();
  private:
    ros::NodeHandle private_nh_;
    void loadMapData();
    sensor_msgs::PointCloudPtr loadLaserScan(const std::string& scan_file);
    //ros parameters
    std::string mapdata_folder_;
    std::string poseFile_;
    std::string baseFrame_;
    std::string fixedFrame_;
    boost::shared_ptr< tf::TransformListener > tflistener_;
    //mapping trajectory
    TNode* node_;
    std::map< int, sensor_msgs::PointCloudPtr > scans_buf_;
    std::map< int, TNode* > node_map_;
    std::map< int, double > node_time_buf_;
    std::vector<int> id_buf_;
    std::vector<int> const_id_buf_;
    std::vector<int> ref_id_buf_;
//    std::vector< GraphManager::OdomPose > fixed_node_buf_;
    int totle_size_;
    //deal selection
    unsigned char sel_mode;
    void handleSelectionMode(const map_optimizer_msg::SelectMode::ConstPtr& msg);
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
    std::string se2_marker_name;
    void makeSE2Marker(const tf::Transform& t);
    void processMarkerFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
    void updateTargetLaserPose(const tf::Transform& t, int ind);

    int target_pose_index_;
    tf::Transform target_tf_;
    // menu handler
    interactive_markers::MenuHandler menu_handler_;
    interactive_markers::MenuHandler::EntryHandle h_mode_last_;
    interactive_markers::MenuHandler::EntryHandle h_mode_ref_laser_;
    interactive_markers::MenuHandler::EntryHandle h_mode_target_laser_;
    interactive_markers::MenuHandler::EntryHandle h_mode_ref_opt_laser_;

    void initMenu(const tf::Transform& t);
    void menuModeCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void menuAddConstCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void menuAddFixedConstCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void menuRefineCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void menuExportOptPosesCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    //
    ros::Publisher poses_pub;
    void publishPoses();
    ros::Publisher selected_scans_pub;
    ros::Publisher target_scan_pub;
    sensor_msgs::PointCloudPtr createLaserScansMsg(const std::vector< int >& ids, bool useOpt = false);
    //Handle for selection event
    ros::Subscriber odom_select_sub;
    void handleOdomSelect(const map_optimizer_msg::OdomSelectList::ConstPtr& msg);
    //
    laser_geometry::LaserProjection projector_;
    //graph manager
    boost::shared_ptr< GraphManager > graph_manager_;
    void createGraph();
    void addConstraint(const tf::Transform& newPose, int id);
    void addFixedConstraint(const tf::Transform& newPose, int id);
    void optimize();
    int findNearestRefId(int tid);
    ros::Publisher opt_poses_pub_;
    void publishOptimizedPoses();
    // export result
    void exportOptPoses();

};
#endif //MAP_OPTIMIZER_H_
