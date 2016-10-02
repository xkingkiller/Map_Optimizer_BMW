#ifndef SELECTED_POINTS_PUBLISHER_H
#define SELECTED_POINTS_PUBLISHER_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <ros/node_handle.h>
# include <ros/publisher.h>

# include "rviz/tool.h"

# include <QCursor>
# include <QObject>
#endif

#include "rviz/default_plugin/tools/selection_tool.h"

namespace map_optimizer_select_tool
{

class MapOptimizerSelectionTool : public rviz::SelectionTool
{
Q_OBJECT
public:
  MapOptimizerSelectionTool();
  ~MapOptimizerSelectionTool();

  /*
   * Hooks on rviz::SelectionTool::processMouseEvent() to get and publish
   * selected points
   */
  virtual int processMouseEvent( rviz::ViewportMouseEvent& event );

  virtual int processKeyEvent( QKeyEvent* event, rviz::RenderPanel* panel );

public Q_SLOTS:
  /*
   * Creates the ROS topic
   */
  void updateTopic();

//  void PointCloudsCallback(const sensor_msgs::PointCloud2ConstPtr &pc_msg);

protected:

  int processSelection();
  ros::NodeHandle nh_;
  ros::Publisher odom_select_pub_;
  std::string odom_select_topic_;
  bool selecting_;
};
} // end namespace map_optimizer_select_tool

#endif // SELECTED_POINTS_PUBLISHER_H

////////////////////////////////////////////////////////////////////////////////

