#include <string>
#include "rviz/selection/selection_manager.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/display_context.h"
#include "rviz/selection/forwards.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/properties/property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/properties/string_property.h"
#include "rviz/view_manager.h"
#include "rviz/view_controller.h"
#include "OGRE/OgreCamera.h"

#include "MapOptimizerSelectionTool.h"

#include <ros/ros.h>
#include <ros/time.h>
#include <QVariant>

#include "map_optimizer_msg/OdomSelectList.h"


namespace map_optimizer_select_tool
{

MapOptimizerSelectionTool::MapOptimizerSelectionTool()
{
    updateTopic();
}

MapOptimizerSelectionTool::~MapOptimizerSelectionTool()
{
}

void MapOptimizerSelectionTool::updateTopic()
{
    odom_select_topic_ = std::string("/map_optimizer/odom_selected");

    odom_select_pub_ = nh_.advertise<map_optimizer_msg::OdomSelectList>( odom_select_topic_.c_str(), 1 );

//    ROS_INFO_STREAM_NAMED("MapOptimizerSelectionTool.updateTopic", "Publishing rviz selected objects on topic " <<  nh_.resolveName (feature_select_pub_) );//<< " with frame_id " << context_->getFixedFrame().toStdString() );

}

int MapOptimizerSelectionTool::processKeyEvent( QKeyEvent* event, rviz::RenderPanel* panel )
{
}

int MapOptimizerSelectionTool::processMouseEvent( rviz::ViewportMouseEvent& event )
{
    int flags = rviz::SelectionTool::processMouseEvent( event );

    // determine current selection mode
    if( event.alt() )
    {
        selecting_ = false;
    }
    else
    {
        if( event.leftDown() )
        {
            selecting_ = true;
        }
    }

    if( selecting_ )
    {
        if( event.leftUp() )
        {
            ROS_INFO_STREAM_NAMED( "MapOptimizerSelectionTool.processKeyEvent", "Using selected area to find a new bounding box and publish the objects inside of it");
            this->processSelection();
        }
    }
    return flags;
}

int MapOptimizerSelectionTool::processSelection()
{
    rviz::SelectionManager* sel_manager = context_->getSelectionManager();
    rviz::M_Picked selection = sel_manager->getSelection();
    rviz::PropertyTreeModel *model = sel_manager->getPropertyModel();
    int num_objs = model->rowCount();
    ROS_INFO_STREAM_NAMED( "MapOptimizerSelectionTool._processSelectedAreaAndFindPoints", "Number of objects in the selected area: " << num_objs);
    if(num_objs < 0)return 0;
    map_optimizer_msg::OdomSelectList msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "";
    for( int i = 0; i < num_objs; i++ )
    {
        QModelIndex child_index = model->index( i, 0 );
        rviz::Property* child = model->getProp( child_index );
        rviz::StringProperty* subchild = (rviz::StringProperty*) child;//child->childAt( 0);

        std::stringstream stream(subchild->getNameStd());
        std::string s;
        std::getline(stream, s, '/');
        if(s != "Marker map_optimizer")
        	continue;

        std::getline(stream, s, '/');

        ROS_INFO_STREAM("Selected: "<<s);
        msg.id_list.push_back(s);
//        if(subchild->getNameStd() == "FeatureID"){
//        	msg.objects.push_back(subchild->getStdString());
//        	std::cout<<"selected obj: "<<subchild->getStdString()<<std::endl;
//        }
    }
    odom_select_pub_.publish(msg);

    return 0;
}


} // end namespace rviz_plugin_selected_points_topic

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( map_optimizer_select_tool::MapOptimizerSelectionTool, rviz::Tool )
