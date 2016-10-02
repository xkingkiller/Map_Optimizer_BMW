#include <ros/ros.h>

#include "map_optimizer.h"

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "map_optimizer");

  MapOptimizer mo;
  //mo.startLiveSlam();
  ros::spin();

  return(0);
}

