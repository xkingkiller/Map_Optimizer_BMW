#!/bin/bash
DIRECT="$(rospack find had_launch)/demo/common/scripts/direct.sh"
NODESTART="$(rospack find had_launch)/demo/common/scripts/nodestart.sh"
echo $SCRIPT
xfce4-terminal -T map_opt -e "bash -ic '$NODESTART 0 $(rospack find map_optimizer)/launch/ map_optimizer.launch'" \
  --tab -T rviz -e "bash -ic '$DIRECT 0 rviz -d $(rospack find map_optimizer)/rviz/map_optmizer.rviz'" \
