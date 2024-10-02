#include "liom/map.h"

int main(int argc, char** argv) {

  ros::init(argc, argv, "liom_map_node");
  ros::NodeHandle nh("~");

  liom::MapNode node(nh);
  ros::AsyncSpinner spinner(0);
  spinner.start();
  node.start();
  ros::waitForShutdown();

  return 0;

}
