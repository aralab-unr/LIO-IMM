#include "liom/odom.h"

int main(int argc, char** argv) {

  ros::init(argc, argv, "liom_odom_node");
  ros::NodeHandle nh("~");

  liom::NodeOdometry node(nh);
  ros::AsyncSpinner spinner(0);
  spinner.start();
  node.start();
  ros::waitForShutdown();

  return 0;

}
