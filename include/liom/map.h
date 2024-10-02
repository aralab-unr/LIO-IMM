
#include "liom/liom.h"

class liom::MapNode {

public:

  MapNode(ros::NodeHandle node_handle);
  ~MapNode();

  void start();

private:

  void getParams();

  void callbackKeyframe(const sensor_msgs::PointCloud2ConstPtr& keyframe);

  bool savePcd(lio_imm::save_pcd::Request& req,
               lio_imm::save_pcd::Response& res);

  ros::NodeHandle nh;
  ros::Subscriber keyframe_sub;
  ros::Publisher map_pub;
  ros::ServiceServer save_pcd_srv;
  pcl::PointCloud<PointType>::Ptr liom_map;
  pcl::VoxelGrid<PointType> voxelgrid;
  std::string odom_frame;

  double leaf_size_;

};
