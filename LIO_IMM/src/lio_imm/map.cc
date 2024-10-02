#include "liom/map.h"

#include <filesystem>

liom::MapNode::MapNode(ros::NodeHandle node_handle) : nh(node_handle) {

  this->getParams();

  this->keyframe_sub = this->nh.subscribe("keyframes", 10,
      &liom::MapNode::callbackKeyframe, this, ros::TransportHints().tcpNoDelay());
  this->map_pub = this->nh.advertise<sensor_msgs::PointCloud2>("map", 100);
  this->save_pcd_srv = this->nh.advertiseService("save_pcd", &liom::MapNode::savePcd, this);

  this->liom_map = pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());

  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

}

liom::MapNode::~MapNode() {}

void liom::MapNode::getParams() {

  ros::param::param<std::string>("~liom/odom/odom_frame", this->odom_frame, "odom");
  ros::param::param<double>("~liom/map/sparse/leafSize", this->leaf_size_, 0.5);

  // Get Node NS and Remove Leading Character
  std::string ns = ros::this_node::getNamespace();
  ns.erase(0,1);

  // Concatenate Frame Name Strings
  this->odom_frame = ns + "/" + this->odom_frame;

}

void liom::MapNode::start() {
}

void liom::MapNode::callbackKeyframe(const sensor_msgs::PointCloud2ConstPtr& keyframe) {

  // convert scan to pcl format
  pcl::PointCloud<PointType>::Ptr keyframe_pcl =
    pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());
  pcl::fromROSMsg(*keyframe, *keyframe_pcl);

  // voxel filter
  this->voxelgrid.setLeafSize(this->leaf_size_, this->leaf_size_, this->leaf_size_);
  this->voxelgrid.setInputCloud(keyframe_pcl);
  this->voxelgrid.filter(*keyframe_pcl);

  // save filtered keyframe to map for rviz
  *this->liom_map += *keyframe_pcl;

  // publish full map
  if (this->liom_map->points.size() == this->liom_map->width * this->liom_map->height) {
    sensor_msgs::PointCloud2 map_ros;
    pcl::toROSMsg(*this->liom_map, map_ros);
    map_ros.header.stamp = ros::Time::now();
    map_ros.header.frame_id = this->odom_frame;
    this->map_pub.publish(map_ros);
  }

}

bool liom::MapNode::savePcd(lio_imm::save_pcd::Request& req,
                            lio_imm::save_pcd::Response& res) {

  pcl::PointCloud<PointType>::Ptr m =
    pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>(*this->liom_map));

  float leaf_size = req.leaf_size;
  std::string p = req.save_path;

  if (!std::filesystem::is_directory(p)) {
    std::cout << "Could not find directory " << p << std::endl;
    res.success = false;
    return false;
  }
  
  std::cout << std::setprecision(2) << "Saving map to " << p + "/liom_map.pcd"
    << " with leaf size " << to_string_with_precision(leaf_size, 2) << "... "; std::cout.flush();

  // voxelize map
  pcl::VoxelGrid<PointType> vg;
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.setInputCloud(m);
  vg.filter(*m);

  // save map
  int ret = pcl::io::savePCDFileBinary(p + "/liom_map.pcd", *m);
  res.success = ret == 0;

  if (res.success) {
    std::cout << "done" << std::endl;
  } else {
    std::cout << "failed" << std::endl;
  }

  return res.success;

}
