#include "liom/odom.h"

liom::NodeOdometry::NodeOdometry(ros::NodeHandle node_handle) : nh(node_handle) {

    // Initialize Parameters
    this->getParams();

    this->liom_initialized  = false;
    this->first_valid_scan  = false;         // First LiDAR scan
    this->first_imu_received = false;         // First IMU meas
    if (this->imu_calibrate_) {this->imu_calibrated = false;}
    else {this->imu_calibrated = true;}
    this->deskew_status = false;
    this->deskew_size = 0;

    // Subscriber nodes
    this->lidar_sub = this->nh.subscribe("pointcloud", 1,
        &liom::NodeOdometry::callbackPointCloud, this, ros::TransportHints().tcpNoDelay());
    
    this->imu_sub   = this->nh.subscribe("imu", 1000,
        &liom::NodeOdometry::callbackImu, this, ros::TransportHints().tcpNoDelay());

    // Publisher Nodes
    this->odom_pub       = this->nh.advertise<nav_msgs::Odometry>("odom", 1, true);
    this->pose_pub       = this->nh.advertise<geometry_msgs::PoseStamped>("pose", 1, true);
    this->path_pub       = this->nh.advertise<nav_msgs::Path>("path", 1, true);
    this->kf_pose_pub    = this->nh.advertise<geometry_msgs::PoseArray>("kf_pose", 1, true);
    this->kf_cloud_pub   = this->nh.advertise<sensor_msgs::PointCloud2>("kf_cloud", 1, true);
    this->deskewed_pub   = this->nh.advertise<sensor_msgs::PointCloud2>("deskewed", 1, true);

    this->publish_timer  = this->nh.createTimer(ros::Duration(0.01), &liom::NodeOdometry::publishPose, this);

    // Initialize Tranformation
    this->T         = Eigen::Matrix4f::Identity();
    this->T_prior   = Eigen::Matrix4f::Identity();
    this->T_corr    = Eigen::Matrix4f::Identity();

    // Initialize State
    this->origin            = Eigen::Vector3f(0, 0, 0);
    this->state.p           = Eigen::Vector3f(0, 0, 0);        // Position
    this->state.q           = Eigen::Quaternionf(1, 0, 0, 0);  // Orientation
    this->state.v.linear.b  = Eigen::Vector3f(0, 0, 0);
    this->state.v.linear.w  = Eigen::Vector3f(0, 0, 0);
    this->state.v.angl.b    = Eigen::Vector3f(0, 0, 0);
    this->state.v.angl.w    = Eigen::Vector3f(0, 0, 0);

    // Initialize LiDAR pose
    this->lidarPose.p   = Eigen::Vector3f(0, 0, 0);
    this->lidarPose.q   = Eigen::Quaternionf(1, 0, 0, 0);

    // Initialize IMU meas
    this->imu_meas.stamp            = 0;          // Timestamp
    this->imu_meas.angl_vel[0]      = 0;          // Angular velocity
    this->imu_meas.angl_vel[1]      = 0;
    this->imu_meas.angl_vel[2]      = 0;
    this->imu_meas.linear_accel[0]  = 0;          // Linear velocity
    this->imu_meas.linear_accel[1]  = 0;
    this->imu_meas.linear_accel[2]  = 0;

    this->imu_buffer.set_capacity(this->imu_buffer_size_);
    this->first_imu_stamp    = 0;                // First IMU stamp
    this->prev_imu_stamp     = 0;                // Previous IMU stamp

    // Initialize original LiDAR scan, deskewed scan, current scan and submap (Local map)
    this->original_scan = pcl::PointCloud<PointType>::ConstPtr (boost::make_shared<const pcl::PointCloud<PointType>>());
    this->deskewed_scan = pcl::PointCloud<PointType>::ConstPtr (boost::make_shared<const pcl::PointCloud<PointType>>());
    this->current_scan = pcl::PointCloud<PointType>::ConstPtr (boost::make_shared<const pcl::PointCloud<PointType>>());
    this->submap_cloud = pcl::PointCloud<PointType>::ConstPtr (boost::make_shared<const pcl::PointCloud<PointType>>());

    this->num_processed_keyframes   = 0;

    this->submap_hasChanged = true;
    this->submap_kf_idx_prev.clear();

    this->first_scan_stamp = 0.;
    this->elapsed_time = 0.;
    this->length_traversed;


    this->convex_hull.setDimension(3);
    this->concave_hull.setDimension(3);
    this->concave_hull.setAlpha(this->keyframe_thresh_dist_);
    this->concave_hull.setKeepInformation(true);

    this->gicp.setCorrespondenceRandomness(this->gicp_k_correspondences_);
    this->gicp.setMaxCorrespondenceDistance(this->gicp_max_corr_dist_);
    this->gicp.setMaximumIterations(this->gicp_max_iter_);
    this->gicp.setTransformationEpsilon(this->gicp_transformation_ep_);
    this->gicp.setRotationEpsilon(this->gicp_rotation_ep_);
    this->gicp.setInitialLambdaFactor(this->gicp_init_lambda_factor_);

    this->gicp_temp.setCorrespondenceRandomness(this->gicp_k_correspondences_);
    this->gicp_temp.setMaxCorrespondenceDistance(this->gicp_max_corr_dist_);
    this->gicp_temp.setMaximumIterations(this->gicp_max_iter_);
    this->gicp_temp.setTransformationEpsilon(this->gicp_transformation_ep_);
    this->gicp_temp.setRotationEpsilon(this->gicp_rotation_ep_);
    this->gicp_temp.setInitialLambdaFactor(this->gicp_init_lambda_factor_);

    pcl::Registration<PointType, PointType>::KdTreeReciprocalPtr temp;
    this->gicp.setSearchMethodSource(temp, true);
    this->gicp.setSearchMethodTarget(temp, true);
    this->gicp_temp.setSearchMethodSource(temp, true);
    this->gicp_temp.setSearchMethodTarget(temp, true);

    this->geo.first_opt_done = false;
    this->geo.prev_vel = Eigen::Vector3f(0., 0., 0.);

    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    this->crop.setNegative(true);
    this->crop.setMin(Eigen::Vector4f(-this->crop_size_, -this->crop_size_, -this->crop_size_, 1.0));
    this->crop.setMax(Eigen::Vector4f(this->crop_size_, this->crop_size_, this->crop_size_, 1.0));

    this->voxel.setLeafSize(this->vf_res_, this->vf_res_, this->vf_res_);

    this->metrics.spaciousness.push_back(0.);
    this->metrics.density.push_back(this->gicp_max_corr_dist_);

    // CPU Specs
    char CPUBrandString[0x40];
    memset(CPUBrandString, 0, sizeof(CPUBrandString));

    this->cpu_type = "";

    #ifdef HAS_CPUID
    unsigned int CPUInfo[4] = {0,0,0,0};
    __cpuid(0x80000000, CPUInfo[0], CPUInfo[1], CPUInfo[2], CPUInfo[3]);
    unsigned int nExIds = CPUInfo[0];
    for (unsigned int i = 0x80000000; i <= nExIds; ++i) {
        __cpuid(i, CPUInfo[0], CPUInfo[1], CPUInfo[2], CPUInfo[3]);
        if (i == 0x80000002)
        memcpy(CPUBrandString, CPUInfo, sizeof(CPUInfo));
        else if (i == 0x80000003)
        memcpy(CPUBrandString + 16, CPUInfo, sizeof(CPUInfo));
        else if (i == 0x80000004)
        memcpy(CPUBrandString + 32, CPUInfo, sizeof(CPUInfo));
    }
    this->cpu_type = CPUBrandString;
    boost::trim(this->cpu_type);
    #endif

    FILE* file;
    struct tms timeSample;
    char line[128];

    this->lastCPU = times(&timeSample);
    this->lastSysCPU = timeSample.tms_stime;
    this->lastUserCPU = timeSample.tms_utime;

    file = fopen("/proc/cpuinfo", "r");
    this->numProcessors = 0;
    while(fgets(line, 128, file) != nullptr) {
        if (strncmp(line, "processor", 9) == 0) this->numProcessors++;
    }
    fclose(file);

}

liom::NodeOdometry::~NodeOdometry() {}

/**
    getParameter Function
**/
void liom::NodeOdometry::getParams() {

    // Version
    ros::param::param<std::string>("~liom/version", this->version_, "0.0.0");

    // Frames
    ros::param::param<std::string>("~liom/frames/odom", this->odom_frame, "odom");
    ros::param::param<std::string>("~liom/frames/baselink", this->baselink_frame, "base_link");
    ros::param::param<std::string>("~liom/frames/lidar", this->lidar_frame, "lidar");
    ros::param::param<std::string>("~liom/frames/imu", this->imu_frame, "imu");

    // Get Node NS and Remove Leading Character
    std::string ns = ros::this_node::getNamespace();
    ns.erase(0, 1);

    // Concatenate Frame Name Strings
    this->odom_frame = ns + "/" + this->odom_frame;
    this->baselink_frame = ns + "/" + this->baselink_frame;
    this->lidar_frame = ns + "/" + this->lidar_frame;
    this->imu_frame = ns + "/" + this->imu_frame;
   
    // Deskew Flag
    ros::param::param<bool>("~liom/pointcloud/deskew", this->deskew_, true);

    // Gravity
    ros::param::param<double>("~liom/odom/gravity", this->gravity_, 9.80665);  

    // Keyframe Threshold
    ros::param::param<double>("~liom/odom/keyframe/threshD", this->keyframe_thresh_dist_, 0.1);
    ros::param::param<double>("~liom/odom/keyframe/threshR", this->keyframe_thresh_rot_, 1.0);

    // Submap
    ros::param::param<int>("~liom/odom/submap/keyframe/knn", this->submap_knn_, 10);
    ros::param::param<int>("~liom/odom/submap/keyframe/kcv", this->submap_kcv_, 10);
    ros::param::param<int>("~liom/odom/submap/keyframe/kcc", this->submap_kcc_, 10);

    // Dense map resolution
    ros::param::param<bool>("~liom/map/dense/filtered", this->densemap_filtered_, true);

    // Wait until movement to publish map
    ros::param::param<bool>("~liom/map/waitUntilMove", this->wait_until_move_, false);

    // Crop Box Filter
    ros::param::param<double>("~liom/odom/preprocessing/cropBoxFilter/size", this->crop_size_, 1.0);

    // Voxel Grid Filter
    ros::param::param<bool>("~liom/pointcloud/voxelize", this->vf_use_, true);
    ros::param::param<double>("~liom/odom/preprocessing/voxelFilter/res", this->vf_res_, 0.05);

    // Adaptive Parameters
    ros::param::param<bool>("~liom/adaptive", this->adaptive_params_, true);

    // Extrinsics
    std::vector<float> t_default{0., 0., 0.};
    std::vector<float> R_default{1., 0., 0., 0., 1., 0., 0., 0., 1.};

    // center of gravity to imu
    std::vector<float> baselink2imu_t, baselink2imu_R;
    ros::param::param<std::vector<float>>("~liom/extrinsics/baselink2imu/t", baselink2imu_t, t_default);
    ros::param::param<std::vector<float>>("~liom/extrinsics/baselink2imu/R", baselink2imu_R, R_default);
    this->extrinsics.baselink2imu.t =
        Eigen::Vector3f(baselink2imu_t[0], baselink2imu_t[1], baselink2imu_t[2]);
    this->extrinsics.baselink2imu.R =
        Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(baselink2imu_R.data(), 3, 3);

    this->extrinsics.baselink2imu_T = Eigen::Matrix4f::Identity();
    this->extrinsics.baselink2imu_T.block(0, 3, 3, 1) = this->extrinsics.baselink2imu.t;
    this->extrinsics.baselink2imu_T.block(0, 0, 3, 3) = this->extrinsics.baselink2imu.R;

    // center of gravity to lidar
    std::vector<float> baselink2lidar_t, baselink2lidar_R;
    ros::param::param<std::vector<float>>("~liom/extrinsics/baselink2lidar/t", baselink2lidar_t, t_default);
    ros::param::param<std::vector<float>>("~liom/extrinsics/baselink2lidar/R", baselink2lidar_R, R_default);

    this->extrinsics.baselink2lidar.t =
        Eigen::Vector3f(baselink2lidar_t[0], baselink2lidar_t[1], baselink2lidar_t[2]);
    this->extrinsics.baselink2lidar.R =
        Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(baselink2lidar_R.data(), 3, 3);

    this->extrinsics.baselink2lidar_T = Eigen::Matrix4f::Identity();
    this->extrinsics.baselink2lidar_T.block(0, 3, 3, 1) = this->extrinsics.baselink2lidar.t;
    this->extrinsics.baselink2lidar_T.block(0, 0, 3, 3) = this->extrinsics.baselink2lidar.R;

    // IMU
    ros::param::param<bool>("~liom/odom/imu/calibration/accel", this->calibrate_accel_, true);
    ros::param::param<bool>("~liom/odom/imu/calibration/gyro", this->calibrate_gyro_, true);
    ros::param::param<double>("~liom/odom/imu/calibration/time", this->imu_calib_time_, 3.0);
    ros::param::param<int>("~liom/odom/imu/bufferSize", this->imu_buffer_size_, 2000);

    std::vector<float> accel_default{0., 0., 0.}; std::vector<float> prior_accel_bias;
    std::vector<float> gyro_default{0., 0., 0.}; std::vector<float> prior_gyro_bias;

    ros::param::param<bool>("~liom/odom/imu/approximateGravity", this->gravity_align_, true);
    ros::param::param<bool>("~liom/imu/calibration", this->imu_calibrate_, true);
    ros::param::param<std::vector<float>>("~liom/imu/intrinsics/accel/bias", prior_accel_bias, accel_default);
    ros::param::param<std::vector<float>>("~liom/imu/intrinsics/gyro/bias", prior_gyro_bias, gyro_default);

    // scale-misalignment matrix
    std::vector<float> imu_sm_default{1., 0., 0., 0., 1., 0., 0., 0., 1.};
    std::vector<float> imu_sm;

    ros::param::param<std::vector<float>>("~liom/imu/intrinsics/accel/sm", imu_sm, imu_sm_default);

    if (!this->imu_calibrate_) {
        this->state.b.accel[0] = prior_accel_bias[0];
        this->state.b.accel[1] = prior_accel_bias[1];
        this->state.b.accel[2] = prior_accel_bias[2];
        this->state.b.gyro[0] = prior_gyro_bias[0];
        this->state.b.gyro[1] = prior_gyro_bias[1];
        this->state.b.gyro[2] = prior_gyro_bias[2];
        this->imu_accel_sm_ = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(imu_sm.data(), 3, 3);
    } else {
        this->state.b.accel = Eigen::Vector3f(0., 0., 0.);
        this->state.b.gyro = Eigen::Vector3f(0., 0., 0.);
        this->imu_accel_sm_ = Eigen::Matrix3f::Identity();
    }

    // GICP
    ros::param::param<int>("~liom/odom/gicp/minNumPoints", this->gicp_min_num_points_, 100);
    ros::param::param<int>("~liom/odom/gicp/kCorrespondences", this->gicp_k_correspondences_, 20);
    ros::param::param<double>("~liom/odom/gicp/maxCorrespondenceDistance", this->gicp_max_corr_dist_,
        std::sqrt(std::numeric_limits<double>::max()));
    ros::param::param<int>("~liom/odom/gicp/maxIterations", this->gicp_max_iter_, 64);
    ros::param::param<double>("~liom/odom/gicp/transformationEpsilon", this->gicp_transformation_ep_, 0.0005);
    ros::param::param<double>("~liom/odom/gicp/rotationEpsilon", this->gicp_rotation_ep_, 0.0005);
    ros::param::param<double>("~liom/odom/gicp/initLambdaFactor", this->gicp_init_lambda_factor_, 1e-9);

    // Geometric Observer
    ros::param::param<double>("~liom/odom/geo/Kp", this->geo_Kp_, 1.0);
    ros::param::param<double>("~liom/odom/geo/Kv", this->geo_Kv_, 1.0);
    ros::param::param<double>("~liom/odom/geo/Kq", this->geo_Kq_, 1.0);
    ros::param::param<double>("~liom/odom/geo/Kab", this->geo_Kab_, 1.0);
    ros::param::param<double>("~liom/odom/geo/Kgb", this->geo_Kgb_, 1.0);
    ros::param::param<double>("~liom/odom/geo/abias_max", this->geo_abias_max_, 1.0);
    ros::param::param<double>("~liom/odom/geo/gbias_max", this->geo_gbias_max_, 1.0);
}


void liom::NodeOdometry::start() {

    ROS_INFO("Starting Odometry Node");
}

void liom::NodeOdometry::publishPose(const ros::TimerEvent& e) {

  // nav_msgs::Odometry
  this->odom_ros.header.stamp = this->imu_stamp;
  this->odom_ros.header.frame_id = this->odom_frame;
  this->odom_ros.child_frame_id = this->baselink_frame;

  this->odom_ros.pose.pose.position.x = this->state.p[0];
  this->odom_ros.pose.pose.position.y = this->state.p[1];
  this->odom_ros.pose.pose.position.z = this->state.p[2];

  this->odom_ros.pose.pose.orientation.w = this->state.q.w();
  this->odom_ros.pose.pose.orientation.x = this->state.q.x();
  this->odom_ros.pose.pose.orientation.y = this->state.q.y();
  this->odom_ros.pose.pose.orientation.z = this->state.q.z();

  this->odom_ros.twist.twist.linear.x = this->state.v.linear.w[0];
  this->odom_ros.twist.twist.linear.y = this->state.v.linear.w[1];
  this->odom_ros.twist.twist.linear.z = this->state.v.linear.w[2];

  this->odom_ros.twist.twist.angular.x = this->state.v.angl.b[0];
  this->odom_ros.twist.twist.angular.y = this->state.v.angl.b[1];
  this->odom_ros.twist.twist.angular.z = this->state.v.angl.b[2];

  this->odom_pub.publish(this->odom_ros);

  // geometry_msgs::PoseStamped
  this->pose_ros.header.stamp = this->imu_stamp;
  this->pose_ros.header.frame_id = this->odom_frame;

  this->pose_ros.pose.position.x = this->state.p[0];
  this->pose_ros.pose.position.y = this->state.p[1];
  this->pose_ros.pose.position.z = this->state.p[2];

  this->pose_ros.pose.orientation.w = this->state.q.w();
  this->pose_ros.pose.orientation.x = this->state.q.x();
  this->pose_ros.pose.orientation.y = this->state.q.y();
  this->pose_ros.pose.orientation.z = this->state.q.z();

  this->pose_pub.publish(this->pose_ros);

}



/**
    Transform IMU function
**/
sensor_msgs::Imu::Ptr liom::NodeOdometry::transformImu(const sensor_msgs::Imu::ConstPtr& imu_raw) {

    sensor_msgs::Imu::Ptr imu (new sensor_msgs::Imu);

    // Copy header
    imu->header = imu_raw->header;

    static double prev_stamp = imu->header.stamp.toSec();
    double dt = imu->header.stamp.toSec() - prev_stamp;
    prev_stamp = imu->header.stamp.toSec();

    if (dt == 0) { dt = 1.0/200.0;}                     // dt: ~= 0.005s (between 2 IMU timestamps)

    // Transform angular velocity (will be the same on a rigid body, so just rotate to ROS convention) 
    Eigen::Vector3f angl_vel(imu_raw->angular_velocity.x,
                            imu_raw->angular_velocity.y,
                            imu_raw->angular_velocity.z);
    
    Eigen::Vector3f angl_vel_cg = this->extrinsics.baselink2imu.R * angl_vel;  

    imu->angular_velocity.x = angl_vel_cg[0];
    imu->angular_velocity.y = angl_vel_cg[1];
    imu->angular_velocity.z = angl_vel_cg[2];

    static Eigen::Vector3f angl_vel_cg_prev = angl_vel_cg;

    // Transform linear acceleration (need to account for component due to translational difference)
    Eigen::Vector3f linear_accel(imu_raw->linear_acceleration.x,
                                 imu_raw->linear_acceleration.y,
                                 imu_raw->linear_acceleration.z);

    Eigen::Vector3f linear_accel_cg = this->extrinsics.baselink2imu.R * linear_accel;


    linear_accel_cg = linear_accel_cg
                      + ((angl_vel_cg - angl_vel_cg_prev)/ dt).cross(-this->extrinsics.baselink2imu.t)
                      + angl_vel_cg.cross(angl_vel_cg.cross(-this->extrinsics.baselink2imu.t));

    angl_vel_cg_prev = angl_vel_cg;

    imu->linear_acceleration.x = linear_accel_cg[0];
    imu->linear_acceleration.y = linear_accel_cg[1];
    imu->linear_acceleration.z = linear_accel_cg[2];

    return imu;


}

void liom::NodeOdometry::propagateState() {

  // Lock thread to prevent state from being accessed by UpdateState
  std::lock_guard<std::mutex> lock( this->geo.mtx );

  double dt = this->imu_meas.dt;

  Eigen::Quaternionf q_hat = this->state.q, omega;
  Eigen::Vector3f world_accel;

  // Transform accel from body to world frame
  world_accel = q_hat._transformVector(this->imu_meas.linear_accel);

  // Accel propogation
  this->state.p[0] += this->state.v.linear.w[0]*dt + 0.5*dt*dt*world_accel[0];
  this->state.p[1] += this->state.v.linear.w[1]*dt + 0.5*dt*dt*world_accel[1];
  this->state.p[2] += this->state.v.linear.w[2]*dt + 0.5*dt*dt*(world_accel[2] - this->gravity_);

  this->state.v.linear.w[0] += world_accel[0]*dt;
  this->state.v.linear.w[1] += world_accel[1]*dt;
  this->state.v.linear.w[2] += (world_accel[2] - this->gravity_)*dt;
  this->state.v.linear.b = this->state.q.toRotationMatrix().inverse() * this->state.v.linear.w;

  // Gyro propogation
  omega.w() = 0;
  omega.vec() = this->imu_meas.angl_vel;
  Eigen::Quaternionf tmp = q_hat * omega;
  this->state.q.w() += 0.5 * dt * tmp.w();
  this->state.q.vec() += 0.5 * dt * tmp.vec();

  // Ensure quaternion is properly normalized
  this->state.q.normalize();

  this->state.v.angl.b = this->imu_meas.angl_vel;
  this->state.v.angl.w = this->state.q.toRotationMatrix() * this->state.v.angl.b;   

  // Modification
  this->accel_total += world_accel;
  this->angl_vec_total += this->imu_meas.angl_vel;
  this->num_imu++;
}





/**
    IMU CallBack Function
**/

void liom::NodeOdometry::callbackImu(const sensor_msgs::Imu::ConstPtr& imu_raw) {

    // when the first IMU arrive
    this->first_imu_received = true;

    // Transform raw IMU from IMU to baselink coordinate
    sensor_msgs::Imu::Ptr imu = this->transformImu(imu_raw);
    this->imu_stamp  = imu->header.stamp;

    Eigen::Vector3f linear_accel;
    Eigen::Vector3f angl_vel;

    // Get IMU samples
    angl_vel[0] = imu->angular_velocity.x;
    angl_vel[1] = imu->angular_velocity.y;
    angl_vel[2] = imu->angular_velocity.z;

    linear_accel[0]  = imu->linear_acceleration.x;
    linear_accel[1]  = imu->linear_acceleration.y;
    linear_accel[2]  = imu->linear_acceleration.z;

    if (this->first_imu_stamp == 0) {
        this->first_imu_stamp = imu->header.stamp.toSec();
    }

    // IMU calibration procedure  --- Do for three seconds
    if (!this->imu_calibrated) {

        static int num_samples = 0;
        static Eigen::Vector3f gyro_avg (0, 0, 0);
        static Eigen::Vector3f accel_avg (0, 0, 0);
        static bool print = true;

        if ((imu->header.stamp.toSec() - this->first_imu_stamp) < this->imu_calib_time_) {

            num_samples++;

            gyro_avg[0] += angl_vel[0];
            gyro_avg[1] += angl_vel[1];
            gyro_avg[2] += angl_vel[2];

            accel_avg[0] += linear_accel[0];
            accel_avg[1] += linear_accel[1];
            accel_avg[2] += linear_accel[2];

            if (print) {
                std::cout << std::endl << " Calibrating IMU for " << this->imu_calib_time_ << " seconds ... ";
                std::cout.flush();
                print = false;
            }
        } else {

            std::cout << " done " << std::endl << std::endl;    // Print done calibration

            gyro_avg  /= num_samples;
            accel_avg /= num_samples;

            std::cout << "Number of samples ==> " << num_samples << std::endl;

            Eigen::Vector3f grav_vec(0, 0, this->gravity_);

            if (this->gravity_align_) {

                // Estimate gravity vector - Only approximate if biases have not been pre-calibrated
                grav_vec = (accel_avg - this->state.b.accel).normalized() * abs(this->gravity_);
                Eigen::Quaternionf grav_q = Eigen::Quaternionf::FromTwoVectors(grav_vec, Eigen::Vector3f(0, 0, this->gravity_));

                // Set gravity aligned orientation
                this->state.q = grav_q;
                this->T.block(0,0,3,3) = this->state.q.toRotationMatrix();
                this->lidarPose.q = this->state.q;

                //RPY
                auto euler    = grav_q.toRotationMatrix().eulerAngles(2, 1, 0);
                double yaw    = euler[0] * (180/M_PI);
                double pitch  = euler[1] * (180/M_PI);
                double roll   = euler[2] * (180/M_PI);

                // use alternate representation if the yaw is smaller
                if (abs(remainder(yaw + 180.0, 360.0)) < abs(yaw)) {
                yaw   = remainder(yaw + 180.0,   360.0);
                pitch = remainder(180.0 - pitch, 360.0);
                roll  = remainder(roll + 180.0,  360.0);
                }

                std::cout << " Estimated initial attitude:" << std::endl;
                std::cout << "   Roll  [deg]: " << to_string_with_precision(roll, 4) << std::endl;
                std::cout << "   Pitch [deg]: " << to_string_with_precision(pitch, 4) << std::endl;
                std::cout << "   Yaw   [deg]: " << to_string_with_precision(yaw, 4) << std::endl;
                std::cout << std::endl;
            }

            if (this->calibrate_accel_) {

            // subtract gravity from avg accel to get bias
            this->state.b.accel = accel_avg - grav_vec;

            std::cout << " Accel biases [xyz]: " << to_string_with_precision(this->state.b.accel[0], 8) << ", "
                                                << to_string_with_precision(this->state.b.accel[1], 8) << ", "
                                                << to_string_with_precision(this->state.b.accel[2], 8) << std::endl;
            }

            if (this->calibrate_gyro_) {

            this->state.b.gyro = gyro_avg;

            std::cout << " Gyro biases  [xyz]: " << to_string_with_precision(this->state.b.gyro[0], 8) << ", "
                                                << to_string_with_precision(this->state.b.gyro[1], 8) << ", "
                                                << to_string_with_precision(this->state.b.gyro[2], 8) << std::endl;
            }

            this->imu_calibrated = true;

        }

    } else {

        // If calibration done
        double dt = imu->header.stamp.toSec() - this->prev_imu_stamp;
        if (dt == 0) { dt = 1.0/200.0;}
        this->imu_rates.push_back(1/dt);

        // Apply the calibrated bias to the new IMU measurements
        this->imu_meas.stamp = imu->header.stamp.toSec();
        this->imu_meas.dt    = dt;
        this->prev_imu_stamp = this->imu_meas.stamp;

        // Correct the measurement
        Eigen::Vector3f linear_accel_corrected = linear_accel;
        Eigen::Vector3f angl_vel_corrected     = angl_vel;

        this->imu_meas.linear_accel = linear_accel_corrected;
        this->imu_meas.angl_vel     = angl_vel_corrected;

        // Store calibrated IMU measurements into imu buffer for manual integration later.
        this->mtx_imu.lock();
        this->imu_buffer.push_front(this->imu_meas);
        this->mtx_imu.unlock();

        // Notify the callbackPointCloud thread that IMU data exists for this time
        this->cv_imu_stamp.notify_one();

        if (this->geo.first_opt_done) {
            // Geometric Observer: Propagate State
            this->propagateState();

            // Modification
            // Initialize the first state
            if (this->state_initialized) {
                Eigen::Vector3f prevPose = this->state.p;
                Eigen::Vector3f prevVel  = this->state.v.linear.w;
                this->x_init << prevPose, prevVel;
                this->imm_initialized = true;
            }
            
            this->state_initialized = false;
        }        
    }

}

/**
    Initialize function
**/
void liom::NodeOdometry::initializeLIOM() {

    // Wait for IMU
    if (!this->first_imu_received || !this->imu_calibrated) {
        return;
    }

    // If IMU recieved and calibrated
    this->liom_initialized = true;
}

/**
   Conver Scan in ROS
**/
void liom::NodeOdometry::getScanFromROS(const sensor_msgs::PointCloud2ConstPtr& pc) {

    // Denote original scan
    pcl::PointCloud<PointType>::Ptr original_scan_ (boost::make_shared<pcl::PointCloud<PointType>>());
    pcl::fromROSMsg(*pc, *original_scan_);

    // Remove NaN points
    std::vector<int> idx;                       //  scan points index
    original_scan_ -> is_dense = false;
    pcl::removeNaNFromPointCloud(*original_scan_, *original_scan_, idx);

    // Crop Box Filter
    this->crop.setInputCloud(original_scan_);           // Provide a pointer to the input dataset
    this->crop.filter(*original_scan_);                 // Call a filtering method 

    // Automatically detect sensor type
    this->sensor = liom::SensorType::UNKNOWN;
    for (auto &field : pc->fields) {
        if (field.name == "t") {
            this->sensor = liom::SensorType::OUSTER;
            break;
        } else if (field.name == "time") {
            this->sensor = liom::SensorType::VELODYNE;
            break;
        } else if (field.name == "timestamp" && original_scan_ ->points[0].timestamp < 1e14) {
            this->sensor = liom::SensorType::HESAI;
            break;
        } else if (field.name == "timestamp" && original_scan_ ->points[0].timestamp > 1e14) {
            this->sensor = liom::SensorType::LIVOX;
            break;
        }
    }

    if (this->sensor == liom::SensorType::UNKNOWN) {
        this->deskew_ = false;
    }

    this->scan_header_stamp = pc->header.stamp;
    this->original_scan = original_scan_;
}

/**
    Check valid IMU function
**/
bool liom::NodeOdometry::imuMeasFromTimeRange(double start_time, double end_time,
                                              boost::circular_buffer<ImuMeas>::reverse_iterator& begin_imu_it,
                                              boost::circular_buffer<ImuMeas>::reverse_iterator& end_imu_it) {

    if (this->imu_buffer.empty() || this->imu_buffer.front().stamp < end_time) {
        // Wait for the latest IMU data
        std::unique_lock<decltype(this->mtx_imu)> lock(this->mtx_imu);
        this->cv_imu_stamp.wait(lock, [this, &end_time]{ return this->imu_buffer.front().stamp >= end_time; });
    }

    auto imu_it = this->imu_buffer.begin();

    auto last_imu_it = imu_it;
    imu_it++;

    while (imu_it != this->imu_buffer.end() && imu_it->stamp >= end_time) {
    last_imu_it = imu_it;
    imu_it++;
    }

    while (imu_it != this->imu_buffer.end() && imu_it->stamp >= start_time) {
    imu_it++;
    }

    if (imu_it == this->imu_buffer.end()) {
        // not enough IMU measurements, return false
        return false;
    }
    imu_it++;

    // Set reverse iterators (to iterate forward in time)
    end_imu_it = boost::circular_buffer<ImuMeas>::reverse_iterator(last_imu_it);
    begin_imu_it = boost::circular_buffer<ImuMeas>::reverse_iterator(imu_it);

    return true;

}



/**
    Integrate IMU function
**/
std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
liom::NodeOdometry::integrateImu(double start_time, Eigen::Quaternionf q_init, Eigen::Vector3f p_init,
                                Eigen::Vector3f v_init, const std::vector<double>& sorted_timestamps) {

    // ROS_WARN("Intergrate_IMU ");
    const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> empty;
    
    if (sorted_timestamps.empty() || start_time > sorted_timestamps.front()) {
        // Invalid input, return empty vector
        return empty;
    }

    boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it;             // Begin IMU in the sweep
    boost::circular_buffer<ImuMeas>::reverse_iterator end_imu_it;               // End IMU in the sweep
    if (this->imuMeasFromTimeRange(start_time, sorted_timestamps.back(), begin_imu_it, end_imu_it) == false) {
    // not enough IMU measurements, return empty vector
    return empty;
    }

    // Backwards integration to find pose at first IMU sample
    const ImuMeas& f1 = *begin_imu_it;
    const ImuMeas& f2 = *(begin_imu_it+1);;

    // Time betweeen first two IMU samples
    double dt = f2.dt;

    // Time between first IMU sample and start_time
    double idt = start_time - f1.stamp;

    // Angular acceleration between first two IMU samples
    Eigen::Vector3f alpha_dt = f2.angl_vel - f1.angl_vel;
    Eigen::Vector3f alpha    = alpha_dt/dt;

    // Average angular velocity (reversed) between first IMU sample and start_time
    Eigen::Vector3f omega_i = -(f1.angl_vel + 0.5*alpha*idt);

    // Set q_init to orientation at first IMU samples
    q_init = Eigen::Quaternionf (
        q_init.w() - 0.5*( q_init.x()*omega_i[0] + q_init.y()*omega_i[1] + q_init.z()*omega_i[2] ) * idt,
        q_init.x() + 0.5*( q_init.w()*omega_i[0] - q_init.z()*omega_i[1] + q_init.y()*omega_i[2] ) * idt,
        q_init.y() + 0.5*( q_init.z()*omega_i[0] + q_init.w()*omega_i[1] - q_init.x()*omega_i[2] ) * idt,
        q_init.z() + 0.5*( q_init.x()*omega_i[1] - q_init.y()*omega_i[0] + q_init.w()*omega_i[2] ) * idt
    );

    q_init.normalize();

    // Average angular velocity between first two IMU samples
    Eigen::Vector3f omega = f1.angl_vel + 0.5*alpha_dt;

    // Orientation at second IMU sample
    Eigen::Quaternionf q2 (
        q_init.w() - 0.5*( q_init.x()*omega[0] + q_init.y()*omega[1] + q_init.z()*omega[2] ) * dt,
        q_init.x() + 0.5*( q_init.w()*omega[0] - q_init.z()*omega[1] + q_init.y()*omega[2] ) * dt,
        q_init.y() + 0.5*( q_init.z()*omega[0] + q_init.w()*omega[1] - q_init.x()*omega[2] ) * dt,
        q_init.z() + 0.5*( q_init.x()*omega[1] - q_init.y()*omega[0] + q_init.w()*omega[2] ) * dt
    );
    q2.normalize();   

    // Acceleration at first IMU sample
    Eigen::Vector3f a1 = q_init._transformVector(f1.linear_accel);
    a1[2] -= this->gravity_;

    // Acceleration at second IMU sample
    Eigen::Vector3f a2 = q2._transformVector(f2.linear_accel);
    a2[2] -= this->gravity_;

    // Jerk between first two IMU samples
    Eigen::Vector3f j = (a2 - a1) / dt;

    // Set v_init to velocity at first IMU sample (go backwards from start_time)
    v_init -= a1*idt + 0.5*j*idt*idt;

    // Set p_init to position at first IMU sample (go backwards from start_time)
    p_init -= v_init*idt + 0.5*a1*idt*idt + (1/6.)*j*idt*idt*idt;

    return this->integrateImuInternal(q_init, p_init, v_init, sorted_timestamps, begin_imu_it, end_imu_it);
}

std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
liom::NodeOdometry::integrateImuInternal(Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
                                     const std::vector<double>& sorted_timestamps,
                                     boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it,
                                     boost::circular_buffer<ImuMeas>::reverse_iterator end_imu_it) {

    // Denote imu vector
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> imu_se3;

    // Initialization
    Eigen::Quaternionf q = q_init;
    Eigen::Vector3f p = p_init;
    Eigen::Vector3f v = v_init;
    Eigen::Vector3f a = q._transformVector(begin_imu_it->linear_accel);
    a[2] -= this->gravity_;

    // Iterate over IMU measurements and timestamps
    auto prev_imu_it = begin_imu_it;
    auto imu_it = prev_imu_it + 1;

    auto stamp_it = sorted_timestamps.begin();

    double count = 0;

    for (; imu_it != end_imu_it; imu_it++) {

        const ImuMeas& f0 = *prev_imu_it;
        const ImuMeas& f = *imu_it;

        // Time between IMU samples
        double dt = f.dt;

        // Angular acceleration
        Eigen::Vector3f alpha_dt = f.angl_vel - f0.angl_vel;
        Eigen::Vector3f alpha = alpha_dt / dt;

        // Average angular velocity
        Eigen::Vector3f omega = f0.angl_vel + 0.5*alpha_dt;

        // Orientation
        q = Eigen::Quaternionf (
        q.w() - 0.5*( q.x()*omega[0] + q.y()*omega[1] + q.z()*omega[2] ) * dt,
        q.x() + 0.5*( q.w()*omega[0] - q.z()*omega[1] + q.y()*omega[2] ) * dt,
        q.y() + 0.5*( q.z()*omega[0] + q.w()*omega[1] - q.x()*omega[2] ) * dt,
        q.z() + 0.5*( q.x()*omega[1] - q.y()*omega[0] + q.w()*omega[2] ) * dt
        );
        q.normalize();

        // Acceleration
        Eigen::Vector3f a0 = a;
        a = q._transformVector(f.linear_accel);
        a[2] -= this->gravity_;

        // Jerk
        Eigen::Vector3f j_dt = a - a0;
        Eigen::Vector3f j = j_dt / dt;

        // Interpolate for given timestamps
        while (stamp_it != sorted_timestamps.end() && *stamp_it <= f.stamp) {
        // Time between previous IMU sample and given timestamp
        double idt = *stamp_it - f0.stamp;

        // Average angular velocity
        Eigen::Vector3f omega_i = f0.angl_vel + 0.5*alpha*idt;

        // Orientation
        Eigen::Quaternionf q_i (
            q.w() - 0.5*( q.x()*omega_i[0] + q.y()*omega_i[1] + q.z()*omega_i[2] ) * idt,
            q.x() + 0.5*( q.w()*omega_i[0] - q.z()*omega_i[1] + q.y()*omega_i[2] ) * idt,
            q.y() + 0.5*( q.z()*omega_i[0] + q.w()*omega_i[1] - q.x()*omega_i[2] ) * idt,
            q.z() + 0.5*( q.x()*omega_i[1] - q.y()*omega_i[0] + q.w()*omega_i[2] ) * idt
        );
        q_i.normalize();

        // Position
        Eigen::Vector3f p_i = p + v*idt + 0.5*a0*idt*idt + (1/6.)*j*idt*idt*idt;

        // Transformation
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        T.block(0, 0, 3, 3) = q_i.toRotationMatrix();
        T.block(0, 3, 3, 1) = p_i;

        imu_se3.push_back(T);

        stamp_it++;
        }
        // Position
        p += v*dt + 0.5*a0*dt*dt + (1/6.)*j_dt*dt*dt;

        // Velocity
        v += a0*dt + 0.5*j_dt*dt;

        prev_imu_it = imu_it;

        count++;
    }

    return imu_se3;
}


/**
    De-skew point cloud function
**/
void liom::NodeOdometry::deskewPointcloud() {

    // Denote the de-skew scan
    pcl::PointCloud<PointType>::Ptr deskewed_scan_ (boost::make_shared<pcl::PointCloud<PointType>>());
    deskewed_scan_ ->points.resize(this->original_scan->points.size());         // Re-size the deskew scan as original scan

    // Individual point timestamps should be relative to this time
    double sweep_ref_time = this->scan_header_stamp.toSec();

    // Sort points by timestamps and build list of timestamps
    std::function<bool(const PointType&, const PointType&)> point_time_cmp;
    std::function<bool(boost::range::index_value<PointType&, long>,
                        boost::range::index_value<PointType&, long>)> point_time_neq;
    std::function<double(boost::range::index_value<PointType&, long>)> extract_point_time;

    if (this->sensor == liom::SensorType::OUSTER) {

        point_time_cmp = [](const PointType& p1, const PointType& p2)
        { return p1.t < p2.t; };
        point_time_neq = [](boost::range::index_value<PointType&, long> p1,
                            boost::range::index_value<PointType&, long> p2)
        { return p1.value().t != p2.value().t; };
        extract_point_time = [&sweep_ref_time](boost::range::index_value<PointType&, long> pt)
        { return sweep_ref_time + pt.value().t * 1e-9f; };

    } else if (this->sensor == liom::SensorType::VELODYNE) {

        point_time_cmp = [](const PointType& p1, const PointType& p2)
        { return p1.time < p2.time; };
        point_time_neq = [](boost::range::index_value<PointType&, long> p1,
                            boost::range::index_value<PointType&, long> p2)
        { return p1.value().time != p2.value().time; };
        extract_point_time = [&sweep_ref_time](boost::range::index_value<PointType&, long> pt)
        { return sweep_ref_time + pt.value().time; };

    } else if (this->sensor == liom::SensorType::HESAI) {

        point_time_cmp = [](const PointType& p1, const PointType& p2)
        { return p1.timestamp < p2.timestamp; };
        point_time_neq = [](boost::range::index_value<PointType&, long> p1,
                            boost::range::index_value<PointType&, long> p2)
        { return p1.value().timestamp != p2.value().timestamp; };
        extract_point_time = [&sweep_ref_time](boost::range::index_value<PointType&, long> pt)
        { return pt.value().timestamp; };

    } else if (this->sensor == liom::SensorType::LIVOX) {
        point_time_cmp = [](const PointType& p1, const PointType& p2)
        { return p1.timestamp < p2.timestamp; };
        point_time_neq = [](boost::range::index_value<PointType&, long> p1,
                            boost::range::index_value<PointType&, long> p2)
        { return p1.value().timestamp != p2.value().timestamp; };
        extract_point_time = [&sweep_ref_time](boost::range::index_value<PointType&, long> pt)
        { return pt.value().timestamp * 1e-9f; };
    }

        
    // copy points into deskewed_scan_ in order of timestamp
    std::partial_sort_copy(this->original_scan->points.begin(), this->original_scan->points.end(),
                            deskewed_scan_->points.begin(), deskewed_scan_->points.end(), point_time_cmp);

    // filter unique timestamps
    auto points_unique_timestamps = deskewed_scan_->points
                                    | boost::adaptors::indexed()
                                    | boost::adaptors::adjacent_filtered(point_time_neq);

    // extract timestamps from points and put them in their own list
    std::vector<double> timestamps;
    std::vector<int> unique_time_indices;

    // compute offset between sweep reference time and first point timestamp
    double offset = 0.0;
    if (this->time_offset_) {
        offset = sweep_ref_time - extract_point_time(*points_unique_timestamps.begin());
    }

    // build list of unique timestamps and indices of first point with each timestamp
    for (auto it = points_unique_timestamps.begin(); it != points_unique_timestamps.end(); it++) {
        timestamps.push_back(extract_point_time(*it) + offset);
        unique_time_indices.push_back(it->index());
    }
    unique_time_indices.push_back(deskewed_scan_->points.size());

    int median_pt_index = timestamps.size() / 2;            // Median point index
    // std::cout << " Median pt index ==> " << median_pt_index << std::endl;
    this->scan_stamp = timestamps[median_pt_index]; // set this->scan_stamp to the timestamp of the median point
    // std::cout << " Scan stamp ==> " << this->scan_stamp << std::endl;

    // don't process scans until IMU data is present
    if (!this->first_valid_scan) {
        ROS_WARN (" Process scan when IMU is present");
        if (this->imu_buffer.empty() || this->scan_stamp <= this->imu_buffer.back().stamp) {
        return;
        }

        this->first_valid_scan = true;
        this->T_prior = this->T; // assume no motion for the first scan
        pcl::transformPointCloud (*deskewed_scan_, *deskewed_scan_, this->T_prior * this->extrinsics.baselink2lidar_T);
        this->deskewed_scan = deskewed_scan_;
        this->deskew_status = true;
        return;
    }

    // IMU prior & deskewing for second scan onwards
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> frames;  // denote frame between start and end of the sweep

    frames = this->integrateImu(this->prev_scan_stamp, this->state.q, this->state.p,
                            this->state.v.linear.w, timestamps);
    this->deskew_size = frames.size(); // if integration successful, equal to timestamps.size()

    // if there are no frames between the start and end of the sweep
    // that probably means that there's a sync issue
    if (frames.size() != timestamps.size()) {
        ROS_FATAL("Bad time sync between LiDAR and IMU!");

        this->T_prior = this->T;
        pcl::transformPointCloud (*deskewed_scan_, *deskewed_scan_, this->T_prior * this->extrinsics.baselink2lidar_T);
        this->deskewed_scan = deskewed_scan_;
        this->deskew_status = false;
        return;
    }

    // update prior to be the estimated pose at the median time of the scan (corresponds to this->scan_stamp)
    this->T_prior = frames[median_pt_index];

    #pragma omp parallel for num_threads(this->num_threads_)
    for (int i = 0; i < timestamps.size(); i++) {

        Eigen::Matrix4f T = frames[i] * this->extrinsics.baselink2lidar_T;

        // transform point to world frame
        for (int k = unique_time_indices[i]; k < unique_time_indices[i+1]; k++) {
        auto &pt = deskewed_scan_->points[k];
        pt.getVector4fMap()[3] = 1.;
        pt.getVector4fMap() = T * pt.getVector4fMap();
        }
    }

    this->deskewed_scan = deskewed_scan_;
    this->deskew_status = true;
}

/**
    Pre-processing points
**/
void liom::NodeOdometry::preprocessPoints() {

    // Deskew the original scan
    if (this->deskew_) {

        // ROS_WARN(" De-skew part ");
        this->deskewPointcloud();

        if (!this->first_valid_scan) {
            return;
        }
    } else {

        ROS_WARN (" If the Point cloud already de-skewed");
        this->scan_stamp = this->scan_header_stamp.toSec();

        // don't process scans until IMU data is present
        if (!this->first_valid_scan) {
            if (this->imu_buffer.empty() || this->scan_stamp <= this->imu_buffer.back().stamp) {
                return;
            }

            this->first_valid_scan = true;
            this->T_prior = this->T;                // Assume no motion for the first scab
        } else {
            // IMU prior for second scan onwards
            std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> frames;
            frames = this->integrateImu(this->prev_scan_stamp, this->state.q, this->state.p,
                    this->geo.prev_vel.cast<float>(), {this->scan_stamp});

            if (frames.size() > 0) {
                this->T_prior = frames.back();
            } else {
                this->T_prior = this->T;
            }
        }

        pcl::PointCloud<PointType>::Ptr deskewed_scan_ (boost::make_shared<pcl::PointCloud<PointType>>());
        pcl::transformPointCloud (*this->original_scan, *deskewed_scan_,
                                this->T_prior * this->extrinsics.baselink2lidar_T);
        this->deskewed_scan = deskewed_scan_;
        this->deskew_status = false;
    }

    // Voxel Grid Filter
    if (this->vf_use_) {
        // ROS_WARN("Voxel Filter ");
        pcl::PointCloud<PointType>::Ptr current_scan_
            (boost::make_shared<pcl::PointCloud<PointType>>(*this->deskewed_scan));
        this->voxel.setInputCloud(current_scan_);
        this->voxel.filter(*current_scan_);
        this->current_scan = current_scan_;
    } else {
        this->current_scan = this->deskewed_scan;
    }
}

/**
    Compute Metrics function
 */
void liom::NodeOdometry::computeMetrics() {
    //
    this->computeSpaciousness();
    this->computeDensity();
}

/**
    Compute Spaciousness function
*/
void liom::NodeOdometry::computeSpaciousness() {

    // Denote range of points
    std::vector<float> ds;

     // Compute range of all points
    for (int i = 0; i <= this->original_scan->points.size(); i++){
        float d = std::sqrt(pow(this->original_scan->points[i].x, 2) +
                             pow(this->original_scan->points[i].y, 2));
        
        // Sort ranges to the list
        ds.push_back(d);
    }

    // Median
    std::nth_element(ds.begin(), ds.begin() + ds.size()/2, ds.end());
    float median_curr = ds[ds.size()/2];
    static float median_prev = median_curr;
    float median_lpf = 0.95*median_prev + 0.05*median_curr;
    median_prev = median_lpf;

    // push back 
    this->metrics.spaciousness.push_back( median_lpf );
}


/**
    Compute Density function
*/ 
void liom::NodeOdometry::computeDensity() {

    float density;

    if (!this->geo.first_opt_done) {
        density = 0;
    } else {
        density = this->gicp.source_density_;
    }

    static float density_prev = density;
    float density_lpf  = 0.95*density_prev + 0.05*density;
    density_prev = density_lpf;

    this->metrics.density.push_back( density_lpf );
}

/**
    Set Adaptive Parameters
*/
void liom::NodeOdometry::setAdaptiveParams() {

    // Spaciousness
    float sp = this->metrics.spaciousness.back();

    if (sp < 0.5) { sp = 0.5;}
    if (sp > 5.0) { sp = 5.0;}                  // Limit value of spaciousness 0.5 <= sp <= 5.0

    this->keyframe_thresh_dist_ = sp;           // Threshold distance

    // Density
    float den = this->metrics.density.back();

    if (den < 0.5*this->gicp_max_corr_dist_) { den = 0.5*this->gicp_max_corr_dist_; }
    if (den > 2.0*this->gicp_max_corr_dist_) { den = 2.0*this->gicp_max_corr_dist_; }

    if (sp < 5.0) { den = 0.5*this->gicp_max_corr_dist_; };
    if (sp > 5.0) { den = 2.0*this->gicp_max_corr_dist_; };

    this->gicp.setMaxCorrespondenceDistance(den);

    // Concave hull alpha
    this->concave_hull.setAlpha(this->keyframe_thresh_dist_);
}


/**
    Set Input Source function
*/
void liom::NodeOdometry::setInputSource() {
    this->gicp.setInputSource(this->current_scan);
    this->gicp.calculateSourceCovariances();
}

/**
    Initialization Input Target function
*/
void liom::NodeOdometry::initializeInputTarget() {

    // Update the scan stamp
    this->prev_scan_stamp = this->scan_stamp;

    // Keep history of keyframes
    this->keyframes.push_back(std::make_pair(std::make_pair(this->state.p, this->state.q), this->current_scan));
    this->keyframe_timestamps.push_back(this->scan_header_stamp);
    this->keyframe_normals.push_back(this->gicp.getSourceCovariances());
    this->keyframe_transformations.push_back(this->T_corr);
    
    // Modification
    this->cov_initialized = true;
    this->state_initialized = true;
    

}


/**
    Publish Key Frame function
*/
void liom::NodeOdometry::publishKeyframe(std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>, pcl::PointCloud<PointType>::ConstPtr> kf, ros::Time timestamp) {

    // Push back
    geometry_msgs::Pose p;
    p.position.x = kf.first.first[0];
    p.position.y = kf.first.first[1];
    p.position.z = kf.first.first[2];
    p.orientation.w = kf.first.second.w();
    p.orientation.x = kf.first.second.x();
    p.orientation.y = kf.first.second.y();
    p.orientation.z = kf.first.second.z();
    this->kf_pose_ros.poses.push_back(p);

    // Publish
    this->kf_pose_ros.header.stamp = timestamp;
    this->kf_pose_ros.header.frame_id  = this->odom_frame;
    this->kf_pose_pub.publish(this->kf_pose_ros);

    // Publish keyframe scan for map
    if (this->vf_use_) {
        if (kf.second->points.size() == kf.second->width * kf.second->height) {
        sensor_msgs::PointCloud2 keyframe_cloud_ros;
        pcl::toROSMsg(*kf.second, keyframe_cloud_ros);
        keyframe_cloud_ros.header.stamp = timestamp;
        keyframe_cloud_ros.header.frame_id = this->odom_frame;
        this->kf_cloud_pub.publish(keyframe_cloud_ros);
        }
    } else {
        sensor_msgs::PointCloud2 keyframe_cloud_ros;
        pcl::toROSMsg(*kf.second, keyframe_cloud_ros);
        keyframe_cloud_ros.header.stamp = timestamp;
        keyframe_cloud_ros.header.frame_id = this->odom_frame;
        this->kf_cloud_pub.publish(keyframe_cloud_ros);
    }

}


/**
    pushSubmapIndices function
*/
void liom::NodeOdometry::pushSubmapIndices(std::vector<float> dists, int k, std::vector<int> frames) {

    // Make sure dists is not empty 
    if (!dists.size()) {return; }                       // ds

    // Maintain max heal of at most k elements
    std::priority_queue<float> pq;


    for (auto d : dists) {
        if (pq.size() >= k && pq.top() > d) {
        pq.push(d);
        pq.pop();
        } else if (pq.size() < k) {
        pq.push(d);
        }
    }

    // get the kth smallest element, which should be at the top of the heap
    float kth_element = pq.top();

    // get all elements smaller or equal to the kth smallest element
    for (int i = 0; i < dists.size(); ++i) {
        if (dists[i] <= kth_element)
        this->submap_kf_idx_curr.push_back(frames[i]);
    }
}


/**
    Compute convex hull function
*/
void liom::NodeOdometry::computeConcaveHull() {

    // At least 4 keyframes for convex hull
    if (this->num_processed_keyframes < 4) {
        return;
    }

    // Create a pointcloud with points at keyframes
    pcl::PointCloud<PointType>::Ptr cloud = 
        pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());


    std::unique_lock<decltype(this->keyframes_mutex)> lock(this->keyframes_mutex);
    for (int i = 0; i < this->num_processed_keyframes; i++) {
        PointType pt;
        pt.x = this->keyframes[i].first.first[0];
        pt.y = this->keyframes[i].first.first[1];
        pt.z = this->keyframes[i].first.first[2];
        cloud->push_back(pt);
    }
    lock.unlock();


    // calculate the convex hull of the point cloud
    this->convex_hull.setInputCloud(cloud);

    // get the indices of the keyframes on the convex hull
    pcl::PointCloud<PointType>::Ptr convex_points =
        pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());
    this->convex_hull.reconstruct(*convex_points);

    pcl::PointIndices::Ptr convex_hull_point_idx = pcl::PointIndices::Ptr (boost::make_shared<pcl::PointIndices>());
    this->convex_hull.getHullPointIndices(*convex_hull_point_idx);

    this->keyframe_convex.clear();
    for (int i=0; i<convex_hull_point_idx->indices.size(); ++i) {
        this->keyframe_convex.push_back(convex_hull_point_idx->indices[i]);
    }

}


void liom::NodeOdometry::computeConvexHull() {

  // at least 4 keyframes for convex hull
  if (this->num_processed_keyframes < 4) {
    return;
  }

  // create a pointcloud with points at keyframes
  pcl::PointCloud<PointType>::Ptr cloud =
    pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());

  std::unique_lock<decltype(this->keyframes_mutex)> lock(this->keyframes_mutex);
  for (int i = 0; i < this->num_processed_keyframes; i++) {
    PointType pt;
    pt.x = this->keyframes[i].first.first[0];
    pt.y = this->keyframes[i].first.first[1];
    pt.z = this->keyframes[i].first.first[2];
    cloud->push_back(pt);
  }
  lock.unlock();

  // calculate the convex hull of the point cloud
  this->convex_hull.setInputCloud(cloud);

  // get the indices of the keyframes on the convex hull
  pcl::PointCloud<PointType>::Ptr convex_points =
    pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());
  this->convex_hull.reconstruct(*convex_points);

  pcl::PointIndices::Ptr convex_hull_point_idx = pcl::PointIndices::Ptr (boost::make_shared<pcl::PointIndices>());
  this->convex_hull.getHullPointIndices(*convex_hull_point_idx);

  this->keyframe_convex.clear();
  for (int i=0; i<convex_hull_point_idx->indices.size(); ++i) {
    this->keyframe_convex.push_back(convex_hull_point_idx->indices[i]);
  }

}


/**
    Build Submap (local map)
*/
void liom::NodeOdometry::buildSubmap(State vehicle_state) {

    // Clear vector of keyframe indices to use for submap
    this->submap_kf_idx_curr.clear();

    // Calculate distance between current pose and poses in keyframe set
    std::unique_lock<decltype(this->keyframes_mutex)> lock(this->keyframes_mutex);
    std::vector<float> ds;
    std::vector<int> keyframe_nn;

    for (int i = 0; i < this->num_processed_keyframes; i++) {
        float d = sqrt( pow(vehicle_state.p[0] - this->keyframes[i].first.first[0], 2) +
                        pow(vehicle_state.p[1] - this->keyframes[i].first.first[1], 2) +
                        pow(vehicle_state.p[2] - this->keyframes[i].first.first[2], 2) );
        ds.push_back(d);
        keyframe_nn.push_back(i);
    }
    lock.unlock();

    // get indices for top K nearest neighbor keyframe pose
    this->pushSubmapIndices(ds, this->submap_knn_, keyframe_nn);

    // Get convex hull indices
    this->computeConvexHull();

    // Get distance for each keyframe on convex hull
    std::vector<float> convex_ds;
    for (const auto& c : this->keyframe_convex) {
        convex_ds.push_back(ds[c]);
    }

    // get indices for top KNN for convex hull
    this->pushSubmapIndices(convex_ds, this->submap_kcv_, this->keyframe_convex);

    //get concave hull indices
    this->computeConcaveHull();

    // get distances for each keyframe on concave hull
    std::vector<float> concave_ds;
    for (const auto& c : this->keyframe_concave) {
        concave_ds.push_back(ds[c]);
    }

    // get indices for top kNN for concave hull
    this->pushSubmapIndices(concave_ds, this->submap_kcc_, this->keyframe_concave);

    // sort current and previous submap kf list of indices
    std::sort(this->submap_kf_idx_curr.begin(), this->submap_kf_idx_curr.end());
    std::sort(this->submap_kf_idx_prev.begin(), this->submap_kf_idx_prev.end());

    // remove duplicate indices
    auto last = std::unique(this->submap_kf_idx_curr.begin(), this->submap_kf_idx_curr.end());
    this->submap_kf_idx_curr.erase(last, this->submap_kf_idx_curr.end());

    // check if submap has changed from previous iteration
    if (this->submap_kf_idx_curr != this->submap_kf_idx_prev){

        this->submap_hasChanged = true;

        // Pause to prevent stealing resources from the main loop if it is running.
        this->pauseSubmapBuildIfNeeded();

        // reinitialize submap cloud and normals
        pcl::PointCloud<PointType>::Ptr submap_cloud_ (boost::make_shared<pcl::PointCloud<PointType>>());
        std::shared_ptr<nano_gicp::CovarianceList> submap_normals_ (std::make_shared<nano_gicp::CovarianceList>());

        for (auto k : this->submap_kf_idx_curr) {

            // create current submap cloud
            lock.lock();
            *submap_cloud_ += *this->keyframes[k].second;
            lock.unlock();

            // grab corresponding submap cloud's normals
            submap_normals_->insert( std::end(*submap_normals_),
                std::begin(*(this->keyframe_normals[k])), std::end(*(this->keyframe_normals[k])) );
        }

        this->submap_cloud = submap_cloud_;
        this->submap_normals = submap_normals_;

        // std::cout << "Submap normals (Covariance ?) ==> " << this->submap_normals->back() << std::endl;

        if (this->cov_initialized) {
            
            // Initalize the covariance
            Eigen::Matrix4d last_matrix = this->submap_normals->back();
            Eigen::Matrix3d lastest_normal = last_matrix.block<3,3>(0,0);

            // Convert to float
            Eigen::Matrix3f lastest_normal_fl = lastest_normal.cast<float>();
            this->R = lastest_normal_fl;
        }


        // Pause to prevent stealing resources from the main loop if it is running.
        this->pauseSubmapBuildIfNeeded();

        this->gicp_temp.setInputTarget(this->submap_cloud);
        this->submap_kdtree = this->gicp_temp.target_kdtree_;
        this->submap_kf_idx_prev = this->submap_kf_idx_curr;
    }
}


/**
    Build Key frames and submap (local map)
*/
void liom::NodeOdometry::buildKeyframesAndSubmap(State vehicle_state) {

    // Transform the new keyframe(s) and associated covariance list(t)
    std::unique_lock<decltype(this->keyframes_mutex)> lock(this->keyframes_mutex);

    for (int i = this->num_processed_keyframes; i < this->keyframes.size(); i++) {
        // ROS_WARN(" Build Key Frames");
        pcl::PointCloud<PointType>::ConstPtr raw_keyframe = this->keyframes[i].second;
        std::shared_ptr<const nano_gicp::CovarianceList> raw_covariances = this->keyframe_normals[i];
        Eigen::Matrix4f T = this->keyframe_transformations[i];
        lock.unlock();

        Eigen::Matrix4d Td = T.cast<double>();

        pcl::PointCloud<PointType>::Ptr transformed_keyframe (boost::make_shared<pcl::PointCloud<PointType>>());
        pcl::transformPointCloud (*raw_keyframe, *transformed_keyframe, T);

        std::shared_ptr<nano_gicp::CovarianceList> transformed_covariances (std::make_shared<nano_gicp::CovarianceList>(raw_covariances->size()));
        std::transform(raw_covariances->begin(), raw_covariances->end(), transformed_covariances->begin(),
                    [&Td](Eigen::Matrix4d cov) { return Td * cov * Td.transpose(); });

        ++this->num_processed_keyframes;

        lock.lock();
        this->keyframes[i].second = transformed_keyframe;
        this->keyframe_normals[i] = transformed_covariances;
        this->publish_keyframe_thread = std::thread( &liom::NodeOdometry::publishKeyframe, this, this->keyframes[i], this->keyframe_timestamps[i] );
        this->publish_keyframe_thread.detach();
    }

    lock.unlock();

    // Pause to prevent stealing resource from the main loop if it is running
    this->pauseSubmapBuildIfNeeded();

    this->buildSubmap(vehicle_state);
}

void liom::NodeOdometry::pauseSubmapBuildIfNeeded() {
  std::unique_lock<decltype(this->main_loop_running_mutex)> lock(this->main_loop_running_mutex);
  this->submap_build_cv.wait(lock, [this]{ return !this->main_loop_running; });
}


/**
    Propagate GICP function
*/
void liom::NodeOdometry::propagateGICP() {
    
    this->lidarPose.p << this->T(0,3), this->T(1,3), this->T(2,3);

    Eigen::Matrix3f rotSO3;
    rotSO3 << this->T(0,0), this->T(0,1), this->T(0,2),
              this->T(1,0), this->T(1,1), this->T(1,2),
              this->T(2,0), this->T(2,1), this->T(2,2);

    Eigen::Quaternionf q(rotSO3);

    // Normalize quaternion
    double norm  = sqrt(q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z());
    q.w() /= norm;
    q.x() /= norm;
    q.y() /= norm;
    q.z() /= norm;
    this->lidarPose.q = q;
}

bool containsNaN(const Eigen::Matrix<float, 3, 1>& mat) {
    return !mat.allFinite();
}

/**
    Update the State function v2
 */
void liom::NodeOdometry::updateState_IMM() {

    // Lock thread to prevent state from being accessed by PropagateState
    std::lock_guard<std::mutex> lock(this->geo.mtx);

    Eigen::Vector3f p_in    = this->lidarPose.p;
    Eigen::Quaternionf q_in = this->lidarPose.q;
    double dt   = this->scan_stamp - this->prev_scan_stamp;

    /**Modification */
    // Initialize the first IMM state
    if (this->imm_initialized) {
        this->angular_vel = this->angl_vec_total / 3.14 * 180 / this->num_imu;   // Compute the average angular velocity
        this->angular_vel = this->angular_vel / this->gravity_;     // Correct value due to gravity
        this->accel_avg = this->accel_total / this->num_imu;        // Compute the average linear velocity
        this->accel_avg[2] -= this->gravity_;                       // Correct z-axis due to attitude

        // Initialize covariance and measurement matrix
        this->x_prev_imm << this->x_init, this->angular_vel, this->accel_avg;
        this->P_imm = Eigen::Matrix<float, 12, 12>::Identity()*0.0001;
        this->Q     = Eigen::Matrix<float, 12, 12>::Identity()*0.05;
        this->Q_ct  = Eigen::Matrix<float, 12, 12>::Zero();
        this->Q_ct.block<3,3>(6, 6) = Eigen::Matrix<float, 3, 3>::Identity() * 0.005;

        this->H     = Eigen::Matrix<float, 3, 12>::Zero();
        this->H.block<3,3>(0,0) = Eigen::Matrix<float, 3, 3>::Identity();
        this->Mu_ij_v2 << 0.111, 0.111, 0.111, 0.111, 0.111, 0.111, 0.111, 0.111, 0.111;
        this->F_cv = Eigen::Matrix<float, 12, 12>::Identity();          // Constant-velocity model
        this->F_ca = Eigen::Matrix<float, 12, 12>::Identity();          // Constant-acceleration model
        this->F_ct = Eigen::Matrix<float, 12, 12>::Identity();          // Constant-turn rate model
        this->cbar_v2 << 0.111, 0.111, 0.111, 0.111, 0.111, 0.111, 0.111, 0.111, 0.111;
        this->imm_initialized = false;
        this->kalman_cov_initialized = true;

        // std::cout << "IMM state initialized " << this->x_prev_imm << std::endl;
    }

    this->F_cv = Eigen::Matrix<float, 12, 12>::Identity();          // Constant-velocity model
    this->F_ca = Eigen::Matrix<float, 12, 12>::Identity();          // Constant-acceleration model
    this->F_ct = Eigen::Matrix<float, 12, 12>::Identity();          // Constant-turn rate model

    this->angular_vel = this->angl_vec_total / 3.14 * 180 / this->num_imu;   // Compute the average angular velocity
    this->angular_vel = this->angular_vel / this->gravity_;     // Correct value due to gravity
    this->accel_avg = this->accel_total / this->num_imu;        // Compute the average linear velocity
    this->accel_avg[2] -= this->gravity_;  

    // Calculate timestampe in a LiDAR sweep
    float lidar_dt = (this->scan_stamp - this->prev_scan_stamp)/2;

    // Compute the transistion matrix for each model
    this->F_cv.block<3,3>(0,3) = Eigen::Matrix<float, 3,3>::Identity() * lidar_dt; // CV model

    this->F_ca.block<3,3>(0,3) = Eigen::Matrix<float, 3,3>::Identity() * lidar_dt; // CA model
    this->F_ca.block<3,3>(3,9) = Eigen::Matrix<float, 3,3>::Identity() * lidar_dt;
    this->F_ca.block<3,3>(0,9) = Eigen::Matrix<float, 3,3>::Identity() * 0.5*lidar_dt*lidar_dt;

    // CT model
    float Tx = this->angular_vel[0];
    float Ty = this->angular_vel[1];
    float Tz = this->angular_vel[2];
    float T_xyz = sqrt(Tx*Tx + Ty*Ty + Tz*Tz);

    float c1 = (cos(T_xyz * lidar_dt) - 1)/(T_xyz*T_xyz);
    float c2    = sin(T_xyz*lidar_dt)/T_xyz;
    float c3    = (1/(T_xyz*T_xyz))*(sin(T_xyz)*lidar_dt)/(T_xyz - lidar_dt);
    float d1    = Ty*Ty + Tz*Tz;
    float d2    = Tx*Tx + Tz*Tz;
    float d3    = Tx*Tx + Ty*Ty;
    Eigen::Matrix<float, 3, 3> A;
    Eigen::Matrix<float, 3, 3> B;
    A = Eigen::Matrix<float, 3, 3>::Zero();
    B = Eigen::Matrix<float, 3, 3>::Zero();

    A.block<1,3>(0,0) << c1*d1, -c2*Tz-c1*Tx*Ty, c2*Ty-c1*Tx*Tz;
    A.block<1,3>(1,0) << c2*Tz-c1*Tx*Ty, c1*d2, -c2*Tx-c1*Ty*Tz;
    A.block<1,3>(2,0) << -c2*Ty-c1*Tx*Tz, c2*Tx-c1*Ty*Tz, c1*d3;

    B.block<1,3>(0,0) << c3*d1, c1*Tz-c3*Tx*Ty, -c1*Ty-c3*Tx*Tz;
    B.block<1,3>(1,0) << -c1*Tz-c3*Tx*Ty, c3*d2, c1*Tx-c3*Ty*Tz;
    B.block<1,3>(2,0) << c1*Ty-c3*Tx*Tz, -c1*Tx-c3*Ty*Tz, c3*d3;

    this->F_ct.block<3,3>(0,3) = B;
    this->F_ct.block<3,3>(3,3) += A; 

    /*
        Kalman Filter first layer
    */
    // Predict state for each model
    this->x_predict_cv = this->F_cv * this->x_prev_imm;
    this->x_predict_ca = this->F_ca * this->x_prev_imm;
    this->x_predict_ct = this->F_ct * this->x_prev_imm;

    // Predict covariance matrix
    this->P_predict_cv = this->F_cv * this->P_imm * this->F_cv.transpose() + this->Q;
    this->P_predict_ca = this->F_ca * this->P_imm * this->F_ca.transpose() + this->Q;
    this->P_predict_ct = this->F_ct * this->P_imm * this->F_ct.transpose() + this->Q;

    /*
        Kalman Filter second layer
    */
    // Predict based on CV model
    this->x_predict_cv_cv = this->F_cv * this->x_predict_cv;
    this->x_predict_cv_ca = this->F_ca * this->x_predict_cv;
    this->x_predict_cv_ct = this->F_ct * this->x_predict_cv;

    // Predict based on CA model
    this->x_predict_ca_cv = this->F_cv * this->x_predict_ca;
    this->x_predict_ca_ca = this->F_ca * this->x_predict_ca;
    this->x_predict_ca_ct = this->F_ct * this->x_predict_ca;

    // Predict based on CT model
    this->x_predict_ct_cv = this->F_cv * this->x_predict_ct;
    this->x_predict_ct_ca = this->F_ca * this->x_predict_ct;
    this->x_predict_ct_ct = this->F_ct * this->x_predict_ct;

    /// Predict covariance CV model
    this->P_predict_cv_cv = this->F_cv * this->P_predict_cv * this->F_cv.transpose();
    this->P_predict_cv_ca = this->F_ca * this->P_predict_cv * this->F_ca.transpose();
    this->P_predict_cv_ct = this->F_ct * this->P_predict_cv * this->F_ct.transpose();

    /// Predict covariance CA model
    this->P_predict_ca_cv = this->F_cv * this->P_predict_ca * this->F_cv.transpose();
    this->P_predict_ca_ca = this->F_ca * this->P_predict_ca * this->F_ca.transpose();
    this->P_predict_ca_ct = this->F_ct * this->P_predict_ca * this->F_ct.transpose();

    /// Predict covariance CT model
    this->P_predict_ct_cv = this->F_cv * this->P_predict_ct * this->F_cv.transpose();
    this->P_predict_ct_ca = this->F_ca * this->P_predict_ct * this->F_ca.transpose();
    this->P_predict_ct_ct = this->F_ct * this->P_predict_ct * this->F_ct.transpose();

    // Compute residual covariance
    this->S_cv_cv = this->H * this->P_predict_cv_cv * this->H.transpose() + this->R;
    this->S_cv_ca = this->H * this->P_predict_cv_ca * this->H.transpose() + this->R;
    this->S_cv_ct = this->H * this->P_predict_cv_ct * this->H.transpose() + this->R;

    this->S_ca_cv = this->H * this->P_predict_ca_cv * this->H.transpose() + this->R;
    this->S_ca_ca = this->H * this->P_predict_ca_ca * this->H.transpose() + this->R;
    this->S_ca_ct = this->H * this->P_predict_ca_ct * this->H.transpose() + this->R;

    this->S_ct_cv = this->H * this->P_predict_ct_cv * this->H.transpose() + this->R;
    this->S_ct_ca = this->H * this->P_predict_ct_ca * this->H.transpose() + this->R;
    this->S_ct_ct = this->H * this->P_predict_ct_ct * this->H.transpose() + this->R;

    // Compute Kalman gain fro each model
    this->K_cv_cv = this->P_predict_cv_cv * this->H.transpose() *  (this->H * this->P_predict_cv_cv * this->H.transpose() + this->R).inverse();
    this->K_cv_ca = this->P_predict_cv_ca * this->H.transpose() *  (this->H * this->P_predict_cv_ca * this->H.transpose() + this->R).inverse();
    this->K_cv_ct = this->P_predict_cv_ct * this->H.transpose() *  (this->H * this->P_predict_cv_ct * this->H.transpose() + this->R).inverse();
    this->K_ca_cv = this->P_predict_ca_cv * this->H.transpose() *  (this->H * this->P_predict_ca_cv * this->H.transpose() + this->R).inverse();
    this->K_ca_ca = this->P_predict_ca_ca * this->H.transpose() *  (this->H * this->P_predict_ca_ca * this->H.transpose() + this->R).inverse();
    this->K_ca_ct = this->P_predict_ca_ct * this->H.transpose() *  (this->H * this->P_predict_ca_ct * this->H.transpose() + this->R).inverse();
    this->K_ct_cv = this->P_predict_ct_cv * this->H.transpose() *  (this->H * this->P_predict_ct_cv * this->H.transpose() + this->R).inverse();
    this->K_ct_ca = this->P_predict_ct_ca * this->H.transpose() *  (this->H * this->P_predict_ct_ca * this->H.transpose() + this->R).inverse();
    this->K_ct_ct = this->P_predict_ct_ct * this->H.transpose() *  (this->H * this->P_predict_ct_ct * this->H.transpose() + this->R).inverse();

    // Compute residual error to measurement
    this->err_cv_cv = p_in - this->H * this->x_predict_cv_cv;
    this->err_cv_ca = p_in - this->H * this->x_predict_cv_ca;
    this->err_cv_ct = p_in - this->H * this->x_predict_cv_ct;
    this->err_ca_cv = p_in - this->H * this->x_predict_ca_cv;
    this->err_ca_ca = p_in - this->H * this->x_predict_ca_ca;
    this->err_ca_ct = p_in - this->H * this->x_predict_ca_ct;
    this->err_ct_cv = p_in - this->H * this->x_predict_ct_cv;
    this->err_ct_ca = p_in - this->H * this->x_predict_ct_ca;
    this->err_ct_ct = p_in - this->H * this->x_predict_ct_ct;

    // Compute the likelihood score of each model compare to the measurement
    float pi = 3.14159265359;
    this->Lfun_cv_cv = 1/sqrt(abs(2*pi * this->S_cv_cv.determinant())) * exp((-0.5 * this->err_cv_cv.transpose() * this->S_cv_cv.inverse() * this->err_cv_cv));
    this->Lfun_cv_ca = 1/sqrt(abs(2*pi * this->S_cv_ca.determinant())) * exp((-0.5 * this->err_cv_ca.transpose() * this->S_cv_ca.inverse() * this->err_cv_ca));
    this->Lfun_cv_ct = 1/sqrt(abs(2*pi * this->S_cv_ct.determinant())) * exp((-0.5 * this->err_cv_ct.transpose() * this->S_cv_ct.inverse() * this->err_cv_ct));
    this->Lfun_ca_cv = 1/sqrt(abs(2*pi * this->S_ca_cv.determinant())) * exp((-0.5 * this->err_ca_cv.transpose() * this->S_ca_cv.inverse() * this->err_ca_cv));
    this->Lfun_ca_ca = 1/sqrt(abs(2*pi * this->S_ca_ca.determinant())) * exp((-0.5 * this->err_ca_ca.transpose() * this->S_ca_ca.inverse() * this->err_ca_ca));
    this->Lfun_ca_ct = 1/sqrt(abs(2*pi * this->S_ca_ct.determinant())) * exp((-0.5 * this->err_ca_ct.transpose() * this->S_ca_ct.inverse() * this->err_ca_ct));
    this->Lfun_ct_cv = 1/sqrt(abs(2*pi * this->S_ct_cv.determinant())) * exp((-0.5 * this->err_ct_cv.transpose() * this->S_ct_cv.inverse() * this->err_ct_cv));
    this->Lfun_ct_ca = 1/sqrt(abs(2*pi * this->S_ct_ca.determinant())) * exp((-0.5 * this->err_ct_ca.transpose() * this->S_ct_ca.inverse() * this->err_ct_ca));
    this->Lfun_ct_ct = 1/sqrt(abs(2*pi * this->S_ct_ct.determinant())) * exp((-0.5 * this->err_ct_ct.transpose() * this->S_ct_ct.inverse() * this->err_ct_ct));

    // Update state and covariance for each model
    this->x_hat_cv_cv = this->x_predict_cv_cv + this->K_cv_cv * this->err_cv_cv;
    this->x_hat_cv_ca = this->x_predict_cv_ca + this->K_cv_ca * this->err_cv_ca;
    this->x_hat_cv_ct = this->x_predict_cv_ct + this->K_cv_ct * this->err_cv_ct;
    this->x_hat_ca_cv = this->x_predict_ca_cv + this->K_ca_cv * this->err_ca_cv;
    this->x_hat_ca_ca = this->x_predict_ca_ca + this->K_ca_ca * this->err_ca_ca;
    this->x_hat_ca_ct = this->x_predict_ca_ct + this->K_ca_ct * this->err_ca_ct;
    this->x_hat_ct_cv = this->x_predict_ct_cv + this->K_ct_cv * this->err_ct_cv;
    this->x_hat_ct_ca = this->x_predict_ct_ca + this->K_ct_ca * this->err_ct_ca;
    this->x_hat_ct_ct = this->x_predict_ct_ct + this->K_ct_ct * this->err_ct_ct;

    this->P_hat_cv_cv = (Eigen::Matrix<float, 12, 12>::Identity() - this->K_cv_cv * this->H) * this->P_predict_cv_cv;
    this->P_hat_cv_ca = (Eigen::Matrix<float, 12, 12>::Identity() - this->K_cv_ca * this->H) * this->P_predict_cv_ca;
    this->P_hat_cv_ct = (Eigen::Matrix<float, 12, 12>::Identity() - this->K_cv_ct * this->H) * this->P_predict_cv_ct;
    this->P_hat_ca_cv = (Eigen::Matrix<float, 12, 12>::Identity() - this->K_ca_cv * this->H) * this->P_predict_ca_cv;
    this->P_hat_ca_ca = (Eigen::Matrix<float, 12, 12>::Identity() - this->K_ca_ca * this->H) * this->P_predict_ca_ca;
    this->P_hat_ca_ct = (Eigen::Matrix<float, 12, 12>::Identity() - this->K_ca_ct * this->H) * this->P_predict_ca_ct;
    this->P_hat_ct_cv = (Eigen::Matrix<float, 12, 12>::Identity() - this->K_ct_cv * this->H) * this->P_predict_ct_cv;
    this->P_hat_ct_ca = (Eigen::Matrix<float, 12, 12>::Identity() - this->K_ct_ca * this->H) * this->P_predict_ct_ca;
    this->P_hat_ct_ct = (Eigen::Matrix<float, 12, 12>::Identity() - this->K_ct_ct * this->H) * this->P_predict_ct_ct;


    // Compute the likelihood of each model
    float t_Lfun = this->Lfun_cv_cv + this->Lfun_cv_ca + this->Lfun_cv_ct + this->Lfun_ca_cv + this->Lfun_ca_ca + this->Lfun_ca_ct + this->Lfun_ct_cv + this->Lfun_ct_ca + this->Lfun_ct_ct;
    float Lfun_cv_cv_new = this->Lfun_cv_cv / t_Lfun;
    float Lfun_cv_ca_new = this->Lfun_cv_ca / t_Lfun;
    float Lfun_cv_ct_new = this->Lfun_cv_ct / t_Lfun;
    float Lfun_ca_cv_new = this->Lfun_ca_cv / t_Lfun;
    float Lfun_ca_ca_new = this->Lfun_ca_ca / t_Lfun;
    float Lfun_ca_ct_new = this->Lfun_ca_ct / t_Lfun;
    float Lfun_ct_cv_new = this->Lfun_ct_cv / t_Lfun;
    float Lfun_ct_ca_new = this->Lfun_ct_ca / t_Lfun;
    float Lfun_ct_ct_new = this->Lfun_ct_ct / t_Lfun;

    // Update the model probability
    this->merge_v2 << Lfun_cv_cv_new, Lfun_cv_ca_new, Lfun_cv_ct_new, Lfun_ca_cv_new, Lfun_ca_ca_new, Lfun_ca_ct_new, Lfun_ct_cv_new, Lfun_ct_ca_new, Lfun_ct_ct_new;

    float c = this->merge_v2 * this->cbar_v2;

    this->Mu_ij_v2 = this->merge_v2.transpose().cwiseProduct(this->cbar_v2/c);
    
    // Update the final estimation and covariance
    this->x_prev_imm = this->x_hat_cv_cv * this->Mu_ij_v2[0] + this->x_hat_cv_ca * this->Mu_ij_v2[1] + this->x_hat_cv_ct * this->Mu_ij_v2[2] + this->x_hat_ca_cv * this->Mu_ij_v2[3] + this->x_hat_ca_ca * this->Mu_ij_v2[4] + this->x_hat_ca_ct * this->Mu_ij_v2[5] + this->x_hat_ct_cv * this->Mu_ij_v2[6] + this->x_hat_ct_ca * this->Mu_ij_v2[7] + this->x_hat_ct_ct * this->Mu_ij_v2[8];
    this->P_imm = this->Mu_ij_v2[0] * (this->P_hat_cv_cv + (this->x_hat_cv_cv - this->x_prev_imm)*(this->x_hat_cv_cv - this->x_prev_imm).transpose()) + Mu_ij_v2[1] * (this->P_hat_cv_ca + (this->x_hat_cv_ca - this->x_prev_imm)*(this->x_hat_cv_ca - this->x_prev_imm).transpose()) + Mu_ij_v2[2] * (this->P_hat_cv_ct + (this->x_hat_cv_ct - this->x_prev_imm)*(this->x_hat_cv_ct - this->x_prev_imm).transpose()) + this->Mu_ij_v2[3] * (this->P_hat_ca_cv + (this->x_hat_ca_cv - this->x_prev_imm)*(this->x_hat_ca_cv - this->x_prev_imm).transpose()) + Mu_ij_v2[4] * (this->P_hat_ca_ca + (this->x_hat_ca_ca - this->x_prev_imm)*(this->x_hat_ca_ca - this->x_prev_imm).transpose()) + Mu_ij_v2[5] * (this->P_hat_ca_ct + (this->x_hat_ca_ct - this->x_prev_imm)*(this->x_hat_ca_ct - this->x_prev_imm).transpose()) + this->Mu_ij_v2[6] * (this->P_hat_ct_cv + (this->x_hat_ct_cv - this->x_prev_imm)*(this->x_hat_ct_cv - this->x_prev_imm).transpose()) + Mu_ij_v2[7] * (this->P_hat_ct_ca + (this->x_hat_ct_ca - this->x_prev_imm)*(this->x_hat_ct_ca - this->x_prev_imm).transpose()) + Mu_ij_v2[8] * (this->P_hat_ct_ct + (this->x_hat_ct_ct - this->x_prev_imm)*(this->x_hat_ct_ct - this->x_prev_imm).transpose());

    this->p_in_prev  = p_in;
    this->accel_total << 0, 0, 0;
    this->angl_vec_total << 0, 0, 0;
    this->num_imu = 0;
    //////
    if (!std::isnan(this->x_prev_imm[1])) {
        this->state.p << this->x_prev_imm.block<1,3>(0,0);
        this->state.v.linear.w << this->x_prev_imm.block<3,1>(3,0);
    }
        Eigen::Quaternionf q_e, q_hat, q_corr;
    q_hat = this->state.q;

    // Construct error quaternion
    q_e = q_hat.conjugate()*q_in;                       // q^_i * q^_k 

    double sgn = 1;
    if (q_e.w() < 0) {
        sgn = -1;
    } 

    // Construct quaternion correction
    q_corr.w()   =    1 - abs(q_e.w());               
    q_corr.vec() =    sgn*q_e.vec();
    q_corr       =    q_hat*q_corr;

    // Denote p_error
    Eigen::Vector3f p_err = p_in - this->state.p;       // LiDAR pose - propagated pose (p^_k - p^_i)
    Eigen::Vector3f p_err_body;

    // Convert p_err to body frames
    p_err_body = q_hat.conjugate()._transformVector(p_err);

    double a_bias_max = this->geo_abias_max_;
    double g_bias_max = this->geo_gbias_max_;

    // Update acceleration bias
    this->state.b.accel -= dt * this->geo_Kab_ * p_err_body;              
    this->state.b.accel  = this->state.b.accel.array().min(a_bias_max).max(-a_bias_max);

    // Update gyro bias
    this->state.b.gyro[0] -= dt * this->geo_Kgb_ * q_e.w() * q_e.x();   
    // std::cout << " Alpha 2 ==> " << this->geo_Kgb_ << std::endl;
    this->state.b.gyro[1] -= dt * this->geo_Kgb_ * q_e.w() * q_e.y();
    this->state.b.gyro[2] -= dt * this->geo_Kgb_ * q_e.w() * q_e.z();
    this->state.b.gyro = this->state.b.gyro.array().min(g_bias_max).max(-g_bias_max);

    this->state.q.w() += dt * this->geo_Kq_ * q_corr.w();
    this->state.q.x() += dt * this->geo_Kq_ * q_corr.x();
    this->state.q.y() += dt * this->geo_Kq_ * q_corr.y();
    this->state.q.z() += dt * this->geo_Kq_ * q_corr.z();
    this->state.q.normalize();

    // Store previous pose, orientation and velocity
    this->geo.prev_p = this->state.p;
    this->geo.prev_q = this->state.q;
    this->geo.prev_vel = this->state.v.linear.w;

    if (!std::isnan(this->x_prev_imm[1])) {
        this->state.p << this->x_prev_imm.block<1,3>(0,0);
        this->state.v.linear.w << this->x_prev_imm.block<3,1>(3,0);
    }
}
/**
    Update the State function
*/
void liom::NodeOdometry::updateState() {

    // Lock thread to prevent state from being accessed by PropagateState
    std::lock_guard<std::mutex> lock( this->geo.mtx );

    Eigen::Vector3f p_in    = this->lidarPose.p;
    Eigen::Quaternionf q_in = this->lidarPose.q;
    double dt = this->scan_stamp - this->prev_scan_stamp;
    
    /**Modification */

    // Initialize the first IMM state
    if (this->imm_initialized) {
        this->angular_vel = this->angl_vec_total / 3.14 * 180 / this->num_imu;   // Compute the average angular velocity
        this->angular_vel = this->angular_vel / this->gravity_;     // Correct value due to gravity
        this->accel_avg = this->accel_total / this->num_imu;        // Compute the average linear velocity
        this->accel_avg[2] -= this->gravity_;                       // Correct z-axis due to attitude

        // Initialize covariance and measurement matrix
        this->x_prev_imm << this->x_init, this->angular_vel, this->accel_avg;
        this->P_imm = Eigen::Matrix<float, 12, 12>::Identity()*0.0001;
        this->Q     = Eigen::Matrix<float, 12, 12>::Identity()*0.005;
        this->Q_ct  = Eigen::Matrix<float, 12, 12>::Zero();
        this->Q_ct.block<3,3>(6, 6) = Eigen::Matrix<float, 3, 3>::Identity() * 0.005;
        // std::cout << "CT covariance " << this->Q_ct << std::endl;

        this->H     = Eigen::Matrix<float, 3, 12>::Zero();
        this->H.block<3,3>(0,0) = Eigen::Matrix<float, 3, 3>::Identity();
        this->P_ij  << 0.9, 0.01, 0.09, 0.025, 0.75, 0.225, 0.075, 0.175, 0.75;
        this->Mu_ij << 0.333, 0.333, 0.333;
        this->F_cv = Eigen::Matrix<float, 12, 12>::Identity();          // Constant-velocity model
        this->F_ca = Eigen::Matrix<float, 12, 12>::Identity();          // Constant-acceleration model
        this->F_ct = Eigen::Matrix<float, 12, 12>::Identity();          // Constant-turn rate model
        this->cbar = this->P_ij * this->Mu_ij;
        this->imm_initialized = false;
        this->kalman_cov_initialized = true;
    }

    this->F_cv = Eigen::Matrix<float, 12, 12>::Identity();          // Constant-velocity model
    this->F_ca = Eigen::Matrix<float, 12, 12>::Identity();          // Constant-acceleration model
    this->F_ct = Eigen::Matrix<float, 12, 12>::Identity();          // Constant-turn rate model

    this->angular_vel = this->angl_vec_total / 3.14 * 180 / this->num_imu;   // Compute the average angular velocity
    this->angular_vel = this->angular_vel / this->gravity_;     // Correct value due to gravity
    this->accel_avg = this->accel_total / this->num_imu;        // Compute the average linear velocity
    this->accel_avg[2] -= this->gravity_;                       // Correct z-axis due to attitude

    // Calculate timestampe in a LiDAR sweep
    float lidar_dt = this->scan_stamp - this->prev_scan_stamp;


    // Compute the transistion matrix for each model
    this->F_cv.block<3,3>(0,3) = Eigen::Matrix<float, 3,3>::Identity() * lidar_dt; // CV model

    this->F_ca.block<3,3>(0,3) = Eigen::Matrix<float, 3,3>::Identity() * lidar_dt; // CA model
    this->F_ca.block<3,3>(3,9) = Eigen::Matrix<float, 3,3>::Identity() * lidar_dt;
    this->F_ca.block<3,3>(0,9) = Eigen::Matrix<float, 3,3>::Identity() * 0.5*lidar_dt*lidar_dt;

    // CT model
    float Tx = this->angular_vel[0];
    float Ty = this->angular_vel[1];
    float Tz = this->angular_vel[2];
    float T_xyz = sqrt(Tx*Tx + Ty*Ty + Tz*Tz);

    float c1 = (cos(T_xyz * lidar_dt) - 1)/(T_xyz*T_xyz);
    float c2    = sin(T_xyz*lidar_dt)/T_xyz;
    float c3    = (1/(T_xyz*T_xyz))*(sin(T_xyz)*lidar_dt)/(T_xyz - lidar_dt);
    float d1    = Ty*Ty + Tz*Tz;
    float d2    = Tx*Tx + Tz*Tz;
    float d3    = Tx*Tx + Ty*Ty;
    Eigen::Matrix<float, 3, 3> A;
    Eigen::Matrix<float, 3, 3> B;
    A = Eigen::Matrix<float, 3, 3>::Zero();
    B = Eigen::Matrix<float, 3, 3>::Zero();

    A.block<1,3>(0,0) << c1*d1, -c2*Tz-c1*Tx*Ty, c2*Ty-c1*Tx*Tz;
    A.block<1,3>(1,0) << c2*Tz-c1*Tx*Ty, c1*d2, -c2*Tx-c1*Ty*Tz;
    A.block<1,3>(2,0) << -c2*Ty-c1*Tx*Tz, c2*Tx-c1*Ty*Tz, c1*d3;

    B.block<1,3>(0,0) << c3*d1, c1*Tz-c3*Tx*Ty, -c1*Ty-c3*Tx*Tz;
    B.block<1,3>(1,0) << -c1*Tz-c3*Tx*Ty, c3*d2, c1*Tx-c3*Ty*Tz;
    B.block<1,3>(2,0) << c1*Ty-c3*Tx*Tz, -c1*Tx-c3*Ty*Tz, c3*d3;

    this->F_ct.block<3,3>(0,3) = B;
    this->F_ct.block<3,3>(3,3) += A; 

    /*
        Kalman filters
    */

    // Predict state for each model
    this->x_predict_cv = this->F_cv * this->x_prev_imm;
    this->x_predict_ca = this->F_ca * this->x_prev_imm;
    this->x_predict_ct = this->F_ct * this->x_prev_imm;
    this->x_predict_ct.block<3,1>(3,0) = this->x_predict_cv.block<3,1>(3,0);

    // Predict covariance matrix
    this->P_predict_cv = this->F_cv * this->P_imm * this->F_cv.transpose() + this->Q;
    this->P_predict_ca = this->F_ca * this->P_imm * this->F_ca.transpose() + this->Q;
    this->P_predict_ct = this->F_ct * this->P_imm * this->F_ct.transpose() + this->Q;

    // Compute residual covariance
    this->S_cv = this->H * this->P_predict_cv * this->H.transpose() + this->R;
    this->S_ca = this->H * this->P_predict_ca * this->H.transpose() + this->R;
    this->S_ct = this->H * this->P_predict_ct * this->H.transpose() + this->R;

    // Compute Kalman gain fro each model
    this->K_cv = this->P_predict_cv * this->H.transpose() *  (this->H * this->P_predict_cv * this->H.transpose() + this->R).inverse();
    this->K_ca = this->P_predict_ca * this->H.transpose() *  (this->H * this->P_predict_ca * this->H.transpose() + this->R).inverse();
    this->K_ct = this->P_predict_ct * this->H.transpose() *  (this->H * this->P_predict_ct * this->H.transpose() + this->R).inverse();

    // Compute residual error to measurement
    this->err_cv = p_in - this->H * this->x_predict_cv;
    this->err_ca = p_in - this->H * this->x_predict_ca;
    this->err_ct = p_in - this->H * this->x_predict_ct;

    if (this->kalman_cov_initialized) {
        // Store the Kalman * error result
        this->Ke_cv_prev = this->K_cv * this->err_cv;
        this->Ke_ca_prev = this->K_ca * this->err_ca;
        this->Ke_ct_prev = this->K_ct * this->err_ct;
        this->kalman_cov_initialized = false;
    } else {
        Eigen::Matrix<float, 12, 1> Ke_diff_cv = this->K_cv * this->err_cv - this->Ke_cv_prev;
        Eigen::Matrix<float, 12, 1> Ke_diff_ca = this->K_ca * this->err_ca - this->Ke_ca_prev;
        Eigen::Matrix<float, 12, 1> Ke_diff_ct = this->K_ct * this->err_ct - this->Ke_ct_prev;

        if (abs(Ke_diff_cv[3]) >= 1.0 || abs(Ke_diff_cv[4]) >= 1.0 || abs(Ke_diff_cv[5]) >= 1.0) {
            this->P_imm = Eigen::Matrix<float, 12, 12>::Identity()*0.0001;
            this->P_predict_cv = this->F_cv * this->P_imm * this->F_cv.transpose() + this->Q;
            this->K_cv = this->P_predict_cv * this->H.transpose() *  (this->H * this->P_predict_cv * this->H.transpose() + this->R).inverse();
            this->S_cv = this->H * this->P_predict_cv * this->H.transpose() + this->R;
            this->Ke_cv_prev = this->K_cv * this->err_cv;
        } else {
            this->Ke_cv_prev = this->K_cv * this->err_cv;
        }

        if (abs(Ke_diff_ca[3]) >= 1.0 || abs(Ke_diff_ca[4]) >= 1.0 || abs(Ke_diff_ca[5]) >= 1.0) {
            this->P_imm = Eigen::Matrix<float, 12, 12>::Identity()*0.0001;
            this->P_predict_ca = this->F_ca * this->P_imm * this->F_ca.transpose() + this->Q;
            this->K_ca = this->P_predict_ca * this->H.transpose() *  (this->H * this->P_predict_ca * this->H.transpose() + this->R).inverse();
            this->S_ca = this->H * this->P_predict_ca * this->H.transpose() + this->R;
            this->Ke_ca_prev = this->K_ca * this->err_ca;
        } else {
            this->Ke_ca_prev = this->K_ca * this->err_ca;
        }

        if (abs(Ke_diff_ct[3]) >= 1.0 || abs(Ke_diff_ct[4]) >= 1.0 || abs(Ke_diff_ct[5]) >= 1.0) {
            this->P_imm = Eigen::Matrix<float, 12, 12>::Identity()*0.0001;
            this->P_predict_ct = this->F_ct * this->P_imm * this->F_ct.transpose() + this->Q;
            this->K_ct = this->P_predict_ct * this->H.transpose() *  (this->H * this->P_predict_ct * this->H.transpose() + this->R).inverse();
            this->S_ct = this->H * this->P_predict_ct * this->H.transpose() + this->R;
            this->Ke_ct_prev = this->K_ct * this->err_ct;
        } else {
            this->Ke_ct_prev = this->K_ct * this->err_ct;
        }

    }

    // Compute the likelihood score of each model compare to the measurement
    float pi = 3.14159265359;
    this->Lfun_cv = 1/sqrt(abs(2*pi * this->S_cv.determinant())) * exp((-0.5 * this->err_cv.transpose() * this->S_cv.inverse() * this->err_cv));
    this->Lfun_ca = 1/sqrt(abs(2*pi * this->S_ca.determinant())) * exp((-0.5 * this->err_ca.transpose() * this->S_ca.inverse() * this->err_ca));
    this->Lfun_ct = 1/sqrt(abs(2*pi * this->S_ct.determinant())) * exp((-0.5 * this->err_ct.transpose() * this->S_ct.inverse() * this->err_ct));

    // Update state and covariance for each model
    this->x_hat_cv = this->x_predict_cv + this->K_cv * this->err_cv;
    this->x_hat_ca = this->x_predict_ca + this->K_ca * this->err_ca;
    this->x_hat_ct = this->x_predict_ct + this->K_ct * this->err_ct;
    this->x_hat_ct.block<3,1>(3,0) = this->x_hat_cv.block<3,1>(3,0);

    this->P_hat_cv = (Eigen::Matrix<float, 12, 12>::Identity() - this->K_cv * this->H) * this->P_predict_cv;
    this->P_hat_ca = (Eigen::Matrix<float, 12, 12>::Identity() - this->K_ca * this->H) * this->P_predict_ca;
    this->P_hat_ct = (Eigen::Matrix<float, 12, 12>::Identity() - this->K_ct * this->H) * this->P_predict_ct;

    if (this->Lfun_cv == 0 && this->Lfun_ca == 0 && this->Lfun_ct == 0) {
        ROS_WARN("debug");
    } else {    
         // Mixing param probability
        float t_Lfun = this->Lfun_cv + this->Lfun_ca + this->Lfun_ct;
        float Lfun_cv_new = this->Lfun_cv / t_Lfun;
        float Lfun_ca_new = this->Lfun_ca / t_Lfun;
        float Lfun_ct_new = this->Lfun_ct / t_Lfun;

        // Update the model probability
        this->merge << Lfun_cv_new, Lfun_ca_new, Lfun_ct_new;
        float c = this->merge * this->cbar;
        this->Mu_ij = this->merge.transpose().cwiseProduct(this->cbar/c);
        if (!containsNaN(this->Mu_ij)) {
            this->Mu_ij_prev = this->Mu_ij;
        } else {
            this->Mu_ij = this->Mu_ij_prev;
        }

        ROS_WARN("Mu = (%.4f, %.4f, %.4f)", this->Mu_ij[0], this->Mu_ij[1], this->Mu_ij[2] );
    }

    this->x_prev_imm = this->x_hat_cv * this->Mu_ij[0] + this->x_hat_ca * this->Mu_ij[1] + this->x_hat_ct * this->Mu_ij[2];
    this->P_imm = this->Mu_ij[0] * (this->P_hat_cv + (this->x_hat_cv - this->x_prev_imm)*(this->x_hat_cv - this->x_prev_imm).transpose()) + Mu_ij[1] * (this->P_hat_ca + (this->x_hat_ca - this->x_prev_imm)*(this->x_hat_ca - this->x_prev_imm).transpose()) + Mu_ij[2] * (this->P_hat_ct + (this->x_hat_ct - this->x_prev_imm)*(this->x_hat_ct - this->x_prev_imm).transpose());

    this->p_in_prev  = p_in;
    this->accel_total << 0, 0, 0;
    this->angl_vec_total << 0, 0, 0;
    this->num_imu = 0;
    ///////////////////////////////////

    Eigen::Quaternionf q_e, q_hat, q_corr;
    q_hat = this->state.q;

    // Construct error quaternion
    q_e = q_hat.conjugate()*q_in;                       // q^_i * q^_k 

    double sgn = 1;
    if (q_e.w() < 0) {
        sgn = -1;
    } 

    // Construct quaternion correction
    q_corr.w()   =    1 - abs(q_e.w());               
    q_corr.vec() =    sgn*q_e.vec();
    q_corr       =    q_hat*q_corr;

    // Denote p_error
    Eigen::Vector3f p_err = p_in - this->state.p;       // LiDAR pose - propagated pose (p^_k - p^_i)
    Eigen::Vector3f p_err_body;

    // Convert p_err to body frames
    p_err_body = q_hat.conjugate()._transformVector(p_err);

    double a_bias_max = this->geo_abias_max_;
    double g_bias_max = this->geo_gbias_max_;

    // Update acceleration bias
    this->state.b.accel -= dt * this->geo_Kab_ * p_err_body;            
    this->state.b.accel  = this->state.b.accel.array().min(a_bias_max).max(-a_bias_max);

    // Update gyro bias
    this->state.b.gyro[0] -= dt * this->geo_Kgb_ * q_e.w() * q_e.x();   
    // std::cout << " Alpha 2 ==> " << this->geo_Kgb_ << std::endl;
    this->state.b.gyro[1] -= dt * this->geo_Kgb_ * q_e.w() * q_e.y();
    this->state.b.gyro[2] -= dt * this->geo_Kgb_ * q_e.w() * q_e.z();
    this->state.b.gyro = this->state.b.gyro.array().min(g_bias_max).max(-g_bias_max);

    // Update state
    this->state.p       += dt * this->geo_Kp_ * p_err;                        
    this->state.v.linear.w += dt * this->geo_Kv_ * p_err;                                       

    this->state.q.w() += dt * this->geo_Kq_ * q_corr.w();
    this->state.q.x() += dt * this->geo_Kq_ * q_corr.x();
    this->state.q.y() += dt * this->geo_Kq_ * q_corr.y();
    this->state.q.z() += dt * this->geo_Kq_ * q_corr.z();
    this->state.q.normalize();

    // Store previous pose, orientation and velocity
    this->geo.prev_p = this->state.p;
    this->geo.prev_q = this->state.q;
    this->geo.prev_vel = this->state.v.linear.w;

    if (!std::isnan(this->x_prev_imm[1])) {
        this->state.p << this->x_prev_imm.block<1,3>(0,0);
        this->x_prev_imm.block<3,1>(3,0) << this->state.v.linear.w;
    }
}

bool isIdentity(const Eigen::Matrix4f& mat) {
    return mat.isIdentity();
}



/**
    Get Next Pose function
*/
void liom::NodeOdometry::getNextPose() {

    // Check if the new submap is ready to be used
    this->new_submap_is_ready = (this->submap_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready);

    if (this->new_submap_is_ready && this->submap_hasChanged) {
        // Set the current global submap as the target cloud
        this->gicp.registerInputTarget(this->submap_cloud);

        // Set submap kdtree
        this->gicp.target_kdtree_ = this->submap_kdtree;

        // Set target cloud's normal as submap normals
        this->gicp.setTargetCovariances(this->submap_normals);

        this->submap_hasChanged = false;
    }

    // Algin with current submap with global IMU transformation as initial guess
    pcl::PointCloud<PointType>::Ptr aligned (boost::make_shared<pcl::PointCloud<PointType>>());
    this->gicp.align(*aligned);

    // Get final transformation in global frame
    this->T_corr = this->gicp.getFinalTransformation();     // "correction " transformation
    this->T = this->T_corr * this->T_prior;

    if (!isIdentity(T_corr)) {
        this->T_corr_prev = this->T_corr;
    } else {
        this->T_corr = this->T_corr_prev;
    }


    // Update the next global pose
    // Both source and target clouds are in the global frame now, so transformation is global
    this->propagateGICP();

    // Geometric observer update
    // this->updateState();
    this->updateState_IMM();

}


/**
    Update the Key Frames function
*/
void liom::NodeOdometry::updateKeyframes() {

    // Calculate difference in pose and rotation to all poses in trajectory
    float closest_d = std::numeric_limits<float>::infinity();
    int   closest_idx = 0;
    int   keyframes_idx = 0;
    int   num_nearby = 0;

    for (const auto& k : this->keyframes) {

        // Calculate distance between current pose and pose in keyframes
        float delta_d = sqrt( pow(this->state.p[0] - k.first.first[0], 2) +
                              pow(this->state.p[1] - k.first.first[1], 2) + 
                              pow(this->state.p[2] - k.first.first[2], 2) );

        // Count the number nearby current pose
        if (delta_d <= this->keyframe_thresh_dist_ * 1.5) {
            ++num_nearby;
        }

        // std::cout << " Number of nearby keyframes ==> " << num_nearby << std::endl;

        // store into variable 
        if (delta_d < closest_d) {
            closest_d   = delta_d;
            closest_idx = keyframes_idx; 
        }

        keyframes_idx++;
    }

    // Get closets pose and corresponding rotation
    Eigen::Vector3f closest_pose = this->keyframes[closest_idx].first.first;
    Eigen::Quaternionf closest_pose_r = this->keyframes[closest_idx].first.second;

    // calculate distance between current pose and closest pose from above
    float dd = sqrt( pow(this->state.p[0] - closest_pose[0], 2) +
                    pow(this->state.p[1] - closest_pose[1], 2) +
                    pow(this->state.p[2] - closest_pose[2], 2) );

    // calculate difference in orientation using SLERP
    Eigen::Quaternionf dq;

    if (this->state.q.dot(closest_pose_r) < 0.) {
        Eigen::Quaternionf lq = closest_pose_r;
        lq.w() *= -1.; lq.x() *= -1.; lq.y() *= -1.; lq.z() *= -1.;
        dq = this->state.q * lq.inverse();
    } else {
        dq = this->state.q * closest_pose_r.inverse();
    }

    double theta_rad = 2. * atan2(sqrt( pow(dq.x(), 2) + pow(dq.y(), 2) + pow(dq.z(), 2) ), dq.w());
    double theta_deg = theta_rad * (180.0/M_PI);

    // update keyframes
    bool newKeyframe = false;

    if (abs(dd) > this->keyframe_thresh_dist_ || abs(theta_deg) > this->keyframe_thresh_rot_) {
        newKeyframe = true;
    }

    if (abs(dd) <= this->keyframe_thresh_dist_) {
        newKeyframe = false;
    }

    if (abs(dd) <= this->keyframe_thresh_dist_ && abs(theta_deg) > this->keyframe_thresh_rot_ && num_nearby <= 1) {
        newKeyframe = true;
    }

    if (newKeyframe) {

        // update keyframe vector
        std::unique_lock<decltype(this->keyframes_mutex)> lock(this->keyframes_mutex);
        // this->keyframes.push_back(std::make_pair(std::make_pair(this->lidarPose.p, this->lidarPose.q), this->current_scan));
        this->keyframes.push_back(std::make_pair(std::make_pair(this->state.p, this->state.q), this->current_scan));
        this->keyframe_timestamps.push_back(this->scan_header_stamp);
        this->keyframe_normals.push_back(this->gicp.getSourceCovariances());
        this->keyframe_transformations.push_back(this->T_corr);
        lock.unlock();

    }

}

void liom::NodeOdometry::publishCloud(pcl::PointCloud<PointType>::ConstPtr published_cloud, Eigen::Matrix4f T_cloud) {

  if (this->wait_until_move_) {
    if (this->length_traversed < 0.1) { return; }
  }

  pcl::PointCloud<PointType>::Ptr deskewed_scan_t_ (boost::make_shared<pcl::PointCloud<PointType>>());

  pcl::transformPointCloud (*published_cloud, *deskewed_scan_t_, T_cloud);

  // published deskewed cloud
  sensor_msgs::PointCloud2 deskewed_ros;
  pcl::toROSMsg(*deskewed_scan_t_, deskewed_ros);
  deskewed_ros.header.stamp = this->scan_header_stamp;
  deskewed_ros.header.frame_id = this->odom_frame;
  this->deskewed_pub.publish(deskewed_ros);

}


/**
    Publish to ROS function
*/
void liom::NodeOdometry::publishToROS(pcl::PointCloud<PointType>::ConstPtr published_cloud, Eigen::Matrix4f T_cloud) {
  this->publishCloud(published_cloud, T_cloud);

  // nav_msgs::Path
  this->path_ros.header.stamp = this->imu_stamp;
  this->path_ros.header.frame_id = this->odom_frame;

  geometry_msgs::PoseStamped p;
  p.header.stamp = this->imu_stamp;
  p.header.frame_id = this->odom_frame;
  p.pose.position.x = this->state.p[0];
  p.pose.position.y = this->state.p[1];
  p.pose.position.z = this->state.p[2];
  p.pose.orientation.w = this->state.q.w();
  p.pose.orientation.x = this->state.q.x();
  p.pose.orientation.y = this->state.q.y();
  p.pose.orientation.z = this->state.q.z();

  this->path_ros.poses.push_back(p);
  this->path_pub.publish(this->path_ros);

  // transform: odom to baselink
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = this->imu_stamp;
  transformStamped.header.frame_id = this->odom_frame;
  transformStamped.child_frame_id = this->baselink_frame;

  transformStamped.transform.translation.x = this->state.p[0];
  transformStamped.transform.translation.y = this->state.p[1];
  transformStamped.transform.translation.z = this->state.p[2];

  transformStamped.transform.rotation.w = this->state.q.w();
  transformStamped.transform.rotation.x = this->state.q.x();
  transformStamped.transform.rotation.y = this->state.q.y();
  transformStamped.transform.rotation.z = this->state.q.z();

  br.sendTransform(transformStamped);

  // transform: baselink to imu
  transformStamped.header.stamp = this->imu_stamp;
  transformStamped.header.frame_id = this->baselink_frame;
  transformStamped.child_frame_id = this->imu_frame;

  transformStamped.transform.translation.x = this->extrinsics.baselink2imu.t[0];
  transformStamped.transform.translation.y = this->extrinsics.baselink2imu.t[1];
  transformStamped.transform.translation.z = this->extrinsics.baselink2imu.t[2];

  Eigen::Quaternionf q(this->extrinsics.baselink2imu.R);
  transformStamped.transform.rotation.w = q.w();
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();

  br.sendTransform(transformStamped);

  // transform: baselink to lidar
  transformStamped.header.stamp = this->imu_stamp;
  transformStamped.header.frame_id = this->baselink_frame;
  transformStamped.child_frame_id = this->lidar_frame;

  transformStamped.transform.translation.x = this->extrinsics.baselink2lidar.t[0];
  transformStamped.transform.translation.y = this->extrinsics.baselink2lidar.t[1];
  transformStamped.transform.translation.z = this->extrinsics.baselink2lidar.t[2];

  Eigen::Quaternionf qq(this->extrinsics.baselink2lidar.R);
  transformStamped.transform.rotation.w = qq.w();
  transformStamped.transform.rotation.x = qq.x();
  transformStamped.transform.rotation.y = qq.y();
  transformStamped.transform.rotation.z = qq.z();

  br.sendTransform(transformStamped);

}

/**
    Point Cloud callback function
**/
void liom::NodeOdometry::callbackPointCloud(const sensor_msgs::PointCloud2ConstPtr& pc) {


    // ROS_WARN(" Point Cloud Call back ");
    std::unique_lock<decltype(this->main_loop_running_mutex)> lock(main_loop_running_mutex);
    this->main_loop_running = true;
    lock.unlock();

    // Extract time
    double then = ros::Time::now().toSec();

    if (this->first_scan_stamp == 0) {
        this->first_scan_stamp = pc->header.stamp.toSec();
    }

    // Initialization procedures (IMU calib, gravity align)
    if (!this->liom_initialized) {
        this-> initializeLIOM();
    }

    // Convert incoming scan 
    this->getScanFromROS(pc);

    // Preprocess points
    this->preprocessPoints();

    if (!this->first_valid_scan) {
        // if the first scan is invalid
        return;
    }

    if (this->current_scan->points.size() <= this->gicp_min_num_points_) {
        ROS_FATAL("Low number of points in the cloud!");
        return;
    }

    // Compute metrics
    this->metrics_thread = std::thread( &liom::NodeOdometry::computeMetrics, this);
    this->metrics_thread.detach();

    // Set Adaptive Parameters
    if (this->adaptive_params_) {
        this->setAdaptiveParams();
    }

    // Set new frame as input source
    this->setInputSource();

    // Set initial frame as first keyframe
    if (this->keyframes.size() == 0) {
        this->initializeInputTarget();
        this->main_loop_running = false;
        this->submap_future = 
            std::async( std::launch::async, &liom::NodeOdometry::buildKeyframesAndSubmap, this, this->state);
        this->submap_future.wait(); // wait until completion
        return;
    }

    // Get the next pose via IMU + S2M + GEO
    this->getNextPose();


    // Update current keyframe poses and map
    this->updateKeyframes();

    // Build keyframe normals and submap if needed (and if we're not already waiting)
    if (this->new_submap_is_ready) {
        this->main_loop_running = false;
        this->submap_future =
        std::async( std::launch::async, &liom::NodeOdometry::buildKeyframesAndSubmap, this, this->state );
    } else {
        lock.lock();
        this->main_loop_running = false;
        lock.unlock();
        this->submap_build_cv.notify_one();
    }

    // Update trajectory
    this->trajectory.push_back( std::make_pair(this->state.p, this->state.q) );

    // Update time stamps
    this->lidar_rates.push_back( 1. / (this->scan_stamp - this->prev_scan_stamp) );
    this->prev_scan_stamp = this->scan_stamp;
    this->elapsed_time = this->scan_stamp - this->first_scan_stamp;

    // Publish stuff to ROS
    pcl::PointCloud<PointType>::ConstPtr published_cloud;
    if (this->densemap_filtered_) {
        published_cloud = this->current_scan;
    } else {
        published_cloud = this->deskewed_scan;
    }
    this->publish_thread = std::thread( &liom::NodeOdometry::publishToROS, this, published_cloud, this->T_corr );
    this->publish_thread.detach();

    // Update some statistics
    this->comp_times.push_back(ros::Time::now().toSec() - then);
    this->gicp_hasConverged = this->gicp.hasConverged();

    // // Debug statements and publish messages
    this->debug_thread = std::thread( &liom::NodeOdometry::debug, this );
    this->debug_thread.detach();

    this->geo.first_opt_done = true;
}

void liom::NodeOdometry::debug() {

  // Total length traversed
  double length_traversed = 0.;
  Eigen::Vector3f p_curr = Eigen::Vector3f(0., 0., 0.);
  Eigen::Vector3f p_prev = Eigen::Vector3f(0., 0., 0.);
  for (const auto& t : this->trajectory) {
    if (p_prev == Eigen::Vector3f(0., 0., 0.)) {
      p_prev = t.first;
      continue;
    }
    p_curr = t.first;
    double l = sqrt(pow(p_curr[0] - p_prev[0], 2) + pow(p_curr[1] - p_prev[1], 2) + pow(p_curr[2] - p_prev[2], 2));

    if (l >= 0.1) {
      length_traversed += l;
      p_prev = p_curr;
    }
  }
  this->length_traversed = length_traversed;

  // Average computation time
  double avg_comp_time =
    std::accumulate(this->comp_times.begin(), this->comp_times.end(), 0.0) / this->comp_times.size();

  // Average sensor rates
  int win_size = 100;
  double avg_imu_rate;
  double avg_lidar_rate;
  if (this->imu_rates.size() < win_size) {
    avg_imu_rate =
      std::accumulate(this->imu_rates.begin(), this->imu_rates.end(), 0.0) / this->imu_rates.size();
  } else {
    avg_imu_rate =
      std::accumulate(this->imu_rates.end()-win_size, this->imu_rates.end(), 0.0) / win_size;
  }
  if (this->lidar_rates.size() < win_size) {
    avg_lidar_rate =
      std::accumulate(this->lidar_rates.begin(), this->lidar_rates.end(), 0.0) / this->lidar_rates.size();
  } else {
    avg_lidar_rate =
      std::accumulate(this->lidar_rates.end()-win_size, this->lidar_rates.end(), 0.0) / win_size;
  }

  // RAM Usage
  double vm_usage = 0.0;
  double resident_set = 0.0;
  std::ifstream stat_stream("/proc/self/stat", std::ios_base::in); //get info from proc directory
  std::string pid, comm, state, ppid, pgrp, session, tty_nr;
  std::string tpgid, flags, minflt, cminflt, majflt, cmajflt;
  std::string utime, stime, cutime, cstime, priority, nice;
  std::string num_threads, itrealvalue, starttime;
  unsigned long vsize;
  long rss;
  stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr
              >> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt
              >> utime >> stime >> cutime >> cstime >> priority >> nice
              >> num_threads >> itrealvalue >> starttime >> vsize >> rss; // don't care about the rest
  stat_stream.close();
  long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // for x86-64 is configured to use 2MB pages
  vm_usage = vsize / 1024.0;
  resident_set = rss * page_size_kb;

  // CPU Usage
  struct tms timeSample;
  clock_t now;
  double cpu_percent;
  now = times(&timeSample);
  if (now <= this->lastCPU || timeSample.tms_stime < this->lastSysCPU ||
      timeSample.tms_utime < this->lastUserCPU) {
      cpu_percent = -1.0;
  } else {
      cpu_percent = (timeSample.tms_stime - this->lastSysCPU) + (timeSample.tms_utime - this->lastUserCPU);
      cpu_percent /= (now - this->lastCPU);
      cpu_percent /= this->numProcessors;
      cpu_percent *= 100.;
  }
  this->lastCPU = now;
  this->lastSysCPU = timeSample.tms_stime;
  this->lastUserCPU = timeSample.tms_utime;
  this->cpu_percents.push_back(cpu_percent);
  double avg_cpu_usage =
    std::accumulate(this->cpu_percents.begin(), this->cpu_percents.end(), 0.0) / this->cpu_percents.size();

  // Print to terminal
  printf("\033[2J\033[1;1H");

  std::time_t curr_time = this->scan_stamp;
  std::string asc_time = std::asctime(std::localtime(&curr_time)); asc_time.pop_back();
  std::cout << "| " << std::left << asc_time;
  std::cout << std::right << std::setfill(' ') << std::setw(42)
    << "Elapsed Time: " + to_string_with_precision(this->elapsed_time, 2) + " seconds "
    << "|" << std::endl;

  if ( !this->cpu_type.empty() ) {
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
      << this->cpu_type + " x " + std::to_string(this->numProcessors)
      << "|" << std::endl;
  }

  if (this->sensor == liom::SensorType::OUSTER) {
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
      << "Sensor Rates: Ouster @ " + to_string_with_precision(avg_lidar_rate, 2)
                                   + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
      << "|" << std::endl;
  } else if (this->sensor == liom::SensorType::VELODYNE) {
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
      << "Sensor Rates: Velodyne @ " + to_string_with_precision(avg_lidar_rate, 2)
                                     + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
      << "|" << std::endl;
  } else if (this->sensor == liom::SensorType::HESAI) {
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
      << "Sensor Rates: Hesai @ " + to_string_with_precision(avg_lidar_rate, 2)
                                  + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
      << "|" << std::endl;
  } else if (this->sensor == liom::SensorType::LIVOX) {
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
      << "Sensor Rates: Livox @ " + to_string_with_precision(avg_lidar_rate, 2)
                                  + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
      << "|" << std::endl;
  } else {
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
      << "Sensor Rates: Unknown LiDAR @ " + to_string_with_precision(avg_lidar_rate, 2)
                                          + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
      << "|" << std::endl;
  }

  std::cout << "|===================================================================|" << std::endl;

  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Position       [xyz] :: " + to_string_with_precision(this->state.p[0], 4) + " "
                                + to_string_with_precision(this->state.p[1], 4) + " "
                                + to_string_with_precision(this->state.p[2], 4)
    << "|" << std::endl;
  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Orientation   [wxyz] :: " + to_string_with_precision(this->state.q.w(), 4) + " "
                                + to_string_with_precision(this->state.q.x(), 4) + " "
                                + to_string_with_precision(this->state.q.y(), 4) + " "
                                + to_string_with_precision(this->state.q.z(), 4)
    << "|" << std::endl;
  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Lin Velocity   [xyz] :: " + to_string_with_precision(this->state.v.linear.b[0], 4) + " "
                                + to_string_with_precision(this->state.v.linear.b[1], 4) + " "
                                + to_string_with_precision(this->state.v.linear.b[2], 4)
    << "|" << std::endl;
  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Ang Velocity   [xyz] :: " + to_string_with_precision(this->state.v.angl.b[0], 4) + " "
                                + to_string_with_precision(this->state.v.angl.b[1], 4) + " "
                                + to_string_with_precision(this->state.v.angl.b[2], 4)
    << "|" << std::endl;
  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Accel Bias        [xyz] :: " + to_string_with_precision(this->state.b.accel[0], 8) + " "
                                + to_string_with_precision(this->state.b.accel[1], 8) + " "
                                + to_string_with_precision(this->state.b.accel[2], 8)
    << "|" << std::endl;
  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Gyro Bias         [xyz] :: " + to_string_with_precision(this->state.b.gyro[0], 8) + " "
                                + to_string_with_precision(this->state.b.gyro[1], 8) + " "
                                + to_string_with_precision(this->state.b.gyro[2], 8)
    << "|" << std::endl;

  std::cout << "|                                                                   |" << std::endl;

  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Distance Traveled  :: " + to_string_with_precision(length_traversed, 4) + " meters"
    << "|" << std::endl;
  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Distance to Origin :: "
      + to_string_with_precision( sqrt(pow(this->state.p[0]-this->origin[0],2) +
                                       pow(this->state.p[1]-this->origin[1],2) +
                                       pow(this->state.p[2]-this->origin[2],2)), 4) + " meters"
    << "|" << std::endl;
  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Registration       :: keyframes: " + std::to_string(this->keyframes.size()) + ", "
                               + "deskewed points: " + std::to_string(this->deskew_size)
    << "|" << std::endl;
  std::cout << "|                                                                   |" << std::endl;

  std::cout << std::right << std::setprecision(2) << std::fixed;
  std::cout << "| Computation Time :: "
    << std::setfill(' ') << std::setw(6) << this->comp_times.back()*1000. << " ms    // Avg: "
    << std::setw(6) << avg_comp_time*1000. << " / Max: "
    << std::setw(6) << *std::max_element(this->comp_times.begin(), this->comp_times.end())*1000.
    << "     |" << std::endl;
  std::cout << "| Cores Utilized   :: "
    << std::setfill(' ') << std::setw(6) << (cpu_percent/100.) * this->numProcessors << " cores // Avg: "
    << std::setw(6) << (avg_cpu_usage/100.) * this->numProcessors << " / Max: "
    << std::setw(6) << (*std::max_element(this->cpu_percents.begin(), this->cpu_percents.end()) / 100.)
                       * this->numProcessors
    << "     |" << std::endl;
  std::cout << "| CPU Load         :: "
    << std::setfill(' ') << std::setw(6) << cpu_percent << " %     // Avg: "
    << std::setw(6) << avg_cpu_usage << " / Max: "
    << std::setw(6) << *std::max_element(this->cpu_percents.begin(), this->cpu_percents.end())
    << "     |" << std::endl;
  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "RAM Allocation   :: " + to_string_with_precision(resident_set/1000., 2) + " MB"
    << "|" << std::endl;

  std::cout << "+-------------------------------------------------------------------+" << std::endl;

}


