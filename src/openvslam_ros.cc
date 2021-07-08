#include <openvslam_ros.h>

#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <openvslam/publish/map_publisher.h>

#include <Eigen/Geometry>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>

namespace openvslam_ros {
system::system(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path)
    : SLAM_(cfg, vocab_file_path), cfg_(cfg), private_nh_("~"), it_(nh_), tp_0_(std::chrono::steady_clock::now()),
      mask_(mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE)),
      pose_pub_(private_nh_.advertise<nav_msgs::Odometry>("camera_pose", 1)),
      graph_pub_(private_nh_.advertise<nav_msgs::Path>("/map/graph", 1)),
      map_pub_(private_nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1)),
      map_to_odom_broadcaster_(),
      tf_(),
      tf_listener_(tf_)
      {time1_ = std::chrono::steady_clock::now();}

void system::publish_pose(const Eigen::Matrix4d& cam_pose_wc) {
    // Extract rotation matrix and translation vector from
   Eigen::Matrix3d rot = cam_pose_wc.block<3, 3>(0, 0);
    Eigen::Vector3d trans = cam_pose_wc.block<3, 1>(0, 3);
    Eigen::Matrix3d cv_to_ros;
    cv_to_ros << 0, 0, 1,
        -1, 0, 0,
        0, -1, 0;

    // Transform from CV coordinate system to ROS coordinate system on camera coordinates
    Eigen::Quaterniond quat(cv_to_ros * rot * cv_to_ros.transpose());
    trans = cv_to_ros * trans;
/*
    // Create odometry message and update it with current camera pose
  nav_msgs::Odometry pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "map";
    pose_msg.child_frame_id = "odom";
    pose_msg.pose.pose.orientation.x = quat.x();
    pose_msg.pose.pose.orientation.y = quat.y();
    pose_msg.pose.pose.orientation.z = quat.z();
    pose_msg.pose.pose.orientation.w = quat.w();
    pose_msg.pose.pose.position.x = trans(0);
    pose_msg.pose.pose.position.y = trans(1);
    pose_msg.pose.pose.position.z = trans(2);
    pose_pub_.publish(pose_msg);
    bool publish_tf = true;
    std::string camera_link_ = "usb_cam";
    std::string odom_frame_ = "odom";
    std::string map_frame_ = "map";
    transform_tolerance_ = 0.5;
    if(publish_tf){
            auto camera_to_odom = tf_->lookupTransform(
                camera_link_, odom_frame_, tf2_ros::fromMsg(builtin_interfaces::msg::Time(stamp)),
                tf2::durationFromSec(0.0));
            Eigen::Affine3d camera_to_odom_affine = tf2::transformToEigen(camera_to_odom.transform);

            auto map_to_odom_msg = tf2::eigenToTransform(map_to_camera_affine * camera_to_odom_affine);
                       auto map_to_odom_msg = tf2::eigenToTransform(map_to_camera_affine * camera_to_odom_affine);
                        tf2::TimePoint transform_timestamp = tf2_ros::fromMsg(stamp) + tf2::durationFromSec(transform_tolerance_);
                        map_to_odom_msg.header.stamp = tf2_ros::toMsg(transform_timestamp);
                        map_to_odom_msg.header.frame_id = map_frame_;
                        map_to_odom_msg.child_frame_id = odom_frame_;
                map_to_odom_broadcaster_.sendTransform(map_to_odom_msg);
    }
*/
  //  Eigen::Matrix3d rot(cam_pose_wc.block<3, 3>(0, 0));
    Eigen::Translation3d trans2(cam_pose_wc.block<3, 1>(0, 3));
    Eigen::Affine3d map_to_camera_affine(trans2 * rot);
    Eigen::Matrix3d rot_ros_to_cv_map_frame;
    rot_ros_to_cv_map_frame << 0, 0, 1,
        -1, 0, 0,
        0, -1, 0;

    // Transform map frame from CV coordinate system to ROS coordinate system
    map_to_camera_affine.prerotate(rot_ros_to_cv_map_frame);
    bool publish_tf = true;
    std::string camera_link_ = "usb_cam";
    std::string odom_frame_ = "odom";
    std::string map_frame_ = "map";
    // Create odometry message and update it with current camera pose
   nav_msgs::Odometry pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = map_frame_;
    pose_msg.child_frame_id = camera_link_;
  pose_msg.pose.pose.orientation.x = quat.x();
    pose_msg.pose.pose.orientation.y = quat.y();
    pose_msg.pose.pose.orientation.z = quat.z();
    pose_msg.pose.pose.orientation.w = quat.w();
    pose_msg.pose.pose.position.x = trans(0);
    pose_msg.pose.pose.position.y = trans(1);
    pose_msg.pose.pose.position.z = trans(2);
   // pose_msg.pose.pose = tf2::toMsg(map_to_camera_affine);
    pose_pub_.publish(pose_msg);

    transform_tolerance_ = 0.5;
    // Send map->odom transform. Set publish_tf to false if not using TF
    if (publish_tf) {
        try {
       //     tf_.setUsingDedicatedThread (true);
         //   tf_.canTransform(camera_link_, odom_frame_,ros::Time(0), ros::Duration(0.0));
            auto camera_to_odom = tf_.lookupTransform(
                camera_link_, odom_frame_, ros::Time(0), ros::Duration(0.0));
            Eigen::Affine3d camera_to_odom_affine = tf2::transformToEigen(camera_to_odom.transform);

            auto map_to_odom_msg = tf2::eigenToTransform(map_to_camera_affine * camera_to_odom_affine);
          //  tf2::TimePoint transform_timestamp = tf2_ros::fromMsg(ros::Time::now()) + tf2::durationFromSec(transform_tolerance_);
            map_to_odom_msg.header.stamp = ros::Time::now();
            map_to_odom_msg.header.frame_id = map_frame_;
            map_to_odom_msg.child_frame_id = odom_frame_;
            map_to_odom_broadcaster_.sendTransform(map_to_odom_msg);
        }
        catch (tf2::TransformException& ex) {
            ROS_ERROR( "Transform failed: %s", ex.what());
            //RCLCPP_ERROR(node_->get_logger(), "Transform failed: %s", ex.what());
        }
    }

}

mono::mono(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path)
    : system(cfg, vocab_file_path, mask_img_path) {
    sub_ = it_.subscribe("camera/image_raw", 1, &mono::callback, this);
}
void mono::callback(const sensor_msgs::ImageConstPtr& msg) {
    const auto tp_1 = std::chrono::steady_clock::now();
    const auto timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - tp_0_).count();

    // input the current frame and estimate the camera pose
    auto cam_pose_wc = SLAM_.feed_monocular_frame(cv_bridge::toCvShare(msg)->image, timestamp, mask_);

    const auto tp_2 = std::chrono::steady_clock::now();

    const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
    track_times_.push_back(track_time);

    if (cam_pose_wc) {
        publish_pose(*cam_pose_wc);
    }

    // publish occupancy grid map and path
    double freq = 1.0;
     std::chrono::duration<double, std::milli> diff = tp_2-time1_;
    //const auto track_time2 = std::chrono::duration_cast<std::chrono::milliseconds>>(tp_2 - time1_).count();
    
    if(diff.count() < 1.0/ freq * 1000){
       return;
    }
    time1_ = tp_2;
    std::vector<openvslam::data::keyframe*> all_keyfrms;
    unsigned int count = SLAM_.get_map_publisher()->get_keyframes(all_keyfrms);
     ROS_INFO_THROTTLE(10, "Number keyframes: %d", count);
     nav_msgs::Path path;
     path.header.stamp = ros::Time::now();
     path.header.frame_id = "map";
     Eigen::Matrix3d cv_to_ros;
     cv_to_ros << 0, 0, 1,
        -1, 0, 0,
        0, -1, 0;
    double max_x = 0.0, max_y = 0.0 , min_x = 0.0, min_y = 0.0;
    std::sort(all_keyfrms.begin(), all_keyfrms.end(),comparePtrToKeyframe);
    for(auto iter : all_keyfrms){
      //  ROS_INFO( "Keyframe id : %d", iter->id_);
        geometry_msgs::PoseStamped pose;
        pose.header = path.header;
        Eigen::Vector3d vec = cv_to_ros* iter->get_cam_center();
        pose.pose.position.x = vec(0);
        if (vec(0) > max_x)  max_x = vec(0);
        if (vec(0) < min_x)  min_x = vec(0);
        pose.pose.position.y = vec(1);
        if (vec(1) > max_y)  max_y = vec(1);
        if (vec(1) < min_y)  min_y = vec(1);
        pose.pose.position.z = vec(2);
        path.poses.push_back(pose);
    }
    graph_pub_.publish(path);
    ROS_INFO_THROTTLE(10, "max_x: %f , max_y: %f , min_x: %f , min_y: %f ", max_x, max_y, min_x, min_y );
    nav_msgs::OccupancyGrid map;
    map.header = path.header;
    double resolution = 0.05;
    map.info.resolution = resolution;
    double center_x = abs(max_x - min_x);
    double center_y = abs(max_y - min_y);
    map.info.width = (center_x + ( 1.5 * 2)) / resolution;//( max_x + min_x) / 2.0 + ( 1.5 * 2);
    map.info.height = (center_y+ ( 1.5 * 2)) / resolution;//(max_y + min_y) / 2.0 + (1.5 * 2) ;
    map.info.origin.position.x =  (max_x - center_x /2.0) - map.info.width  / 2.0 * resolution;
    map.info.origin.position.y =  (max_y - center_y / 2.0) - map.info.height / 2.0 * resolution;
    map.info.origin.position.z = 0.0;
    //TODO get quaternion
    for(unsigned int i = 0; i < map.info.width; i++){
        for(unsigned int j = 0; j < map.info.height; j++){
            map.data.push_back(253);
        }
    }

    map_pub_.publish(map);
   
}

stereo::stereo(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path,
               const bool rectify)
    : system(cfg, vocab_file_path, mask_img_path),
      rectifier_(rectify ? std::make_shared<openvslam::util::stereo_rectifier>(cfg) : nullptr),
      left_sf_(it_, "camera/left/image_raw", 1),
      right_sf_(it_, "camera/right/image_raw", 1),
      sync_(SyncPolicy(10), left_sf_, right_sf_) {
    sync_.registerCallback(&stereo::callback, this);
}

void stereo::callback(const sensor_msgs::ImageConstPtr& left, const sensor_msgs::ImageConstPtr& right) {
    auto leftcv = cv_bridge::toCvShare(left)->image;
    auto rightcv = cv_bridge::toCvShare(right)->image;
    if (leftcv.empty() || rightcv.empty()) {
        return;
    }

    if (rectifier_) {
        rectifier_->rectify(leftcv, rightcv, leftcv, rightcv);
    }

    const auto tp_1 = std::chrono::steady_clock::now();
    const auto timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - tp_0_).count();

    // input the current frame and estimate the camera pose
    auto cam_pose_wc = SLAM_.feed_stereo_frame(leftcv, rightcv, timestamp, mask_);

    const auto tp_2 = std::chrono::steady_clock::now();

    const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
    track_times_.push_back(track_time);

    if (cam_pose_wc) {
        publish_pose(*cam_pose_wc);
    }
}

rgbd::rgbd(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path)
    : system(cfg, vocab_file_path, mask_img_path),
      color_sf_(it_, "camera/color/image_raw", 1),
      depth_sf_(it_, "camera/depth/image_raw", 1),
      sync_(SyncPolicy(10), color_sf_, depth_sf_) {
    sync_.registerCallback(&rgbd::callback, this);
}

void rgbd::callback(const sensor_msgs::ImageConstPtr& color, const sensor_msgs::ImageConstPtr& depth) {
    auto colorcv = cv_bridge::toCvShare(color)->image;
    auto depthcv = cv_bridge::toCvShare(depth)->image;
    if (colorcv.empty() || depthcv.empty()) {
        return;
    }

    const auto tp_1 = std::chrono::steady_clock::now();
    const auto timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - tp_0_).count();

    // input the current frame and estimate the camera pose
    auto cam_pose_wc = SLAM_.feed_RGBD_frame(colorcv, depthcv, timestamp, mask_);

    const auto tp_2 = std::chrono::steady_clock::now();

    const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
    track_times_.push_back(track_time);

    if (cam_pose_wc) {
        publish_pose(*cam_pose_wc);
    }
}
} // namespace openvslam_ros
