#ifndef ROS_PUBLISHER_H_
#define ROS_PUBLISHER_H_

#include <map>
#include <vector>
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "utils.h"
#include "read_configs.h"
#include "thread_publisher.h"

struct FeatureMessgae{
  double time;
  cv::Mat image;
  std::vector<bool> inliers;
  std::vector<cv::KeyPoint> keypoints;
  std::vector<Eigen::Vector4d> lines;
  std::vector<int> line_track_ids;
  std::vector<std::map<int, double>> points_on_lines;
};
typedef std::shared_ptr<FeatureMessgae> FeatureMessgaePtr;
typedef std::shared_ptr<const FeatureMessgae> FeatureMessgaeConstPtr;

struct FramePoseMessage{
  double time;
  Eigen::Matrix4d pose;
};
typedef std::shared_ptr<FramePoseMessage> FramePoseMessagePtr;
typedef std::shared_ptr<const FramePoseMessage> FramePoseMessageConstPtr;

struct KeyframeMessage{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::vector<double> times;
  std::vector<int> ids;
  std::vector<Eigen::Matrix4d> poses;
};
typedef std::shared_ptr<KeyframeMessage> KeyframeMessagePtr;
typedef std::shared_ptr<const KeyframeMessage> KeyframeMessageConstPtr;

struct MapMessage{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  double time;
  bool reset;
  std::vector<int> ids;
  std::vector<Eigen::Vector3d> points;
};
typedef std::shared_ptr<MapMessage> MapMessagePtr;
typedef std::shared_ptr<const MapMessage> MapMessageConstPtr;

struct MapLineMessage{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  double time;
  bool reset;
  std::vector<int> ids;
  std::vector<Vector6d> lines;
};
typedef std::shared_ptr<MapLineMessage> MapLineMessagePtr;
typedef std::shared_ptr<const MapLineMessage> MapLineMessageConstPtr;

class RosPublisher : public rclcpp::Node {
public:
  RosPublisher(const RosPublisherConfig& ros_publisher_config);

  void PublishFeature(FeatureMessgaePtr feature_message);
  void PublishFramePose(FramePoseMessagePtr frame_pose_message);
  void PublisheKeyframe(KeyframeMessagePtr keyframe_message);
  void PublishMap(MapMessagePtr map_message);
  void PublishMapLine(MapLineMessagePtr mapline_message);

private:
  RosPublisherConfig _config;

  // for publishing features
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr _ros_feature_pub;
  ThreadPublisher<FeatureMessgae> _feature_publisher;

  // for publishing frame
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _ros_frame_pose_pub;
  ThreadPublisher<FramePoseMessage> _frame_pose_publisher;

  // for publishing keyframes
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr _ros_keyframe_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _ros_path_pub;
  std::map<int, int> _keyframe_id_to_index;
  geometry_msgs::msg::PoseArray _ros_keyframe_array;
  nav_msgs::msg::Path _ros_path;
  ThreadPublisher<KeyframeMessage> _keyframe_publisher;

  // for publishing mappoints
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr _ros_map_pub;
  std::unordered_map<int, int> _mappoint_id_to_index;
  sensor_msgs::msg::PointCloud _ros_mappoints;
  ThreadPublisher<MapMessage> _map_publisher;

  // for publishing maplines
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _ros_mapline_pub;
  std::unordered_map<int, int> _mapline_id_to_index;
  visualization_msgs::msg::Marker _ros_maplines;
  ThreadPublisher<MapLineMessage> _mapline_publisher;
};
typedef std::shared_ptr<RosPublisher> RosPublisherPtr;

#endif  // ROS_PUBLISHER_H_
