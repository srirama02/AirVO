#include "ros_publisher.h"

#include <Eigen/Geometry>

#include "utils.h"

double GetCurrentTime() {
    return rclcpp::Clock().now().seconds();
}

RosPublisher::RosPublisher(const RosPublisherConfig& ros_publisher_config)
: Node("ros_publisher"), _config(ros_publisher_config) {
    if (_config.feature) {
        _ros_feature_pub = this->create_publisher<sensor_msgs::msg::Image>(_config.feature_topic, 10);
        auto publish_feature_function =
            [this](const FeatureMessgaeConstPtr& feature_message) {
                cv::Mat drawed_image = DrawFeatures(feature_message->image, feature_message->keypoints,
                                                    feature_message->inliers, feature_message->lines, feature_message->line_track_ids,
                                                    feature_message->points_on_lines);
                auto ros_feature_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", drawed_image).toImageMsg();
                ros_feature_msg->header.stamp = rclcpp::Time(feature_message->time);
                _ros_feature_pub->publish(*ros_feature_msg);
            };
        _feature_publisher.Register(publish_feature_function);
        _feature_publisher.Start();
    }

    if (_config.frame_pose) {
        _ros_frame_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(_config.frame_pose_topic, 10);
        auto publish_frame_pose_function =
            [this](const FramePoseMessageConstPtr& frame_pose_message) {
                geometry_msgs::msg::PoseStamped pose_stamped;
                pose_stamped.header.stamp = rclcpp::Time(frame_pose_message->time);
                pose_stamped.header.frame_id = "map";
                pose_stamped.pose.position.x = frame_pose_message->pose(0, 3);
                pose_stamped.pose.position.y = frame_pose_message->pose(1, 3);
                pose_stamped.pose.position.z = frame_pose_message->pose(2, 3);
                Eigen::Quaterniond q(frame_pose_message->pose.block<3, 3>(0, 0));
                pose_stamped.pose.orientation.x = q.x();
                pose_stamped.pose.orientation.y = q.y();
                pose_stamped.pose.orientation.z = q.z();
                pose_stamped.pose.orientation.w = q.w();
                _ros_frame_pose_pub->publish(pose_stamped);
            };
        _frame_pose_publisher.Register(publish_frame_pose_function);
        _frame_pose_publisher.Start();
    }

    if (_config.keyframe) {
        _ros_keyframe_pub = this->create_publisher<geometry_msgs::msg::PoseArray>(_config.keyframe_topic, 10);
        _ros_path_pub = this->create_publisher<nav_msgs::msg::Path>(_config.path_topic, 10);

        auto publish_keyframe_function =
            [this](const KeyframeMessageConstPtr& keyframe_message) {
                for (int i = 0; i < keyframe_message->ids.size(); i++) {
                    int keyframe_id = keyframe_message->ids[i];

                    geometry_msgs::msg::Pose pose;
                    pose.position.x = keyframe_message->poses[i](0, 3);
                    pose.position.y = keyframe_message->poses[i](1, 3);
                    pose.position.z = keyframe_message->poses[i](2, 3);
                    Eigen::Quaterniond q(keyframe_message->poses[i].block<3, 3>(0, 0));
                    pose.orientation.x = q.x();
                    pose.orientation.y = q.y();
                    pose.orientation.z = q.z();
                    pose.orientation.w = q.w();

                    geometry_msgs::msg::PoseStamped pose_stamped;
                    pose_stamped.header.stamp = rclcpp::Time(keyframe_message->times[i]);
                    pose_stamped.pose = pose;

                    auto it = _keyframe_id_to_index.find(keyframe_id);
                    if (it == _keyframe_id_to_index.end()) {
                        _ros_keyframe_array.poses.push_back(pose);
                        _ros_path.poses.push_back(pose_stamped);
                        _keyframe_id_to_index[keyframe_id] = _ros_keyframe_array.poses.size() - 1;
                    } else {
                        int idx = it->second;
                        _ros_keyframe_array.poses[idx] = pose;
                        _ros_path.poses[idx] = pose_stamped;
                    }
                }
                _ros_keyframe_pub->publish(_ros_keyframe_array);
                _ros_path_pub->publish(_ros_path);
            };
        _keyframe_publisher.Register(publish_keyframe_function);
        _keyframe_publisher.Start();
    }

    if (_config.map) {
        _ros_map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(_config.map_topic, 1);

        auto publish_map_function =
            [this](const MapMessageConstPtr& map_message) {
                for (int i = 0; i < map_message->ids.size(); i++) {
                    int mappoint_id = map_message->ids[i];
                    auto it = _mappoint_id_to_index.find(mappoint_id);
                    if (it == _mappoint_id_to_index.end()) {
                        geometry_msgs::msg::Point32 point;
                        point.x = map_message->points ;
                        point.y = map_message->points ;
                        point.z = map_message->points ;
                        _ros_mappoints.points.push_back(point);
                        _mappoint_id_to_index[mappoint_id] = _ros_mappoints.points.size() - 1;
                    } else {
                        int idx = it->second;
                        _ros_mappoints.points[idx].x = map_message->points[i](0);
                        _ros_mappoints.points[idx].y = map_message->points[i](1);
                        _ros_mappoints.points[idx].z = map_message->points[i](2);
                    }
                }
                _ros_map_pub->publish(_ros_mappoints);
            };
        _map_publisher.Register(publish_map_function);
        _map_publisher.Start();

        _ros_mapline_pub = this->create_publisher<visualization_msgs::msg::Marker>(_config.mapline_topic, 1);

        auto publish_mapline_function =
            [this](const MapLineMessageConstPtr& mapline_message) {
                for (int i = 0; i < mapline_message->ids.size(); i++) {
                    int mapline_id = mapline_message->ids[i];
                    auto it = _mapline_id_to_index.find(mapline_id);
                    if (it == _mapline_id_to_index.end()) {
                        geometry_msgs::msg::Point point1, point2;
                        point1.x = mapline_message->lines ;
                        point1.y = mapline_message->lines ;
                        point1.z = mapline_message->lines ;
                        point2.x = mapline_message->lines ;
                        point2.y = mapline_message->lines ;
                        point2.z = mapline_message->lines ;
                        _ros_maplines.points.push_back(point1);
                        _ros_maplines.points.push_back(point2);

                        std_msgs::msg::ColorRGBA color;
                        Eigen::Vector3d color_vector;
                        GenerateColor(mapline_id, color_vector);
                        color.r = 0.0;
                        color.g = 0.0;
                        color.b = 1.0;
                        color.a = 1.0;
                        _ros_maplines.colors.push_back(color);
                        _ros_maplines.colors.push_back(color);
                        _mapline_id_to_index[mapline_id] = _ros_maplines.points.size() - 2;
                    } else {
                        int idx = it->second;
                        _ros_maplines.points[idx].x = mapline_message->lines[i](0);
                        _ros_maplines.points[idx].y = mapline_message->lines[i](1);
                        _ros_maplines.points[idx].z = mapline_message->lines[i](2);
                        _ros_maplines.points[idx + 1].x = mapline_message->lines[i](3);
                        _ros_maplines.points[idx + 1].y = mapline_message->lines[i](4);
                        _ros_maplines.points[idx + 1].z = mapline_message->lines[i](5);
                    }
                }
                _ros_mapline_pub->publish(_ros_maplines);
            };
        _mapline_publisher.Register(publish_mapline_function);
        _mapline_publisher.Start();
    }
}

void RosPublisher::PublishFeature(FeatureMessgaePtr feature_message) {
    _feature_publisher.Publish(feature_message);
}

void RosPublisher::PublishFramePose(FramePoseMessagePtr frame_pose_message) {
    _frame_pose_publisher.Publish(frame_pose_message);
}

void RosPublisher::PublisheKeyframe(KeyframeMessagePtr keyframe_message) {
    _keyframe_publisher.Publish(keyframe_message);
}

void RosPublisher::PublishMap(MapMessagePtr map_message) {
    _map_publisher.Publish(map_message);
}

void RosPublisher::PublishMapLine(MapLineMessagePtr mapline_message) {
    _mapline_publisher.Publish(mapline_message);
}