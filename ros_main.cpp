#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

// AirVO
#include "read_configs.h"
#include "dataset.h"
#include "map_builder.h"

using namespace std::chrono_literals;
using namespace message_filters;

MapBuilder* p_map_builder;

void GrabStereo(const sensor_msgs::msg::Image::ConstSharedPtr& imgLeft, const sensor_msgs::msg::Image::ConstSharedPtr& imgRight){
    // Copy the ROS image messages to cv::Mat
    cv_bridge::CvImageConstPtr cv_ptrLeft, cv_ptrRight;
    try{
        cv_ptrLeft = cv_bridge::toCvShare(imgLeft);
        cv_ptrRight = cv_bridge::toCvShare(imgRight);
    }
    catch (cv_bridge::Exception& e){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cv_bridge exception: %s", e.what());
        return;
    }

    static int frame_id = 0;
    auto before_infer = std::chrono::steady_clock::now();

    InputDataPtr input_data = std::shared_ptr<InputData>(new InputData());
    input_data->index = frame_id;
    input_data->image_left = cv_ptrLeft->image.clone();
    input_data->image_right = cv_ptrRight->image.clone();
    input_data->time = rclcpp::Time(imgLeft->header.stamp).seconds();

    if(input_data == nullptr) return;
    p_map_builder->AddInput(input_data);

    auto after_infer = std::chrono::steady_clock::now();
    auto cost_time = std::chrono::duration_cast<std::chrono::milliseconds>(after_infer - before_infer).count();

    std::cout << "i ===== " << frame_id++ << " Processing Time: " << cost_time << " ms." << std::endl;
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("air_vo_ros");

    std::string left_topic, right_topic;
    node->get_parameter_or("left_topic", left_topic, std::string("/camera/left/image_raw"));
    node->get_parameter_or("right_topic", right_topic, std::string("/camera/right/image_raw"));

    message_filters::Subscriber<sensor_msgs::msg::Image> sub_img_left(node.get(), left_topic, 1);
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_img_right(node.get(), right_topic, 1);

    typedef sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync_pol;
    Synchronizer<sync_pol> sync(sync_pol(10), sub_img_left, sub_img_right);
    sync.registerCallback(&GrabStereo);

    // AirVO
    std::string config_path, model_dir;
    node->get_parameter_or("config_path", config_path, std::string("default_config_path"));
    node->get_parameter_or("model_dir", model_dir, std::string("default_model_dir"));
    Configs configs(config_path, model_dir);
    node->get_parameter_or("camera_config_path", configs.camera_config_path, std::string("default_camera_config_path"));
    node->get_parameter_or("saving_dir", configs.saving_dir, std::string("default_saving_dir"));
    std::string traj_path;
    node->get_parameter_or("traj_path", traj_path, std::string("default_traj_path"));

    p_map_builder = new MapBuilder(configs);

    // Starts the operation
    rclcpp::spin(node);

    // Shutting down
    std::cout << "Saving trajectory to " << traj_path << std::endl;
    p_map_builder->SaveTrajectory(traj_path);

    rclcpp::shutdown();

    return 0;
}
