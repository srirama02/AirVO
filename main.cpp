#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include "read_configs.h"
#include "dataset.h"
#include "map_builder.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("air_vo");

  std::string config_path, model_dir;
  node->get_parameter_or("config_path", config_path, std::string("default_config_path"));
  node->get_parameter_or("model_dir", model_dir, std::string("default_model_dir"));
  Configs configs(config_path, model_dir);
  node->get_parameter_or("dataroot", configs.dataroot, std::string("default_dataroot"));
  node->get_parameter_or("camera_config_path", configs.camera_config_path, std::string("default_camera_config_path"));
  node->get_parameter_or("saving_dir", configs.saving_dir, std::string("default_saving_dir"));
  std::string traj_path;
  node->get_parameter_or("traj_path", traj_path, std::string("default_traj_path"));

  Dataset dataset(configs.dataroot);
  MapBuilder map_builder(configs);
  size_t dataset_length = dataset.GetDatasetLength();
  for(size_t i = 0; i < dataset_length && rclcpp::ok(); ++i){
    std::cout << "i ===== " << i << std::endl;
    auto before_infer = std::chrono::steady_clock::now();

    InputDataPtr input_data = dataset.GetData(i);
    if(input_data == nullptr) continue;
    map_builder.AddInput(input_data);

    auto after_infer = std::chrono::steady_clock::now();
    auto cost_time = std::chrono::duration_cast<std::chrono::milliseconds>(after_infer - before_infer).count();
    std::cout << "One Frame Processing Time: " << cost_time << " ms." << std::endl;
  }
  map_builder.ShutDown();
  map_builder.SaveTrajectory(traj_path);
  rclcpp::shutdown();

  return 0;
}
