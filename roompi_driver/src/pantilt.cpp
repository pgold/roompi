
#include <cmath>

#include <algorithm>
#include <functional>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include "roompi_interfaces/msg/pan_tilt.hpp"

using std::placeholders::_1;

struct ChannelMapping
{
  double clip_min;
  double clip_max;
  int32_t bias;
  double scale;
};

// TODO(pgold): use std::clamp once C++17 is available.
template <typename T>
T clamp(const T& x, const T& lo, const T& hi)
{
  return std::max(lo, std::min(x, hi));
}

int32_t servo_value(int32_t bias, double scale, double theta)
{
  return bias + std::lround(scale * theta);
}

int32_t map_angle(double theta, const ChannelMapping& mapping)
{
  double clipped_theta = clamp(theta, mapping.clip_min, mapping.clip_max);
  return servo_value(mapping.bias, mapping.scale, clipped_theta);
}

class PanTilt : public rclcpp::Node
{
public:
  PanTilt() : Node("pan_tilt")
  {
    // Channel selection parameters.
    pan_channel_ = this->declare_parameter<int>("pan_channel", 0);
    tilt_channel_ = this->declare_parameter<int>("tilt_channel", 1);

    // Pan mapping parameters.
    pan_mapping_.clip_min = this->declare_parameter<double>("pan_clip_min", -90.0);
    pan_mapping_.clip_max = this->declare_parameter<double>("pan_clip_max", 90.0);
    pan_mapping_.bias = this->declare_parameter<int32_t>("pan_bias", 307);
    pan_mapping_.scale = this->declare_parameter<double>("pan_scale", 1.13);
    // Tilt mapping parameters.
    tilt_mapping_.clip_min = this->declare_parameter<double>("tilt_clip_min", -90.0);
    tilt_mapping_.clip_max = this->declare_parameter<double>("tilt_clip_max", 90.0);
    tilt_mapping_.bias = this->declare_parameter<int32_t>("tilt_bias", 307);
    tilt_mapping_.scale = this->declare_parameter<double>("tilt_scale", 1.13);

    pca_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("command", 10);
    pantilt_sub_ = this->create_subscription<roompi_interfaces::msg::PanTilt>(
        "pan_tilt", 1, std::bind(&PanTilt::on_pantilt, this, _1));
  }

private:
  void on_pantilt(roompi_interfaces::msg::PanTilt::SharedPtr pantilt)
  {
    auto message = std_msgs::msg::Int32MultiArray();
    message.data = std::vector<int32_t>(16, -1);
    message.data[pan_channel_] = map_angle(pantilt->pan, pan_mapping_);
    message.data[tilt_channel_] = map_angle(pantilt->tilt, tilt_mapping_);
    pca_pub_->publish(message);
  }

  int pan_channel_;
  int tilt_channel_;

  ChannelMapping pan_mapping_;
  ChannelMapping tilt_mapping_;

  rclcpp::Subscription<roompi_interfaces::msg::PanTilt>::SharedPtr pantilt_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pca_pub_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PanTilt>());
  rclcpp::shutdown();
  return 0;
}
