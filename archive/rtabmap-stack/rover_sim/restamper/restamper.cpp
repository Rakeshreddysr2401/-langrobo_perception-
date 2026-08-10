// rover_sim restamper — C++ stage of the D555 impersonation
// (SIM_REAL_PARITY.md §3). Exists because:
//   * the contract requires WALL-clock stamps, but gz --initial-sim-time
//     with an epoch-sized value silently breaks sensor scheduling
//     (found live 2026-07-17), so gz stamps start at 0 and must be replaced;
//   * a python hop for 896x504 images caps at ~5Hz on this laptop —
//     the image path has to stay compiled end-to-end.
//
// In:  /rover_sim/{infra1,color}/{image,camera_info}   (ros_gz bridge)
//      /rover_sim/depth/{image,camera_info}            (32FC1, metres)
// Out: /camera/camera0/infra1/image_rect_raw + camera_info
//      /camera/camera0/depth/image_rect_raw  + camera_info   (16UC1, mm)
//      /camera/camera0/color/image_raw       + camera_info
// Each image gets stamp=now(); its cached camera_info is re-published with
// the SAME stamp (the real driver pairs them identically). frame_ids pass
// through from the sensors' gz_frame_id (= contract optical frames).
#include <cmath>
#include <cstring>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;

class Restamper : public rclcpp::Node {
public:
  Restamper() : Node("rover_sim_restamper") {
    auto qos = rclcpp::SensorDataQoS();
    const std::string cam = "/camera/camera0";

    pub_infra_ = create_publisher<Image>(cam + "/infra1/image_rect_raw", qos);
    pub_infra_ci_ = create_publisher<CameraInfo>(cam + "/infra1/camera_info", qos);
    pub_depth_ = create_publisher<Image>(cam + "/depth/image_rect_raw", qos);
    pub_depth_ci_ = create_publisher<CameraInfo>(cam + "/depth/camera_info", qos);
    pub_color_ = create_publisher<Image>(cam + "/color/image_raw", qos);
    pub_color_ci_ = create_publisher<CameraInfo>(cam + "/color/camera_info", qos);

    sub_infra_ci_ = create_subscription<CameraInfo>(
        "/rover_sim/infra1/camera_info", qos,
        [this](CameraInfo::SharedPtr m) { infra_ci_ = m; });
    sub_depth_ci_ = create_subscription<CameraInfo>(
        "/rover_sim/depth/camera_info", qos,
        [this](CameraInfo::SharedPtr m) { depth_ci_ = m; });
    sub_color_ci_ = create_subscription<CameraInfo>(
        "/rover_sim/color/camera_info", qos,
        [this](CameraInfo::SharedPtr m) { color_ci_ = m; });

    sub_infra_ = create_subscription<Image>(
        "/rover_sim/infra1/image", qos, [this](Image::SharedPtr m) {
          auto st = now();
          m->header.stamp = st;
          pub_infra_->publish(*m);
          publish_ci(infra_ci_, pub_infra_ci_, st);
        });
    sub_color_ = create_subscription<Image>(
        "/rover_sim/color/image", qos, [this](Image::SharedPtr m) {
          auto st = now();
          m->header.stamp = st;
          pub_color_->publish(*m);
          publish_ci(color_ci_, pub_color_ci_, st);
        });
    sub_depth_ = create_subscription<Image>(
        "/rover_sim/depth/image", qos, [this](Image::SharedPtr m) {
          auto st = now();
          Image out;
          out.header = m->header;
          out.header.stamp = st;
          out.height = m->height;
          out.width = m->width;
          out.encoding = "16UC1";  // millimetres, 0 = invalid (RealSense)
          out.is_bigendian = 0;
          out.step = m->width * 2;
          const size_t n = size_t(m->height) * m->width;
          out.data.resize(n * 2);
          const float* in = reinterpret_cast<const float*>(m->data.data());
          uint16_t* dst = reinterpret_cast<uint16_t*>(out.data.data());
          for (size_t i = 0; i < n; ++i) {
            const float v = in[i];
            dst[i] = (std::isfinite(v) && v > 0.0f)
                         ? uint16_t(std::fmin(v * 1000.0f, 65535.0f))
                         : 0;
          }
          pub_depth_->publish(out);
          publish_ci(depth_ci_, pub_depth_ci_, st);
        });
    RCLCPP_INFO(get_logger(), "restamper up: gz names -> contract names, wall stamps");
  }

private:
  void publish_ci(const CameraInfo::SharedPtr& ci,
                  const rclcpp::Publisher<CameraInfo>::SharedPtr& pub,
                  const rclcpp::Time& st) {
    if (!ci) return;
    CameraInfo out = *ci;
    out.header.stamp = st;
    pub->publish(out);
  }

  rclcpp::Publisher<Image>::SharedPtr pub_infra_, pub_depth_, pub_color_;
  rclcpp::Publisher<CameraInfo>::SharedPtr pub_infra_ci_, pub_depth_ci_, pub_color_ci_;
  rclcpp::Subscription<Image>::SharedPtr sub_infra_, sub_depth_, sub_color_;
  rclcpp::Subscription<CameraInfo>::SharedPtr sub_infra_ci_, sub_depth_ci_, sub_color_ci_;
  CameraInfo::SharedPtr infra_ci_, depth_ci_, color_ci_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Restamper>());
  rclcpp::shutdown();
  return 0;
}
