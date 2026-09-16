/// \file
/// Harici taban cizgisinin ciktisini CSV'ye yazar.
///
/// `robot_localization` sonucunu Kerteriz'e VERMEZ; yalnizca ayni ATE aracina
/// girebilecek bir dosya uretir. Iki surec birbirini gormez.

#include <cstdint>
#include <fstream>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("baseline_recorder");
  const auto yol = node->declare_parameter<std::string>("output_csv", "");
  const auto konu = node->declare_parameter<std::string>("topic", "/odometry/filtered");
  if (yol.empty()) {
    RCLCPP_ERROR(node->get_logger(), "output_csv parametresi zorunlu");
    return 2;
  }

  auto dosya = std::make_shared<std::ofstream>(yol);
  if (!*dosya) {
    RCLCPP_ERROR(node->get_logger(), "cikti acilamadi: %s", yol.c_str());
    return 1;
  }
  *dosya << "timestamp_ns,px,py,pz,vx,vy,vz\n";
  dosya->setf(std::ios::fixed);
  dosya->precision(9);

  auto sayac = std::make_shared<int>(0);
  auto abone = node->create_subscription<nav_msgs::msg::Odometry>(
      konu, 200, [dosya, sayac](const nav_msgs::msg::Odometry::SharedPtr m) {
        const std::int64_t t =
            static_cast<std::int64_t>(m->header.stamp.sec) * 1000000000LL + m->header.stamp.nanosec;
        *dosya << t << ',' << m->pose.pose.position.x << ',' << m->pose.pose.position.y << ','
               << m->pose.pose.position.z << ',' << m->twist.twist.linear.x << ','
               << m->twist.twist.linear.y << ',' << m->twist.twist.linear.z << '\n';
        dosya->flush(); // surec disaridan sonlandirilsa bile son ornek kaybolmasin
        ++(*sayac);
      });

  rclcpp::spin(node);
  dosya->flush();
  RCLCPP_INFO(node->get_logger(), "kaydedilen ornek: %d", *sayac);
  rclcpp::shutdown();
  return 0;
}
