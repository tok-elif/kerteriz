/// \file
/// KITTI kanonik olaylarini ROS konularina yayinlar — YALNIZCA HARICI
/// `robot_localization` TABAN CIZGISI ICIN.
///
/// ======================= KATMAN SINIRI — ONEMLI =============================
///
/// Bu dugum Kerteriz filtresinin PARCASI DEGILDIR. Kerteriz hicbir noktada
/// `robot_localization` cagirmaz, sonucunu okumaz ve ona bagimli degildir;
/// `kerteriz_core` bu paketi tanimaz. Iki surec AYNI VERI SETINI bagimsiz
/// tuketir:
///
///     ayni veri seti -> Kerteriz ESKF          -> yorunge_K
///                    -> robot_localization EKF -> yorunge_RL
///
/// Burasi yalnizca veri seti -> ROS mesaji cevirisidir.
///
/// ============================== ZAMAN =======================================
///
/// Veri seti damgalari 2011 yilindadir; duvar saatiyle yayinlamak taban
/// cizgisinin tum olcumleri "cok eski" diye atmasina yol acardi. Bu yuzden
/// dugum `/clock` yayinlar ve taban cizgisi `use_sim_time` ile kosar. Benzetim
/// zamani veri seti damgalarina gore ilerletilir; mutlak zaman hicbir yerde
/// kayan noktaya cevrilmez, ROS mesaj damgasi saniye+nanosaniye TAMSAYI
/// alanlarindan kurulur.

#include "kerteriz_bringup/kitti_oxts.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <thread>
#include <vector>

namespace {

using kerteriz::TimeNs;
using kerteriz_bringup::DatasetEvent;
using kerteriz_bringup::DatasetEventKind;

builtin_interfaces::msg::Time zaman(TimeNs t) {
  builtin_interfaces::msg::Time m;
  m.sec = static_cast<std::int32_t>(t / 1000000000LL);
  m.nanosec = static_cast<std::uint32_t>(t % 1000000000LL);
  return m;
}

} // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("kitti_baseline_publisher");

  const auto veri_yolu = node->declare_parameter<std::string>("dataset_dir", "");
  const auto adim_ms = node->declare_parameter<int>("clock_step_ms", 10);
  const auto hiz = node->declare_parameter<double>("realtime_factor", 1.0);
  // Taban cizgisine GNSS HIZI verilmez; gerekce yapilandirma dosyasindadir.
  const auto hiz_yayinla = node->declare_parameter<bool>("publish_gnss_velocity", false);

  if (veri_yolu.empty()) {
    RCLCPP_ERROR(node->get_logger(), "dataset_dir parametresi zorunlu");
    return 2;
  }

  kerteriz_bringup::KittiConfig cfg;
  cfg.dataset_dir = veri_yolu;
  std::vector<DatasetEvent> olaylar;
  const auto d = kerteriz_bringup::load_kitti_oxts(cfg, olaylar);
  if (!d.ok) {
    RCLCPP_ERROR(node->get_logger(), "KITTI ayristirma hatasi: %s", d.message.c_str());
    return 1;
  }

  auto saat = node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
  auto imu_yayin = node->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 50);
  auto gnss_yayin = node->create_publisher<nav_msgs::msg::Odometry>("/odometry/gps", 50);

  const TimeNs bas = olaylar.front().stamp_ns;
  const TimeNs son = olaylar.back().stamp_ns;
  const TimeNs adim = static_cast<TimeNs>(adim_ms) * 1000000LL;

  // Taban cizgisi abonelerinin baglanmasi icin kisa bir bekleme.
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));

  std::size_t i = 0;
  int imu_sayisi = 0;
  int gnss_sayisi = 0;
  int atlanan_interpolated = 0;

  for (TimeNs t = bas; t <= son + adim && rclcpp::ok(); t += adim) {
    rosgraph_msgs::msg::Clock c;
    c.clock = zaman(t);
    saat->publish(c);

    while (i < olaylar.size() && olaylar[i].stamp_ns <= t) {
      const DatasetEvent& e = olaylar[i];
      ++i;
      if (e.kind == DatasetEventKind::kImu) {
        sensor_msgs::msg::Imu m;
        m.header.stamp = zaman(e.stamp_ns);
        m.header.frame_id = "base_link";
        m.angular_velocity.x = e.imu.gyro.x();
        m.angular_velocity.y = e.imu.gyro.y();
        m.angular_velocity.z = e.imu.gyro.z();
        m.linear_acceleration.x = e.imu.accel.x();
        m.linear_acceleration.y = e.imu.accel.y();
        m.linear_acceleration.z = e.imu.accel.z();
        // Yonelim SAGLANMAZ: Kerteriz de IMU'dan yonelim almiyor.
        m.orientation_covariance[0] = -1.0;
        for (int k = 0; k < 3; ++k) {
          m.angular_velocity_covariance[k * 4] = 1e-4;
          m.linear_acceleration_covariance[k * 4] = 1e-2;
        }
        imu_yayin->publish(m);
        ++imu_sayisi;
      } else if (e.kind == DatasetEventKind::kGnssPosition) {
        if (e.source_interpolated) {
          ++atlanan_interpolated; // Kerteriz ile AYNI politika
          continue;
        }
        nav_msgs::msg::Odometry m;
        m.header.stamp = zaman(e.stamp_ns);
        m.header.frame_id = "odom";
        m.child_frame_id = "base_link";
        m.pose.pose.position.x = e.position_w.x();
        m.pose.pose.position.y = e.position_w.y();
        m.pose.pose.position.z = e.position_w.z();
        m.pose.pose.orientation.w = 1.0;
        for (int k = 0; k < 3; ++k) {
          for (int j = 0; j < 3; ++j) {
            m.pose.covariance[static_cast<std::size_t>(k * 6 + j)] = e.position_cov_w(k, j);
          }
        }
        gnss_yayin->publish(m);
        ++gnss_sayisi;
      } else if (e.kind == DatasetEventKind::kGnssVelocity && hiz_yayinla) {
        // Varsayilan olarak KAPALIDIR. nav_msgs/Odometry twist'i COCUK
        // CERCEVEDEDIR; bizim GNSS hizimiz ise dunya ENU'sundadir. Cevirmek
        // icin bir yonelim gerekir ve o yonelimi taban cizgisine vermek ona
        // Kerteriz'de olmayan bilgi vermek olurdu. Fark gizlenmiyor,
        // yapilandirmada ve raporda yaziliyor.
        continue;
      }
    }

    if (hiz > 0.0) {
      const auto uyku = std::chrono::nanoseconds(static_cast<long long>(adim / hiz));
      std::this_thread::sleep_for(uyku);
    }
    rclcpp::spin_some(node);
  }

  RCLCPP_INFO(node->get_logger(), "yayin bitti: imu=%d gnss=%d atlanan_interpolated=%d", imu_sayisi,
              gnss_sayisi, atlanan_interpolated);
  // Taban cizgisinin son ciktiyi uretmesi icin kisa bekleme.
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  rclcpp::shutdown();
  return 0;
}
