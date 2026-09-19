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

#include "kerteriz_bringup/gnss_sampling.hpp"
#include "kerteriz_bringup/kitti_oxts.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
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
  // Parametre GERIYE DONUK uyumluluk icin duruyor ama YALNIZCA false kabul
  // eder: true verildiginde dugum acik hatayla cikar. Onceden true degeri
  // sessizce yok sayiliyordu ve "hiz yayinlaniyor" izlenimi birakiyordu.
  const auto hiz_yayinla = node->declare_parameter<bool>("publish_gnss_velocity", false);
  // F2.4-C: 0 (veya verilmemis) = LEGACY, seyreltme yok. Pozitif deger
  // Kerteriz kosucusuyla AYNI saf politikayi calistirir.
  const auto stride = node->declare_parameter<int>("gnss_position_stride", 0);

  if (veri_yolu.empty()) {
    RCLCPP_ERROR(node->get_logger(), "dataset_dir parametresi zorunlu");
    return 2;
  }
  if (hiz_yayinla) {
    RCLCPP_ERROR(node->get_logger(),
                 "publish_gnss_velocity=true DESTEKLENMIYOR. nav_msgs/Odometry twist'i "
                 "COCUK CERCEVEDEDIR, GNSS hizimiz ise dunya ENU'sundadir; cevirmek icin "
                 "taban cizgisine Kerteriz'de olmayan bir yonelim vermek gerekirdi "
                 "(config/robot_localization_baseline.yaml). Parametre sessizce yok "
                 "sayilmaz; kaldirin veya false birakin.");
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

  // F2.4-C secim plani. AYNI paylasimli fonksiyon Kerteriz kosucusunda da
  // kullanilir; iki kestirimci boylece BIREBIR ayni olcum damgalarini alir.
  // Bu bir iddia degil: asagida damga sayisi ve ozeti basilarak gosterilir.
  kerteriz_bringup::GnssSamplingPolicy politika;
  politika.enabled = stride > 0;
  politika.stride = politika.enabled ? stride : 1;
  const auto plan = kerteriz_bringup::build_sampling_plan(olaylar, politika);
  if (!plan.status.ok) {
    RCLCPP_ERROR(node->get_logger(), "seyreltme plani kurulamadi: %s", plan.status.message.c_str());
    return 1;
  }
  if (politika.enabled) {
    // Ozet Kerteriz kosucusuyla AYNI paylasimli fonksiyondan gelir
    // (gnss_sampling.hpp). Algoritma burada TEKRARLANMAZ: iki kopya ayrisirsa
    // ozet tam da yakalamasi beklenen ayrismayi gizlerdi. Iki sureci
    // karsilastirmak icin stride + olcum sayisi + ozet uclusune bakilir.
    const std::uint64_t ozet = kerteriz_bringup::measurement_stamp_digest(plan);
    // `stride` `declare_parameter<int>`'ten gelir ama rclcpp tamsayi
    // parametreyi `int64_t` olarak dondurur; `auto` da onu yakalar. Bu yuzden
    // bicim `%ld` ve arguman acikca `long`'a cevrilir — `%d` ile basmak
    // tanimsiz davranisti. Daraltip `%d` birakmak da olurdu ama o, buyuk bir
    // parametre degerini LOG'da sessizce kirpardi; burada deger oldugu gibi
    // yazilir. Plan alanlari `int`'tir, onlarin `%d`'si dogrudur.
    RCLCPP_INFO(node->get_logger(),
                "GNSS konum seyreltmesi acik: stride=%ld aday=%d secili_slot=%d olcum=%d "
                "ara-degerlenmis-atlanan=%d damga_ozeti=%llu",
                static_cast<long>(stride), plan.candidate_count, plan.selected_slot_count,
                plan.selected_usable_count, plan.selected_interpolated_skipped,
                static_cast<unsigned long long>(ozet));
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
  int atlanan_secilmeyen = 0;

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
        // F2.4-C kapisi. Legacy'de `enabled = false` oldugu icin kisa devre
        // ile hic degerlendirilmez ve eski davranis KORUNUR.
        if (politika.enabled &&
            !kerteriz_bringup::contains_stamp(plan.measurement_stamps, e.stamp_ns)) {
          ++atlanan_secilmeyen;
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
      } else if (e.kind == DatasetEventKind::kGnssVelocity) {
        // Bu taban cizgisinde GNSS hizi HICBIR yapilandirmada yayinlanmaz;
        // `publish_gnss_velocity=true` verilirse dugum yukarida acik hatayla
        // cikmistir. Fark gizlenmiyor, yapilandirmada ve raporda yaziliyor.
        continue;
      }
    }

    if (hiz > 0.0) {
      const auto uyku = std::chrono::nanoseconds(static_cast<long long>(adim / hiz));
      std::this_thread::sleep_for(uyku);
    }
    rclcpp::spin_some(node);
  }

  RCLCPP_INFO(node->get_logger(),
              "yayin bitti: imu=%d gnss=%d atlanan_interpolated=%d atlanan_secilmeyen=%d",
              imu_sayisi, gnss_sayisi, atlanan_interpolated, atlanan_secilmeyen);
  // Taban cizgisinin son ciktiyi uretmesi icin kisa bekleme.
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  rclcpp::shutdown();
  return 0;
}
