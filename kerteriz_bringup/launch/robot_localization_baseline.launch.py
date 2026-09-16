"""HARICI taban cizgisi calistirmasi — robot_localization.

Kerteriz filtresi BU LAUNCH'IN PARCASI DEGILDIR. Burada yalnizca veri seti
ROS konularina yayinlanir, robot_localization bagimsiz olarak kosar ve ciktisi
CSV'ye kaydedilir. Kerteriz kendi kosucusuyla AYRI calistirilir.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pay = get_package_share_directory("kerteriz_bringup")
    varsayilan_yapilandirma = os.path.join(pay, "config", "robot_localization_baseline.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("dataset_dir", description="KITTI raw _sync dizini"),
            DeclareLaunchArgument("output_csv", description="taban cizgisi yorunge CSV'si"),
            DeclareLaunchArgument("config", default_value=varsayilan_yapilandirma),
            # Kerteriz kosucusunun urettigi baslangic durumu. Taban cizgisi ile
            # Kerteriz AYNI baslangic bilgisini tek kod yolundan alir.
            DeclareLaunchArgument("initial_state_params"),
            DeclareLaunchArgument("realtime_factor", default_value="4.0"),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[
                    LaunchConfiguration("config"),
                    LaunchConfiguration("initial_state_params"),
                ],
            ),
            Node(
                package="kerteriz_bringup",
                executable="kerteriz_baseline_recorder",
                name="baseline_recorder",
                output="screen",
                parameters=[
                    {
                        "output_csv": LaunchConfiguration("output_csv"),
                        "use_sim_time": True,
                    }
                ],
            ),
            Node(
                package="kerteriz_bringup",
                executable="kerteriz_kitti_baseline_publisher",
                name="kitti_baseline_publisher",
                output="screen",
                parameters=[
                    {
                        "dataset_dir": LaunchConfiguration("dataset_dir"),
                        "realtime_factor": LaunchConfiguration("realtime_factor"),
                        "use_sim_time": False,
                    }
                ],
            ),
        ]
    )
