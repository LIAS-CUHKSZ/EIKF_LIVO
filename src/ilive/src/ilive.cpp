#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <common_lib.h>
#include <kd_tree/ikd_Tree.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <opencv2/core/eigen.hpp>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <FOV_Checker/FOV_Checker.h>

#include "ilive.hpp"

#include "Basis/loam/IMU_Processing.hpp"
#include "Basis/tools/tools_logger.hpp"
#include "Basis/tools/tools_color_printf.hpp"
#include "Basis/tools/tools_eigen.hpp"
#include "Basis/tools/tools_data_io.hpp"
#include "Basis/tools/tools_timer.hpp"
#include "Basis/tools/tools_openCV_3_to_4.hpp"

/**
 * @ref https://blog.csdn.net/weixin_42048023/category_10459061.html
 */ 


Camera_Lidar_queue g_camera_lidar_queue;
MeasureGroup Measures;      // 存放雷达数据和IMU数据的结构体变量
StatesGroup g_lio_state; 

StateParameter g_parames;
std::string data_dump_dir = std::string("/mnt/0B3B134F0B3B134F/color_temp_ilive/");


void mySigintHandler(int sig)
{
  ROS_INFO("Shutting down ROS node.");
  ros::shutdown();
}

int main(int argc, char **argv)
{
    printf_program("ILIVE");
    Common_tools::printf_software_version();
    Eigen::initParallel();
    ros::init(argc, argv, "ILIVE_main");
    ILIVE *ilive = new ILIVE();
    ros::Rate rate(5000);
    bool status = ros::ok();
    // 设置中断信号处理程序
    signal(SIGINT, mySigintHandler);
    ros::spin();
    return 0;
}
