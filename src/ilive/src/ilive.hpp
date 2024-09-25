#pragma once

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

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "Basis/tools/lib_sophus/so3.hpp"
#include "Basis/tools/lib_sophus/se3.hpp"

#include "Basis/tools/tools_logger.hpp"
#include "Basis/tools/tools_color_printf.hpp"
#include "Basis/tools/tools_eigen.hpp"
#include "Basis/tools/tools_data_io.hpp"
#include "Basis/tools/tools_timer.hpp"
#include "Basis/tools/tools_thread_pool.hpp"
#include "Basis/tools/tools_ros.hpp"

#include "Basis/loam/IMU_Processing.hpp"

#include "Basis/rgb_map/image_frame.hpp"
#include "Basis/rgb_map/pointcloud_rgbd.hpp"
#include "Basis/rgb_map/rgbmap_tracker.hpp"
#include "StateParameter.h"
#include "FeatsDown.h"

#define THREAD_SLEEP_TIM 1

#include "offline_map_recorder.hpp"

#define INIT_TIME (0)
// #define LASER_POINT_COV (0.0015) // Ori
#define NUM_MATCH_POINTS (5)

#define MAXN 360000

const int laserCloudWidth = 48;
const int laserCloudHeight = 48;
const int laserCloudDepth = 48;
const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;
// estimator inputs and output;
extern Camera_Lidar_queue g_camera_lidar_queue;
extern MeasureGroup Measures;
extern StatesGroup g_lio_state;
extern StateParameter g_parames;
 extern double g_lidar_star_tim;
extern double g_vio_frame_cost_time;
extern double g_lio_frame_cost_time;
void dump_lio_state_to_log(FILE *fp);

extern Common_tools::Cost_time_logger g_cost_time_logger;
extern std::shared_ptr<Common_tools::ThreadPool> m_thread_pool_ptr;
class ILIVE
{
public:
    // Design global paras for VIO/LIO

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::mutex m_mutex_lio_process; // lio process mutex
    std::shared_ptr<ImuProcess> m_imu_process;
    std::string root_dir = ROOT_DIR;
    FILE *m_lio_state_fp;
    FILE *m_lio_costtime_fp;

    // paras for lio. We will init these paras in the config file.
    std::vector<double> m_gravity;
    double m_maximum_pt_kdtree_dis = 1.0;
    double m_maximum_res_dis = 1.0;
    double m_planar_check_dis = 0.05;  // point to plane dis
    double m_lidar_imu_time_delay = 0; // lidar imu time delay
    double m_camera_imu_time_delay = 0;

    double m_long_rang_pt_dis = 500.0; // long range point distance
    bool m_if_publish_feature_map = true;
    int NUM_MAX_ITERATIONS = 0;
    int FOV_RANGE = 4; // range of FOV = FOV_RANGE * cube_len. It's used to check the point is in the FOV or not.

    // These paras in which block of LIO
    int laserCloudCenWidth = 24;
    int laserCloudCenHeight = 24;
    int laserCloudCenDepth = 24;

    int laserCloudValidNum = 0;
    int laserCloudSelNum = 0;

    // Is it necessary for LIO/VIO????
    double T1[MAXN], T2[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN];
    int time_log_counter = 0;

    /// IMU relative variables
    std::mutex mtx_buffer;            
    std::condition_variable sig_buffer; 
    bool lidar_pushed = false;          
    bool flg_exit = false;              
    bool flg_reset = false;           
    // Buffers for measurements
    double cube_len = 0.0;
    double lidar_end_time = 0.0;
    double last_timestamp_lidar = -1;
    double last_timestamp_imu = -1;
    double HALF_FOV_COS = 0.0;
    double FOV_DEG = 0.0;
    double res_mean_last = 0.05;
    double total_distance = 0.0;
    Eigen::Vector3d position_last = Zero3d;
    double copy_time, readd_time, fov_check_time, readd_box_time, delete_box_time;
    double kdtree_incremental_time, kdtree_search_time;

    double LASER_POINT_COV = 0.00015;

    std::deque<sensor_msgs::PointCloud2::ConstPtr> lidar_buffer; // lidar帧buffer
    std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer_lio;       // IMU data buffer for LIO
    std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer_vio;       // IMU data buffer for VIO

    // surf feature in map - 地图中的平面特征,因为lidar点的残差构建基于lidar特征点和地图中的平面距离
    PointCloudXYZINormal::Ptr featsFromMap;
    PointCloudXYZINormal::Ptr cube_points_add;
    // all points
    PointCloudXYZINormal::Ptr laserCloudFullRes2;

    Eigen::Vector3f XAxisPoint_body;  //(LIDAR_SP_LEN = 2, 0.0, 0.0);
    Eigen::Vector3f XAxisPoint_world; //(LIDAR_SP_LEN = 2, 0.0, 0.0);



    bool cube_updated[laserCloudNum];
    int laserCloudValidInd[laserCloudNum];
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudFullResColor; //(new pcl::PointCloud<pcl::PointXYZI>());

    KD_TREE ikdtree; // 增量kdtree

    ros::Publisher pubLaserCloudFullRes;
    ros::Publisher pubLaserCloudEffect;
    ros::Publisher pubLaserCloudMap;
    ros::Publisher pubOdomAftMapped; //用来显示当前的姿态(小坐标系)
    ros::Publisher pubPath; // 用来显示整条轨迹


    ros::Publisher pubAllCov;//用来发布协防差


    ros::Subscriber sub_pcl;
    ros::Subscriber sub_imu;
    ros::Subscriber sub_img, sub_img_comp;

    ros::Publisher pub_track_img, pub_raw_img;
    ros::Publisher pub_odom_cam, pub_path_cam;
    bool dense_map_en, flg_EKF_inited = 0, flg_map_initialized = 0, flg_EKF_converged = 0;
    int effect_feat_num = 0, frame_num = 0;
    double filter_size_corner_min, m_voxel_downsample_size_surf, filter_size_map_min, fov_deg, deltaT, deltaR, aver_time_consu = 0, frame_first_pt_time = 0;
    double m_voxel_downsample_size_axis_z;
    geometry_msgs::PoseStamped msg_body_pose;
//-//
    geometry_msgs::PoseWithCovariance msg_pose_cov;
//-//

    nav_msgs::Odometry odomAftMapped;
    PointType pointOri, pointSel, coeff;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterMap;
    // std::vector<double> m_initial_pose;

    Offline_map_recorder m_mvs_recorder;

    /*** debug record ***/
    // ILIVE() = delete;
    ros::NodeHandle m_ros_node_handle;

    // ANCHOR - camera measurement related.
    ros::Publisher m_pub_visual_tracked_3d_pts;
    ros::Publisher m_pub_render_rgb_pts;
    std::vector<std::shared_ptr<ros::Publisher>> m_pub_rgb_render_pointcloud_ptr_vec;
    std::mutex m_camera_data_mutex;
    double m_camera_start_ros_tim = -3e8;
    std::deque<sensor_msgs::ImageConstPtr> m_queue_image_msg;

    std::deque<std::shared_ptr<Image_frame>> m_queue_image_with_pose;
    std::list<std::shared_ptr<Image_frame>> g_image_vec;
    Eigen::Matrix3d g_cam_K;
    Eigen::Matrix<double, 5, 1> g_cam_dist;
    double m_vio_scale_factor = 1.0;
    cv::Mat m_ud_map1, m_ud_map2;

    int g_camera_frame_idx = 0;
    int g_LiDAR_frame_index = 0;

    int valid_cam_points = 0;
    int valid_lid_points = 0;
    std::mutex g_mutex_render;
    std::shared_ptr<Image_frame> g_last_image_pose_for_render = nullptr;
    std::list<double> frame_cost_time_vec;
    Rgbmap_tracker op_track;
    Global_map m_map_rgb_pts;
    int m_maximum_image_buffer = 2;
    int m_track_windows_size = 50;
    double m_minumum_rgb_pts_size = 0.05;
    double m_vio_image_width = 0;
    double m_vio_image_heigh = 0;
    int m_if_estimate_i2c_extrinsic = 0;
    int m_if_estimate_intrinsic = 0;
    double m_control_image_freq = 100;
    int m_maximum_vio_tracked_pts = 300;
    int m_lio_update_point_step = 1;
    int m_append_global_map_point_step = 1;
    int m_pub_pt_minimum_views = 5;
    double m_recent_visited_voxel_activated_time = 0.0;
    int m_number_of_new_visited_voxel = 0;
    double m_tracker_minimum_depth = 3;
    double m_tracker_maximum_depth = 200;
    int m_if_record_mvs = 0;
    cv::Mat intrinsic, dist_coeffs;
    bool if_save = false;
    std::string m_method = "EKF";

    int m_lio_init = 1;
    int m_vio_f2map_init = 1;
    int m_vio_f2f_init = 1;

    // extinct parameters
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> ext_R_lid_in_imu;
    Eigen::Matrix<double, 3, 1> ext_t_lid_in_imu;
    void load_lio_parameters(StateParameter &_paras);

    mat_3_3 m_inital_rot_ext_i2c;
    vec_3 m_inital_pos_ext_i2c;
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> m_camera_intrinsic;
    Eigen::Matrix<double, 5, 1> m_camera_dist_coeffs;
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> m_camera_ext_R;
    Eigen::Matrix<double, 3, 1> m_camera_ext_t;

    double m_image_downsample_ratio = 1.0;
    nav_msgs::Path camera_path;
    double m_cam_measurement_weight = 1e-3;
    double m_cov_scale = 1;
    // double m_lid_measurement_weight=0.001;

    double m_imu_cov=1;

    int m_if_pub_raw_img = 1;
    int vio_proj_iter_times = 1;
    int vio_f2map_iter_times = 2;
    std::vector<std::shared_ptr<RGB_pts>> m_last_added_rgb_pts_vec;
    std::string m_map_output_dir;
    std::shared_ptr<std::shared_future<void>> m_render_thread = nullptr;

    // VIO subsystem related
    void load_parameters(StateParameter &_paras);
    void set_initial_camera_parameter(StatesGroup &state,
                                      double *camera_intrinsic_data,
                                      double *camera_dist_data,
                                      double *imu_camera_ext_R,
                                      double *imu_camera_ext_t,
                                      double cam_k_scale);
    void process_image(cv::Mat &image, double msg_time);
    void image_callback(const sensor_msgs::ImageConstPtr &msg);
    void image_comp_callback(const sensor_msgs::CompressedImageConstPtr &msg);
    void set_image_pose(std::shared_ptr<Image_frame> &image_pose, const StatesGroup &state);
    void publish_camera_odom(std::shared_ptr<Image_frame> &image, double msg_time);
    void publish_track_img(cv::Mat &img, double frame_cost_time);
    void publish_raw_img(cv::Mat &img);
    void publish_track_pts(Rgbmap_tracker &tracker);
    bool vio_preintegration(StatesGroup &state_in, StatesGroup &state_out, double current_frame_time);
    bool vio_projection(StatesGroup &state_in, Rgbmap_tracker &op_track);
    bool vio_photometric(StatesGroup &state_in, Rgbmap_tracker &op_track, std::shared_ptr<Image_frame> &image);
    void service_VIO_update();
    void service_process_img_buffer();
    void service_pub_rgb_maps();
    char cv_keyboard_callback();
    void set_initial_state_cov(StatesGroup &stat);
    cv::Mat generate_control_panel_img();
    void lio_update(Common_tools::Timer tim, PointCloudXYZINormal::Ptr &laserCloudOri, FeatsDown &feats_down, Eigen::Vector3d &euler_cur, StatesGroup &state_in);
    // ANCHOR -  service_pub_rgb_maps

    void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);

    bool sync_packages(MeasureGroup &meas);

    ILIVE()
    { //(1)初始化涉及到的发布消息格式
        pubLaserCloudFullRes = m_ros_node_handle.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100);
        pubLaserCloudEffect = m_ros_node_handle.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100);
        pubLaserCloudMap = m_ros_node_handle.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100);
        pubOdomAftMapped = m_ros_node_handle.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 100); 
//-//
        pubAllCov = m_ros_node_handle.advertise<std_msgs::Float64MultiArray>("/covariance16", 10); 
//-//

        pub_track_img = m_ros_node_handle.advertise<sensor_msgs::Image>("/track_img", 1000);

    
        pub_raw_img = m_ros_node_handle.advertise<sensor_msgs::Image>("/raw_in_img", 1000);
        m_pub_visual_tracked_3d_pts = m_ros_node_handle.advertise<sensor_msgs::PointCloud2>("/track_pts", 10);
        m_pub_render_rgb_pts = m_ros_node_handle.advertise<sensor_msgs::PointCloud2>("/render_pts", 10);
        pubPath = m_ros_node_handle.advertise<nav_msgs::Path>("/path", 10);

        pub_odom_cam = m_ros_node_handle.advertise<nav_msgs::Odometry>("/camera_odom", 10);
        pub_path_cam = m_ros_node_handle.advertise<nav_msgs::Path>("/camera_path", 10);
        std::string LiDAR_pointcloud_topic, IMU_topic, IMAGE_topic, IMAGE_topic_compressed;

        //(2)从ros参数服务器上获取参数
        get_ros_parameter<std::string>(m_ros_node_handle, "/LiDAR_pointcloud_topic", LiDAR_pointcloud_topic, std::string("/laser_cloud_flat"));
        get_ros_parameter<std::string>(m_ros_node_handle, "/IMU_topic", IMU_topic, std::string("/livox/imu"));
        get_ros_parameter<std::string>(m_ros_node_handle, "/Image_topic", IMAGE_topic, std::string("/camera/image_color"));
        IMAGE_topic_compressed = std::string(IMAGE_topic).append("/compressed");

        //(3)打印基本信息
        if (1)
        {
            scope_color(ANSI_COLOR_BLUE_BOLD);
            cout << "======= Summary of subscribed topics =======" << endl;
            cout << "LiDAR pointcloud topic: " << LiDAR_pointcloud_topic << endl;
            cout << "IMU topic: " << IMU_topic << endl;
            cout << "Image topic: " << IMAGE_topic << endl;
            cout << "Image compressed topic: " << IMAGE_topic << endl;
            cout << "=======        -End-                =======" << endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }


        sub_imu = m_ros_node_handle.subscribe(IMU_topic.c_str(), 2000000, &ILIVE::imu_cbk, this, ros::TransportHints().tcpNoDelay());
        sub_pcl = m_ros_node_handle.subscribe(LiDAR_pointcloud_topic.c_str(), 2000000, &ILIVE::feat_points_cbk, this, ros::TransportHints().tcpNoDelay());
        sub_img = m_ros_node_handle.subscribe(IMAGE_topic.c_str(), 1000000, &ILIVE::image_callback, this, ros::TransportHints().tcpNoDelay());
        sub_img_comp = m_ros_node_handle.subscribe(IMAGE_topic_compressed.c_str(), 1000000, &ILIVE::image_comp_callback, this, ros::TransportHints().tcpNoDelay());
        m_pub_rgb_render_pointcloud_ptr_vec.resize(1e3);

        // (5) 从ROS参数服务器上获取参数信息
        // ANCHOR - ROS parameters
        if (1)
        {
            scope_color(ANSI_COLOR_RED);
            get_ros_parameter(m_ros_node_handle, "ilive_common/map_output_dir", m_map_output_dir,
                              Common_tools::get_home_folder().append("/ilive_output"));
            get_ros_parameter(m_ros_node_handle, "ilive_common/append_global_map_point_step", m_append_global_map_point_step, 4);
            get_ros_parameter(m_ros_node_handle, "ilive_common/recent_visited_voxel_activated_time", m_recent_visited_voxel_activated_time, 0.0);
            get_ros_parameter(m_ros_node_handle, "ilive_common/maximum_image_buffer", m_maximum_image_buffer, 20000);
            get_ros_parameter(m_ros_node_handle, "ilive_common/tracker_minimum_depth", m_tracker_minimum_depth, 0.1);
            get_ros_parameter(m_ros_node_handle, "ilive_common/tracker_maximum_depth", m_tracker_maximum_depth, 200.0);
            get_ros_parameter(m_ros_node_handle, "ilive_common/track_windows_size", m_track_windows_size, 40);
            get_ros_parameter(m_ros_node_handle, "ilive_common/minimum_pts_size", m_minumum_rgb_pts_size, 0.05);
            get_ros_parameter(m_ros_node_handle, "ilive_common/record_offline_map", m_if_record_mvs, 0);
            get_ros_parameter(m_ros_node_handle, "ilive_common/pub_pt_minimum_views", m_pub_pt_minimum_views, 5);

            get_ros_parameter(m_ros_node_handle, "ilive_common/image_downsample_ratio", m_image_downsample_ratio, 1.0);
            get_ros_parameter(m_ros_node_handle, "ilive_common/esikf_iter_f2f_times", vio_proj_iter_times, 1);
            get_ros_parameter(m_ros_node_handle, "ilive_common/vio_f2map_iter_times", vio_f2map_iter_times, 1);

            get_ros_parameter(m_ros_node_handle, "ilive_common/estimate_i2c_extrinsic", m_if_estimate_i2c_extrinsic, 0);
            get_ros_parameter(m_ros_node_handle, "ilive_common/estimate_intrinsic", m_if_estimate_intrinsic, 0);
            get_ros_parameter(m_ros_node_handle, "ilive_common/maximum_vio_tracked_pts", m_maximum_vio_tracked_pts, 600);

            get_ros_parameter(m_ros_node_handle, "ilive_common/use_lio", m_lio_init, 1);
            get_ros_parameter(m_ros_node_handle, "ilive_common/use_vio_f2f", m_vio_f2f_init, 0);
            get_ros_parameter(m_ros_node_handle, "ilive_common/use_vio_f2map", m_vio_f2map_init, 0);
            get_ros_parameter(m_ros_node_handle, "ilive_common/if_save_mapping", if_save, false);



        }
        if (1)
        {
            scope_color(ANSI_COLOR_GREEN);
            get_ros_parameter(m_ros_node_handle, "ilive_lio/dense_map_enable", dense_map_en, true);
            get_ros_parameter(m_ros_node_handle, "ilive_lio/max_iteration", NUM_MAX_ITERATIONS, 1);
            get_ros_parameter(m_ros_node_handle, "ilive_lio/fov_degree", fov_deg, 360.00);
            get_ros_parameter(m_ros_node_handle, "ilive_lio/voxel_downsample_size_surf", m_voxel_downsample_size_surf, 0.3);
            get_ros_parameter(m_ros_node_handle, "ilive_lio/voxel_downsample_size_axis_z", m_voxel_downsample_size_axis_z,
                              m_voxel_downsample_size_surf);
            get_ros_parameter(m_ros_node_handle, "ilive_lio/filter_size_map", filter_size_map_min, 0.4);
            get_ros_parameter(m_ros_node_handle, "ilive_lio/cube_side_length", cube_len, 10000000.0);
            get_ros_parameter(m_ros_node_handle, "ilive_lio/maximum_pt_kdtree_dis", m_maximum_pt_kdtree_dis, 0.5);
            get_ros_parameter(m_ros_node_handle, "ilive_lio/maximum_res_dis", m_maximum_res_dis, 0.3);
            get_ros_parameter(m_ros_node_handle, "ilive_lio/planar_check_dis", m_planar_check_dis, 0.10);
            get_ros_parameter(m_ros_node_handle, "ilive_lio/long_rang_pt_dis", m_long_rang_pt_dis, 500.0);
            get_ros_parameter(m_ros_node_handle, "ilive_lio/publish_feature_map", m_if_publish_feature_map, true);
            get_ros_parameter(m_ros_node_handle, "ilive_lio/lio_update_point_step", m_lio_update_point_step, 1);


        }
        if(1)
        {
            scope_color(ANSI_COLOR_YELLOW_BOLD);
 
        }

        if (1)
        {
            scope_color(ANSI_COLOR_BLUE);
            m_ros_node_handle.getParam("ilive_common/gravity", m_gravity);
            std::string default_method = "EKF";
            get_ros_parameter(m_ros_node_handle, "ilive_common/method", m_method, default_method);
            get_ros_parameter(m_ros_node_handle, "ilive_IMU/imu_cov_scale", m_imu_cov,1.0);
            get_ros_parameter(m_ros_node_handle, "ilive_vio/cam_cov_scale", m_cov_scale, 0.01);
            get_ros_parameter(m_ros_node_handle, "ilive_lio/lid_cov_scale", LASER_POINT_COV, 0.00015);
           get_ros_parameter(m_ros_node_handle, "ilive_vio/camera_time_delay", m_camera_imu_time_delay, 0.0);
            get_ros_parameter(m_ros_node_handle, "ilive_lio/lidar_time_delay", m_lidar_imu_time_delay, 0.0);
 
            g_parames.Gravity = Eigen::Map<Eigen::Vector3d, Eigen::Unaligned>(m_gravity.data(), m_gravity.size());
            g_lio_state.gravity = g_parames.Gravity;
            load_parameters(g_parames);
            g_lio_state.method = m_method;
        }

        // (6)创建地图输出路径
        if (!Common_tools::if_file_exist(m_map_output_dir))
        {
            cout << ANSI_COLOR_BLUE_BOLD << "Create ilive output dir: " << m_map_output_dir << ANSI_COLOR_RESET << endl;
            Common_tools::create_dir(m_map_output_dir);
        }
        // (7)初始化用到的变量
        m_thread_pool_ptr = std::make_shared<Common_tools::ThreadPool>(6, true, false); // 设置线程池 At least 5 threads are needs, here we allocate 6 threads.
        g_cost_time_logger.init_log(std::string(m_map_output_dir).append("/cost_time_logger.log"));
        m_map_rgb_pts.set_minmum_dis(m_minumum_rgb_pts_size);
        m_map_rgb_pts.m_recent_visited_voxel_activated_time = m_recent_visited_voxel_activated_time;
        featsFromMap = boost::make_shared<PointCloudXYZINormal>();
        cube_points_add = boost::make_shared<PointCloudXYZINormal>(); ////////////NOTES!!!!!!!!!!
        laserCloudFullRes2 = boost::make_shared<PointCloudXYZINormal>();
        laserCloudFullResColor = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

        XAxisPoint_body = Eigen::Vector3f(LIDAR_SP_LEN, 0.0, 0.0);  // (2, 0, 0)
        XAxisPoint_world = Eigen::Vector3f(LIDAR_SP_LEN, 0.0, 0.0); // (2, 0, 0)

        downSizeFilterSurf.setLeafSize(m_voxel_downsample_size_surf, m_voxel_downsample_size_surf, m_voxel_downsample_size_axis_z);
        downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);

        // (8)设置LIO的log文件, 创建LIO子系统线程
        m_lio_state_fp = fopen(std::string(m_map_output_dir).append("/lic_lio.log").c_str(), "w+");
        m_lio_costtime_fp = fopen(std::string(m_map_output_dir).append("/lic_lio_costtime.log").c_str(), "w+");
        m_thread_pool_ptr->commit_task(&ILIVE::service_LIO_update, this); // 创建LIO线程
    }
    ~ILIVE(){};

    // project lidar frame to world
    template <typename T>
    void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po)
    {
        Eigen::Vector3d p_body(pi[0], pi[1], pi[2]);
        Eigen::Vector3d p_global(g_lio_state.rot_end * (ext_R_lid_in_imu * p_body + ext_t_lid_in_imu) + g_lio_state.pos_end);

        // Eigen::Vector3d p_global(g_lio_state.rot_end * ext_R_lid_in_imu * p_body + g_lio_state.rot_end * ext_t_lid_in_imu + g_lio_state.pos_end);

        po[0] = p_global(0);
        po[1] = p_global(1);
        po[2] = p_global(2);
    }
    void pointBodyToWorld(PointType const *const pi, PointType *const po);
    void RGBpointBodyToWorld(PointType const *const pi, pcl::PointXYZI *const po);
    void feat_points_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg_in);
    void wait_render_thread_finish();
    bool get_pointcloud_data_from_ros_message(sensor_msgs::PointCloud2::ConstPtr &msg, pcl::PointCloud<pcl::PointXYZINormal> &pcl_pc);
    int service_LIO_update();
    void publish_render_pts(ros::Publisher &pts_pub, Global_map &m_map_rgb_pts);
    void print_dash_board();
    Pose consistent_solution_to_LIDAR(PointCloudXYZINormal::Ptr laserCloudOri, PointCloudXYZINormal::Ptr coeffSel, int N, const std::vector<double> &uTq, double &Cov_estimate);
};
