#ifndef COMMON_LIB_H
#define COMMON_LIB_H
#define G_m_s2 (9.81)  
#include <so3_math.h>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <rosbag/bag.h>

#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include "tools_color_printf.hpp"
#include "tools_eigen.hpp"
#include "tools_ros.hpp"
#include <queue>
#include <deque>
#include "lib_sophus/se3.hpp"
#include "lib_sophus/so3.hpp"
#include "StateParameter.h"

// #define DEBUG_PRINT
#define USE_ikdtree
#define printf_line std::cout << __FILE__ << " " << __LINE__ << std::endl;

#define PI_M (3.14159265358)

#define USE_dt  1

#if USE_dt
#define DIM_OF_STATES (16)
#else
#define DIM_OF_STATES (15)
#endif

#define DIM_OF_PROC_N (12)
#define CUBE_LEN (6.0)
#define LIDAR_SP_LEN (2)
#define INIT_COV (0.01)

#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]
#define CONSTRAIN(v, min, max) ((v > min) ? ((v < max) ? v : max) : min)
#define ARRAY_FROM_EIGEN(mat) mat.data(), mat.data() + mat.rows() * mat.cols()
#define STD_VEC_FROM_EIGEN(mat) std::vector<decltype(mat)::Scalar>(mat.data(), mat.data() + mat.rows() * mat.cols())

#define DEBUG_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "Log/" + name))

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZINormal;

static const Eigen::Matrix3d Eye3d(Eigen::Matrix3d::Identity());
static const Eigen::Vector3d Zero3d(0, 0, 0);


struct Pose6D
{
    typedef double data_type;
    data_type offset_time;
    data_type rot[9];
    data_type acc[3];
    data_type vel[3];
    data_type pos[3];
    data_type gyr[3];
};

struct Pose{
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
};

struct Camera_Lidar_queue
{
    double m_first_imu_time = -3e88;
    double m_sliding_window_tim = 10000;
    double m_last_imu_time = -3e88;
    double m_last_visual_time = -3e88;
    double m_last_lidar_time = -3e88;
    double m_visual_init_time = 3e88;
    double m_lidar_drag_cam_tim = 5.0;
    double m_if_lidar_start_first = 1;
    double m_camera_imu_td = 0;

    int m_if_acc_mul_G = 1;
    

    int m_if_have_lidar_data = 0;
    int m_if_have_camera_data = 0;
    int m_if_lidar_can_start = 1;
    Eigen::Vector3d g_noise_cov_acc;
    Eigen::Vector3d g_noise_cov_gyro;

    std::string m_bag_file_name;
    int m_if_dump_log = 1;

    std::deque<sensor_msgs::PointCloud2::ConstPtr> *m_liar_frame_buf = nullptr;

    double time_wrt_first_imu_time(double & time)
    {
        return time - m_first_imu_time;
    }
    
    Camera_Lidar_queue()
    {
        m_if_have_lidar_data = 0;
        m_if_have_camera_data = 0;
    };
    ~Camera_Lidar_queue(){};

    // 设置IMU的当前时刻时间
    double imu_in(const double in_time)
    {
        if (m_first_imu_time < 0)
        {
            m_first_imu_time = in_time;
        }
        m_last_imu_time = std::max(in_time, m_last_imu_time);
        return m_last_imu_time;
    }

    int lidar_in(const double &in_time)
    {
        if (m_if_have_lidar_data == 0)  // 只进来一次,用于判断是否有雷达数据输入
        {                               // 以后就可以用m_if_have_lidar_data判断是否有lidar数据
            m_if_have_lidar_data = 1;
        }
        if (in_time < m_last_imu_time - m_sliding_window_tim)
        {
            std::cout << ANSI_COLOR_RED_BOLD << "LiDAR incoming frame too old, need to be drop!!!" << ANSI_COLOR_RESET << std::endl;
            // TODO: Drop LiDAR frame
        }
        return 1;
    }

    int camera_in(const double &in_time)
    {
        if (in_time < m_last_imu_time - m_sliding_window_tim)
        {
            std::cout << ANSI_COLOR_RED_BOLD << "Camera incoming frame too old, need to be drop!!!" << ANSI_COLOR_RESET << std::endl;
        }
        return 1;
    }

    double get_lidar_front_time()
    {
        if (m_liar_frame_buf != nullptr && m_liar_frame_buf->size())
        {
            m_last_lidar_time = m_liar_frame_buf->front()->header.stamp.toSec()+0.1 ;
            return m_last_lidar_time;
        }
        else
        {
            return -3e88;
        }
    }

    double get_camera_front_time()
    {
        return m_last_visual_time;
    }

    bool if_camera_can_process()
    {
        m_if_have_camera_data = 1;
        double cam_last_time = get_camera_front_time();
        double lidar_last_time = get_lidar_front_time();
        
        if (m_if_have_lidar_data != 1)
        {
            return true;
        }

        if (cam_last_time < 0 || lidar_last_time < 0)
        {
//            cout<< "LIDAR-SIZE:"<<m_liar_frame_buf->size()<<endl;
 //           cout << "3.differ_Cam_tim = " << cam_last_time-lidar_last_time << endl; 
            return false;
        }

        if (lidar_last_time <= cam_last_time)
        {
            return false;
        }
        else
        {
            return true;
        }
        return false;
    }

    void display_last_cam_LiDAR_time()
    {
        double cam_last_time = get_camera_front_time();
        double lidar_last_time = get_lidar_front_time();
        scope_color(ANSI_COLOR_GREEN_BOLD);
        cout<< std::setprecision(15) <<  "Camera time = " << cam_last_time << ", LiDAR last time =  "<< lidar_last_time << endl;        
    }

    bool if_lidar_can_process()
    {
        // m_if_have_lidar_data = 1;
        double cam_last_time = get_camera_front_time(); //获取camera队列头部的cam相机数据(最旧数据)时间
        double lidar_last_time = get_lidar_front_time();//获取lidar队列头部的lidar数据(最旧数据)时间
        if (m_if_have_camera_data == 0)
        {   // 没有相机数据时,不用考虑相机和Lidar的频率对齐关系(参考论文R2LIVE-fig3)
            // 此时直接处理Lidar数据即可
            return true;
        }

        if (cam_last_time < 0 || lidar_last_time < 0)   // 小于0说明队列中还没有数据(默认值为负)
        {
 //           cout << "1. differ_lidar_tim = " << lidar_last_time-cam_last_time << endl; 

            // cout << "Cam_tim = " << cam_last_time << ", lidar_last_time = " << lidar_last_time << endl; 
            return false;
        }

        /**
         * @note 雷达最旧数据必须落后于相机最旧数据,才能处理雷达数据(lidar-10hz, camera-20hz)
         *      cam   :  _ _|_ _|_ _|_ _|_ _
         *      lidar :  _ _ _ _|_ _ _ _|_ _ _ _
         *                  a   b   c   d
         *      从上图可知,假设处理b-d之间的lidar数据,则b时刻lidar数据是当前lidar_buffer中的最旧数据
         *      而该时刻的lidar数据时间必须小于最新处理camera c-d数据中的c时刻相机数据时间.
         */ 

        if (lidar_last_time > cam_last_time) // 雷达上次处理的时间最新,不满足条件
        {
           //cout << "1. differ_lidar_tim = " << lidar_last_time-cam_last_time << endl; 
           return false;
        }
        else{
            return true;
        }
        return false;
    }
};

/**
 * @note 存放雷达数据和IMU数据的结构体
 */ 
struct MeasureGroup // Lidar data and imu dates for the curent process
{
    MeasureGroup()
    {
        this->lidar.reset(new PointCloudXYZINormal());
    };
    double lidar_beg_time;
    double lidar_end_time;
    PointCloudXYZINormal::Ptr lidar;    // 雷达点云数据
    std::deque<sensor_msgs::Imu::ConstPtr> imu; // imu数据
};

struct StatesGroup
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix3d rot_end;                                 // [0-2] the estimated attitude (rotation matrix) at the end lidar point
    Eigen::Vector3d pos_end;                                 // [3-5] the estimated position at the end lidar point (world frame)
    Eigen::Vector3d vel_end;                                 // [6-8] the estimated velocity at the end lidar point (world frame)
    Eigen::Vector3d bias_g;                                  // [9-11] gyroscope bias
    Eigen::Vector3d bias_a;                                  // [12-14] accelerator bias
    Eigen::Vector3d gravity;                                 // [15-17] the estimated gravity acceleration

    Eigen::Matrix3d rot_ext_i2c;                             // [18-20] Extrinsic between IMU frame to Camera frame on rotation.
    Eigen::Vector3d pos_ext_i2c;                             // [21-23] Extrinsic between IMU frame to Camera frame on position.
    double          td_ext_i2c_delta;                        // [24]    Extrinsic between IMU frame to Camera frame on position.
    vec_4           cam_intrinsic;                           // [25-28] Intrinsice of camera [fx, fy, cx, cy]
    Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> cov; // states covariance
    double last_update_time = 0;
    double          td_ext_i2c;
    std::string method;
    bool if_state_inited; 



    StatesGroup()
    {
        rot_end = Eigen::Matrix3d::Identity();
        pos_end = vec_3::Zero();
        vel_end = vec_3::Zero();
        bias_g = vec_3::Zero();
        bias_a = vec_3::Zero();
        gravity = Eigen::Vector3d(0.0,0.0,9.805);

        //Ext camera w.r.t. IMU
        rot_ext_i2c = Eigen::Matrix3d::Identity();
        pos_ext_i2c = vec_3::Zero();

        cov = Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES>::Identity() * INIT_COV;
        // cov.block(18, 18, 6,6) *= 0.1;
        last_update_time = 0;
        td_ext_i2c_delta = 0;
        td_ext_i2c = -0.0;
        method = "EKF";
        if_state_inited = false; ///no initialization
    }

    ~StatesGroup(){}

    StatesGroup operator+(const Eigen::Matrix<double, DIM_OF_STATES, 1> &state_add)
    {
        StatesGroup a = *this;
        Eigen::Matrix<double,3,3> jacobians_left = left_jacobian_of_rotation_matrix(state_add(0), state_add(1), state_add(2));
        Eigen::Matrix<double,3,3> rotation_increment = Exp(state_add(0), state_add(1), state_add(2));
        //a.rot_end = this->rot_end * Sophus::SO3d::exp(vec_3(state_add(0, 0), state_add(1, 0), state_add(2, 0) ) );
        if(this->method=="InEKF"||this->method=="EIKF"){        
        a.rot_end = rotation_increment*this->rot_end;
        a.pos_end = rotation_increment*this->pos_end + jacobians_left*state_add.block<3, 1>(3, 0);//position update +left Jacobians
        a.vel_end = rotation_increment*this->vel_end + jacobians_left*state_add.block<3, 1>(6, 0);// velocity update + left Jacobians
        }
        else{
        a.rot_end = this->rot_end * rotation_increment;
        a.pos_end = this->pos_end + state_add.block<3, 1>(3, 0);
        a.vel_end = this->vel_end + state_add.block<3, 1>(6, 0);
        }
        a.bias_g = this->bias_g + state_add.block<3, 1>(9, 0);
        a.bias_a = this->bias_a + state_add.block<3, 1>(12, 0);
#if USE_dt
        a.td_ext_i2c_delta=this->td_ext_i2c_delta+state_add(15);

#endif 
        a.cov = this->cov;
        a.last_update_time = this->last_update_time;

        return a;
    }

    StatesGroup &operator+=(const Eigen::Matrix<double, DIM_OF_STATES, 1> &state_add)
    {
        Eigen::Matrix<double,3,3> jacobians_left = left_jacobian_of_rotation_matrix(state_add(0), state_add(1), state_add(2));
        Eigen::Matrix<double,3,3> rotation_increment = Exp(state_add(0), state_add(1), state_add(2));
        //a.rot_end = this->rot_end * Sophus::SO3d::exp(vec_3(state_add(0, 0), state_add(1, 0), state_add(2, 0) ) );
        if(this->method=="InEKF"||this->method=="EIKF"){        
        this->rot_end= rotation_increment*this->rot_end;
        this->pos_end = rotation_increment*this->pos_end + jacobians_left*state_add.block<3, 1>(3, 0);//position update +left Jacobians
        this->vel_end = rotation_increment*this->vel_end + jacobians_left*state_add.block<3, 1>(6, 0);// velocity update + left Jacobians
        }
        else{
        this->rot_end = this->rot_end * rotation_increment;
        this->pos_end  = this->pos_end + state_add.block<3, 1>(3, 0);
        this->vel_end = this->vel_end + state_add.block<3, 1>(6, 0);
        }

    #if USE_dt
        this->td_ext_i2c_delta=this->td_ext_i2c_delta+state_add(15);
#endif
        this->bias_g += state_add.block<3, 1>(9, 0);
        this->bias_a += state_add.block<3, 1>(12, 0);
        return *this;
    }

    Eigen::Matrix<double, DIM_OF_STATES, 1> operator-(const StatesGroup &b)
    {
        Eigen::Matrix<double, DIM_OF_STATES, 1> a;
        if(this->method=="InEKF"||this->method=="EIKF"){        
        Eigen::Matrix3d rotd(this->rot_end*b.rot_end.transpose());
        a.block<3, 1>(0, 0) = SO3_LOG(rotd);
        Eigen::Matrix<double,3,3> jacobians_left_inv = left_jacobian_of_rotation_matrix( a(0), a(1), a(2) ).inverse();

        a.block<3, 1>(3, 0) = jacobians_left_inv*(this->pos_end - rotd*b.pos_end);
        a.block<3, 1>(6, 0) = jacobians_left_inv*(this->vel_end - rotd*b.vel_end);
        a.block<3, 1>(9, 0) = this->bias_g - b.bias_g;
        a.block<3, 1>(12, 0) = this->bias_a - b.bias_a;
        }
        else{
        Eigen::Matrix3d rotd(b.rot_end.transpose() * this->rot_end);
        a.block<3, 1>(0, 0) = SO3_LOG(rotd);
        a.block<3, 1>(3, 0) = this->pos_end - b.pos_end;
        a.block<3, 1>(6, 0) = this->vel_end - b.vel_end;
        a.block<3, 1>(9, 0) = this->bias_g - b.bias_g;
        a.block<3, 1>(12, 0) = this->bias_a - b.bias_a;
}



#if USE_dt
        a(15)=this->td_ext_i2c_delta-b.td_ext_i2c_delta;

#endif 
        return a;
    }

    static void display(const StatesGroup &state, std::string str = std::string("State: "))
    {
        vec_3 angle_axis = SO3_LOG(state.rot_end) * 57.3;
        printf("%s |", str.c_str());
        printf("[%.5f] | ", state.last_update_time);
        printf("(%.3f, %.3f, %.3f) | ", angle_axis(0), angle_axis(1), angle_axis(2));
        printf("(%.3f, %.3f, %.3f) | ", state.pos_end(0), state.pos_end(1), state.pos_end(2));
        printf("(%.3f, %.3f, %.3f) | ", state.vel_end(0), state.vel_end(1), state.vel_end(2));
        printf("(%.3f, %.3f, %.3f) | ", state.bias_g(0), state.bias_g(1), state.bias_g(2));
        printf("(%.3f, %.3f, %.3f) \r\n", state.bias_a(0), state.bias_a(1), state.bias_a(2));
    }
};

template <typename T>
T rad2deg(T radians)
{
    return radians * 180.0 / PI_M;
}

template <typename T>
T deg2rad(T degrees)
{
    return degrees * PI_M / 180.0;
}

template <typename T>
auto set_pose6d(const double t, const Eigen::Matrix<T, 3, 1> &a, const Eigen::Matrix<T, 3, 1> &g,
                const Eigen::Matrix<T, 3, 1> &v, const Eigen::Matrix<T, 3, 1> &p, const Eigen::Matrix<T, 3, 3> &R)
{
    Pose6D rot_kp;
    rot_kp.offset_time = t;
    for (int i = 0; i < 3; i++)
    {
        rot_kp.acc[i] = a(i);
        rot_kp.gyr[i] = g(i);
        rot_kp.vel[i] = v(i);
        rot_kp.pos[i] = p(i);
        for (int j = 0; j < 3; j++)
            rot_kp.rot[i * 3 + j] = R(i, j);
    }
    // Eigen::Map<Eigen::Matrix3d>(rot_kp.rot, 3,3) = R;
    return std::move(rot_kp);
}

template <typename T>
Eigen::VectorXd kroneckerproduct(const Eigen::Matrix<T, 3, 1> &p,const Eigen::Matrix<T, 3, 1> &u){
    Eigen::VectorXd result(9);
    result<<p(0)*u(0),p(0)*u(1),p(0)*u(2),p(1)*u(0),p(1)*u(1),p(1)*u(2),p(2)*u(0),p(2)*u(1),p(2)*u(2);
    return result;
}

#endif
