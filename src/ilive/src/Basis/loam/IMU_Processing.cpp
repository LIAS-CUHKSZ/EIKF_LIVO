#include "IMU_Processing.hpp"


// #include "ilive.hpp"
// ros::NodeHandle m_ros_node_handle1;

double g_lidar_star_tim = 0;


//定义发布者
// ros::Publisher pub_prop;



// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "IMU_propagation");
//     ros::NodeHandle n;
//     pub_prop = n.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 100); 

//     ros::spin();
//     return 0;
// }


//工具函数
boost::array<double, 36> matrixToBoostArray1(const Eigen::Matrix<double, 6, 6>& matrix) {
    boost::array<double, 36> result;
    int index = 0;
    for (int i = 0; i < matrix.rows(); ++i) {
        for (int j = 0; j < matrix.cols(); ++j) {
            result[index++] = matrix(i, j);
        }
    }
    return result;
}

//用来发布propagation之后的状态
void pub_propagation_func(StatesGroup &state, ros::Publisher pub, const ros::Time &ct){
    nav_msgs::Odometry odomAftpropagate;
//读取状态

    Eigen::Vector3d euler_cur = RotMtoEuler(state.rot_end); //publish imu rot
    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));

    odomAftpropagate.header.frame_id = "world";
    odomAftpropagate.child_frame_id = "/aft_mapped";
    odomAftpropagate.header.stamp= ct;
    odomAftpropagate.pose.pose.orientation.x = geoQuat.x;
    odomAftpropagate.pose.pose.orientation.y = geoQuat.y;
    odomAftpropagate.pose.pose.orientation.z = geoQuat.z;
    odomAftpropagate.pose.pose.orientation.w = geoQuat.w;
    odomAftpropagate.pose.pose.position.x = state.pos_end(0);
    odomAftpropagate.pose.pose.position.y = state.pos_end(1);
    odomAftpropagate.pose.pose.position.z = state.pos_end(2);

//读取协防差
    boost::array<double, 36> pose_cov_now;
    Eigen::Matrix<double, 6, 6> subMatrix = state.cov.block<6, 6>(0, 0);
    pose_cov_now=matrixToBoostArray1(subMatrix);
    odomAftpropagate.pose.covariance = pose_cov_now;

    pub.publish(odomAftpropagate);

}





ImuProcess::ImuProcess(StateParameter params) : b_first_frame_(true), imu_need_init_(true), last_imu_(nullptr), start_timestamp_(-1)
{
  //  Eigen::Quaterniond q(0, 1, 0, 0);
  //  Eigen::Vector3d t(0, 0, 0);
  //有待完善：注释了是因为 会说state_inout没有声明
    //state_inout.rot_end = Eye3d;
   // state_inout.pos_end = Eigen::Vector3d(0,0,0);
    //state_inout.vel_end = Eigen::Vector3d(0,0,0);
    init_iter_num = 1;
    cov_acc = Eigen::Vector3d(COV_START_ACC_DIAG, COV_START_ACC_DIAG, COV_START_ACC_DIAG);
    cov_gyr = Eigen::Vector3d(COV_START_GYRO_DIAG, COV_START_GYRO_DIAG, COV_START_GYRO_DIAG);
    mean_acc = Eigen::Vector3d(0, 0, 0);
    mean_gyr = Eigen::Vector3d(0, 0, 0);
    angvel_last = Zero3d;
    MAX_INIT_ITER_NUM = params.max_IMU_calibration_num;
    ext_R_lid_in_imu = params.Lidar_IN_IMU_Rotation;
    ext_t_lid_in_imu = params.Lidar_IN_IMU_Position;
    COV_OMEGA_NOISE_DIAG = params.COV_OMEGA_NOISE_DIAG;
    COV_ACC_NOISE_DIAG = params.COV_ACC_NOISE_DIAG;
    COV_BIAS_ACC_NOISE_DIAG=params.COV_BIAS_ACC_NOISE_DIAG;
    COV_BIAS_GYRO_NOISE_DIAG=params.COV_BIAS_ACC_NOISE_DIAG;
    predict_method = params.m_method;

    pub_prop= nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 100); 


}

ImuProcess::~ImuProcess()
{ /**fout.close();*/
}

void ImuProcess::Reset()
{
    ROS_WARN("Reset ImuProcess");
    angvel_last = Zero3d;

    cov_acc = Eigen::Vector3d(COV_START_ACC_DIAG, COV_START_ACC_DIAG, COV_START_ACC_DIAG);
    cov_gyr = Eigen::Vector3d(COV_START_GYRO_DIAG, COV_START_GYRO_DIAG, COV_START_GYRO_DIAG);
    mean_acc = Eigen::Vector3d(0, 0, 0);
    mean_gyr = Eigen::Vector3d(0, 0, 0);

    imu_need_init_ = true;
    b_first_frame_ = true;
    init_iter_num = 1;
    last_imu_ = nullptr;

    // gyr_int_.Reset(-1, nullptr);
    start_timestamp_ = -1;
    v_imu_.clear();
    IMU_pose.clear();

    cur_pcl_un_.reset(new PointCloudXYZINormal());
}

void ImuProcess::IMU_Initial(const MeasureGroup &meas, StatesGroup &state_inout, int &N)
{

    /** 1. initializing the gravity, gyro bias, acc and gyro covariance
     *     初始化重力,陀螺仪bias,加速度和陀螺仪协方差
     *  2. normalize the acceleration measurenments to unit gravity
     *      归一化加速度测量值到单位重力下
     **/
    ROS_INFO("IMU Initializing: %.1f %%", double(N) / MAX_INIT_ITER_NUM * 100);
    Eigen::Vector3d cur_acc, cur_gyr;

    if (b_first_frame_)
    {
        Reset();
        N = 1;
        b_first_frame_ = false;
    }

    for (const auto &imu : meas.imu)
    {
        const auto &imu_acc = imu->linear_acceleration; // 获取imu线速度
      
        const auto &gyr_acc = imu->angular_velocity;    // 获取imu角速度
        cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    
        cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

        // 计算加速度和角速度均值 - 即期望
        mean_acc += (cur_acc - mean_acc) / N; // mean_acc = Eigen::Vector3d( 0, 0, -9.805 );
        mean_gyr += (cur_gyr - mean_gyr) / N; // mean_gyr = Eigen::Vector3d( 0, 0, 0 );

        // 计算加速度和角速度方差
        N++;
    }

    cov_acc = Eigen::Vector3d(COV_START_ACC_DIAG, COV_START_ACC_DIAG, COV_START_ACC_DIAG);
    cov_gyr = Eigen::Vector3d(COV_START_GYRO_DIAG, COV_START_GYRO_DIAG, COV_START_GYRO_DIAG);
 
    Eigen::Vector3d z_axis = mean_acc / mean_acc.norm();
    Eigen::Matrix3d Ro;
    gram_schmidt(z_axis, Ro);
    state_inout.rot_end = Ro.transpose();
 //   state_inout.rot_end = Eye3d;
    state_inout.pos_end = Eigen::Vector3d(0,0,0);
    state_inout.vel_end = Eigen::Vector3d(0,0,0);
    Eigen::Matrix3d R_IMUI=state_inout.rot_end;
    state_inout.bias_g = mean_gyr;
    state_inout.bias_a = mean_acc-R_IMUI.transpose() * state_inout.gravity;
}

void ImuProcess::lic_state_propagate(const MeasureGroup &meas, StatesGroup &state_inout)
{
    /*** add the imu of the last frame-tail to the of current frame-head ***/
    auto v_imu = meas.imu;
    v_imu.push_front(last_imu_);
    // const double &imu_beg_time = v_imu.front()->header.stamp.toSec();
    const double &imu_end_time = v_imu.back()->header.stamp.toSec();//容器中最后一个时间
    const double &pcl_beg_time = meas.lidar_beg_time;

    /*** sort point clouds by offset time ***/
    PointCloudXYZINormal pcl_out = *(meas.lidar);
    std::sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
    const double &pcl_end_time = pcl_beg_time + pcl_out.points.back().curvature / double(1000);
    double end_pose_dt = pcl_end_time - imu_end_time;

    state_inout = imu_preintegration(state_inout, v_imu, end_pose_dt);


    last_imu_ = meas.imu.back();
}

// Avoid abnormal state input
bool check_state(StatesGroup &state_inout)
{
    bool is_fail = false;
    for (int idx = 0; idx < 3; idx++)
    {
        if (fabs(state_inout.vel_end(idx)) > 100)
        {
            is_fail = true;
            scope_color(ANSI_COLOR_RED_BG);
            for (int i = 0; i < 10; i++)
            {
                cout << __FILE__ << ", " << __LINE__ << ", check_state fail !!!! " << state_inout.vel_end.transpose() << endl;
            }
            state_inout.vel_end(idx) = 0.0;
        }
    }
    return is_fail;
}

// Avoid abnormal state input
void check_in_out_state(const StatesGroup &state_in, StatesGroup &state_inout)
{
    if ((state_in.pos_end - state_inout.pos_end).norm() > 1.0)
    {
        scope_color(ANSI_COLOR_RED_BG);
        for (int i = 0; i < 10; i++)
        {
            cout << __FILE__ << ", " << __LINE__ << ", check_in_out_state fail !!!! " << state_in.pos_end.transpose() << " | "
                 << state_inout.pos_end.transpose() << endl;
        }
        state_inout.pos_end = state_in.pos_end;
    }
}

std::mutex g_imu_premutex;

StatesGroup ImuProcess::imu_preintegration(const StatesGroup &state_in, std::deque<sensor_msgs::Imu::ConstPtr> &vec_imu, double end_pose_dt)
{
    std::unique_lock<std::mutex> lock(g_imu_premutex);
    StatesGroup state_inout = state_in;
    if (check_state(state_inout))
    {
        state_inout.display(state_inout, "state_inout");
        state_in.display(state_in, "state_in");
    }
    Eigen::Vector3d acc_imu(0, 0, 0), angvel_avr(0, 0, 0), acc_avr(0, 0, 0), vel_imu(0, 0, 0), pos_imu(0, 0, 0);
    vel_imu = state_inout.vel_end;
    pos_imu = state_inout.pos_end;
    Eigen::Matrix3d R_imu(state_inout.rot_end);
    Eigen::MatrixXd F_x(Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES>::Identity());
    Eigen::MatrixXd cov_w(Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES>::Zero());
    double dt = 0;
    bool if_first_imu = 1;
    Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> Cov_predict(state_inout.cov);

    for (std::deque<sensor_msgs::Imu::ConstPtr>::iterator it_imu = vec_imu.begin(); it_imu != (vec_imu.end() - 1); it_imu++)
    {
        sensor_msgs::Imu::ConstPtr head = *(it_imu);
        sensor_msgs::Imu::ConstPtr tail = *(it_imu + 1);

        angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x), 0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
            0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
        acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
            0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y), 0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

        angvel_avr -= state_inout.bias_g;

        acc_avr = acc_avr - state_inout.bias_a;

        if (tail->header.stamp.toSec() < state_inout.last_update_time)
        {
            continue;
        }

        if (if_first_imu)
        {
            if_first_imu = 0;
            dt = tail->header.stamp.toSec() - state_inout.last_update_time;
        }
        else
        {
            dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
        }
        dt = std::max( -0.05, std::min( dt, 0.05) );
        cout<<dt<<endl;


//目标：把这部分发布出去

        // The following code will excute the preintegration of the IMU covariance.
        compute_covariance(state_inout, Cov_predict , angvel_avr, acc_avr,dt );
        state_inout.cov=Cov_predict; 
        Eigen::Matrix3d Exp_f = Exp(angvel_avr, dt);
        // The following code will excute the preintegration of the IMU state.
        R_imu = R_imu * Exp_f;
        acc_imu = R_imu * acc_avr - state_inout.gravity;
        pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;
        vel_imu = vel_imu + acc_imu * dt;
        angvel_last = angvel_avr;
        acc_s_last = acc_imu;

        StatesGroup state_tmp;
        state_tmp.vel_end = vel_imu;
        state_tmp.rot_end = R_imu;
        state_tmp.pos_end = pos_imu;
        state_tmp.cov = Cov_predict;

     
        pub_propagation_func(state_tmp, pub_prop,tail->header.stamp);
        
        // pub_propagation_func(state_tmp, pub_prop,tail->header.stamp);

    }
    //In multi-sensor platform, we will align the state with other sensors
    dt = end_pose_dt;
    state_inout.last_update_time = vec_imu.back()->header.stamp.toSec() + dt;
    // if (dt > 0.05)
    // {
    //     scope_color(ANSI_COLOR_RED_BOLD);
    //     for (int i = 0; i < 1; i++)
    //     {
    //         cout << __FILE__ << ", " << __LINE__ << "dt = " << dt << endl;
    //     }
    //     dt = 0.05;
    // }
    // dt = std::max( -0.05, std::min( dt, 0.05) );

    compute_covariance(state_inout, Cov_predict , angvel_avr, acc_avr,dt);
    state_inout.vel_end = vel_imu + acc_imu * dt;
    state_inout.rot_end = R_imu * Exp(angvel_avr, dt);
    state_inout.pos_end = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;
    state_inout.cov = Cov_predict;
    
    return state_inout;
}

/**
 * @note Covariance compute
 * @param state_in : rotation,position, velocity
 * @param P : Covariance after the predict process
 * @param angvel_avr, acc_avg: used for the covariance propagation of EKF
 * @param 
 */
void ImuProcess::compute_covariance(const StatesGroup state_in, Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> &P, const Eigen::Vector3d angvel_avr, const Eigen::Vector3d acc_avr,const double dt){
    Eigen::Vector3d vel_imu(0, 0, 0), pos_imu(0, 0, 0);
    vel_imu = state_in.vel_end;
    pos_imu = state_in.pos_end;
    Eigen::Matrix3d R_imu(state_in.rot_end);
    Eigen::MatrixXd F_x(Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES>::Identity());
    Eigen::MatrixXd cov_w(Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES>::Zero());
    int rid = 0;
    int pid = 3;
    int vid = 6;        
    int bgid = 9;
    int baid = 12;
    int ng_id = 0;
    int na_id = 3;
    int nbg_id = 6;
    int nba_id = 9;
    Eigen::Matrix3d acc_avr_skew;
    Eigen::Matrix3d Exp_f = Exp(angvel_avr, dt);
    acc_avr_skew << SKEW_SYM_MATRIX(acc_avr);
    //Eigen::Matrix3d Jr_omega_dt = right_jacobian_of_rotion_matrix<double>(angvel_avr*dt);
    Eigen::Matrix3d Jr_omega_dt = Eigen::Matrix3d::Identity();
       if(predict_method=="InEKF"||predict_method=="EIKF")
       {
            F_x.block<3, 3>(vid, rid) << SKEW_SYM_MATRIX(-state_in.gravity);
            F_x.block<3, 3>(pid, vid).setIdentity();
            F_x.block<3, 3>(rid, bgid).noalias() = -R_imu;
            F_x.block<3, 3>(pid, bgid).noalias() = -(vec_to_hat(pos_imu)) * R_imu;
            F_x.block<3, 3>(vid, bgid).noalias() = -(vec_to_hat(vel_imu)) * R_imu;
            F_x.block<3, 3>(vid, baid).noalias() = -R_imu;
            Eigen::MatrixXd Tem = F_x.block<15, 15>(0, 0);
            Tem = Eigen::Matrix<double, 15, 15>::Identity() + Tem * dt + 0.5 * Tem * Tem * dt * dt + 0.167 * Tem * Tem * Tem * dt * dt * dt;
            F_x.block<15, 15>(0, 0) = Tem;

            Eigen::Matrix3d cov_acc_diag, cov_gyr_diag, cov_omega_diag;
            cov_omega_diag = Eigen::Vector3d(COV_OMEGA_NOISE_DIAG, COV_OMEGA_NOISE_DIAG, COV_OMEGA_NOISE_DIAG).asDiagonal();
            cov_acc_diag = Eigen::Vector3d(COV_ACC_NOISE_DIAG, COV_ACC_NOISE_DIAG, COV_ACC_NOISE_DIAG).asDiagonal();
            cov_gyr_diag = Eigen::Vector3d(COV_GYRO_NOISE_DIAG, COV_GYRO_NOISE_DIAG, COV_GYRO_NOISE_DIAG).asDiagonal();
            // cov_w.block<3, 3>(0, 0) = cov_omega_diag * dt * dt;
            Eigen::Matrix<double, 15, 12> G = Eigen::Matrix<double, 15, 12>::Zero();
        
            G.block(rid, ng_id, 3, 3) = R_imu * dt;
            G.block(pid, ng_id, 3, 3) = vec_to_hat(pos_imu) * R_imu * dt;
            G.block(vid, ng_id, 3, 3) = vec_to_hat(vel_imu) * R_imu * dt;
            G.block(vid, na_id, 3, 3) = R_imu * dt;
            G.block(bgid, nbg_id, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity() * dt;
            G.block(baid, nba_id, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity() * dt;

            Eigen::Matrix<double, 12, 12> Cov_n = Eigen::Matrix<double, 12, 12>::Zero();
            Cov_n.block<3, 3>(0, 0) = cov_omega_diag; ///??????TODO: to check the covariance
            Cov_n.block<3, 3>(3, 3) = cov_acc_diag;   //???
            Cov_n.block<3, 3>(6, 6) = Eigen::Vector3d(COV_BIAS_GYRO_NOISE_DIAG, COV_BIAS_GYRO_NOISE_DIAG, COV_BIAS_GYRO_NOISE_DIAG).asDiagonal();
            Cov_n.block<3, 3>(9, 9) = Eigen::Vector3d(COV_BIAS_ACC_NOISE_DIAG, COV_BIAS_ACC_NOISE_DIAG, COV_BIAS_ACC_NOISE_DIAG).asDiagonal();

            Eigen::Matrix<double, 15, 15> Qd = Eigen::Matrix<double, 15, 15>::Zero();
            Qd = G * Cov_n * G.transpose();
            Qd = 0.5 * (Qd + Qd.transpose());
            cov_w.block<15, 15>(0, 0) = Qd;
            P = F_x * state_in.cov * F_x.transpose() + cov_w;}
        else{
            /* covariance propagation */
            F_x.block<3, 3>(0, 0) = Exp_f.transpose();
            F_x.block<3, 3>(0, 9) = -Eye3d * dt;
            //F_x.block<3, 3>(0, 9) = -Jr_omega_dt * dt;
            // F_x.block<3,3>(3,0)  = -R_imu * off_vel_skew * dt;
            //F_x.block<3, 3>(3, 3) = Eye3d; // Already the identity.
            F_x.block<3, 3>(3, 6) = Eye3d * dt;
            F_x.block<3, 3>(6, 0) = -R_imu * acc_avr_skew * dt;
            F_x.block<3, 3>(6, 12) = -R_imu * dt;
            Eigen::Matrix3d cov_acc_diag, cov_gyr_diag, cov_omega_diag;
            cov_omega_diag = Eigen::Vector3d(COV_OMEGA_NOISE_DIAG, COV_OMEGA_NOISE_DIAG, COV_OMEGA_NOISE_DIAG).asDiagonal();
            cov_acc_diag = Eigen::Vector3d(COV_ACC_NOISE_DIAG, COV_ACC_NOISE_DIAG, COV_ACC_NOISE_DIAG).asDiagonal();
            cov_gyr_diag = Eigen::Vector3d(COV_GYRO_NOISE_DIAG, COV_GYRO_NOISE_DIAG, COV_GYRO_NOISE_DIAG).asDiagonal();
            cov_w.block<3, 3>(0, 0) = cov_omega_diag * dt * dt;
        // cov_w.block<3, 3>(0, 0) = Jr_omega_dt * cov_omega_diag * Jr_omega_dt * dt * dt;
            cov_w.block<3, 3>(6, 6) = R_imu * cov_acc_diag * R_imu.transpose() * dt * dt;
            //cov_w.block<3, 3>(6, 6) = cov_acc_diag * dt * dt;
            cov_w.block<3, 3>(9, 9).diagonal() =
                Eigen::Vector3d(COV_BIAS_GYRO_NOISE_DIAG, COV_BIAS_GYRO_NOISE_DIAG, COV_BIAS_GYRO_NOISE_DIAG) * dt * dt; // bias gyro covariance
            cov_w.block<3, 3>(12, 12).diagonal() =
                Eigen::Vector3d(COV_BIAS_ACC_NOISE_DIAG, COV_BIAS_ACC_NOISE_DIAG, COV_BIAS_ACC_NOISE_DIAG) * dt * dt; // bias acc covariance
            P= F_x * state_in.cov * F_x.transpose() + cov_w;
        }
}



/**
 * @note Lidar点去畸变
 * @param meas : 输入的Lidar和IMU数据
 * @param _state_inout : Lidar的状态信息
 * @param pcl_out : 去畸变后的点
 */

void ImuProcess::lic_point_cloud_undistort(const MeasureGroup &meas, const StatesGroup &_state_inout, PointCloudXYZINormal &pcl_out)
{

    StatesGroup state_out = _state_inout;
    auto v_imu = meas.imu;                                           // 提取IMu数据
    v_imu.push_front(last_imu_);                                     // 将上一时刻的imu数据放入队列,这样就可以从上一时刻进行连续传播
    const double &imu_end_time = v_imu.back()->header.stamp.toSec(); // 拿到当前帧尾部的imu的时间
    const double &pcl_beg_time = meas.lidar_beg_time;                // pcl开始的时间戳

    pcl_out = *(meas.lidar);                                            // 提取出lidar数据
    std::sort(pcl_out.points.begin(), pcl_out.points.end(), time_list); // 对lidar数据按时间排序
    const double &pcl_end_time = pcl_beg_time + pcl_out.points.back().curvature / double(1000);

    IMU_pose.clear();
    // IMUpose.push_back(set_pose6d(0.0, Zero3d, Zero3d, state.vel_end, state.pos_end, state.rot_end));
    IMU_pose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, state_out.vel_end, state_out.pos_end, state_out.rot_end));

    Eigen::Vector3d acc_imu, angvel_avr, acc_avr, v_Ii(state_out.vel_end), p_Ii(state_out.pos_end);
    Eigen::Matrix3d R_Ii(state_out.rot_end);
    Eigen::MatrixXd F_x(Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES>::Identity());
    Eigen::MatrixXd cov_w(Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES>::Zero());
    double dt = 0;

    for (auto it_imu = v_imu.begin(); it_imu != (v_imu.end() - 1); it_imu++)
    {
        auto &&head = *(it_imu);     // 拿到当前帧的imu数据
        auto &&tail = *(it_imu + 1); // 拿到下一帧的imu数据
        angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x), 0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
            0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
        acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
            0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y), 0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);
        // 减去bias
        angvel_avr -= state_out.bias_g;
        acc_avr = acc_avr - state_out.bias_a;
        dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
        /* covariance propagation */

        Eigen::Matrix3d acc_avr_skew;                // FAST-LIO 公式(2)
        Eigen::Matrix3d Exp_f = Exp(angvel_avr, dt); // 角速度+delta增量后->旋转矩阵
        acc_avr_skew << SKEW_SYM_MATRIX(acc_avr);    // 加速度向量的反对称矩阵
        R_Ii = R_Ii * Exp_f;                         // 基于角速度,求解相邻时刻的IMU的旋转

        /* Specific acceleration (global frame) of IMU */
        acc_imu = R_Ii * acc_avr- state_out.gravity; // 全局坐标系下的加速度量

        /* propagation of IMU : IMU位置的传播量*/
        p_Ii = p_Ii + v_Ii * dt + 0.5 * acc_imu * dt * dt;

        /* velocity of IMU : IMU速度的传播量*/
        v_Ii = v_Ii + acc_imu * dt;

        /* save the poses at each IMU measurements */
        angvel_last = angvel_avr; // 保存角速度和加速度
        acc_s_last = acc_imu;
        double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;
        // std::cout<<"acc "<<acc_imu.transpose()<<"vel "<<acc_imu.transpose()<<"vel "<<pos_imu.transpose()<<std::endl;
        IMU_pose.push_back(set_pose6d(offs_t, acc_imu, angvel_avr, v_Ii, p_Ii, R_Ii));
    }

    // state_out is IMU's state at [lidar end time]
    dt = pcl_end_time - imu_end_time;                               // Lidar和IMU的最后时刻时间差   NOTE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    state_out.vel_end = v_Ii + acc_imu * dt;                        // 通过IMU的加速度预估Lidar最后时刻时的速度
    state_out.rot_end = R_Ii * Exp(angvel_avr, dt);                 // 预估最后时刻的IMU在世界系下的旋转
    state_out.pos_end = p_Ii + v_Ii * dt + 0.5 * acc_imu * dt * dt; // 预估最后时刻的IMU在世界系下的位置

    /*** undistort each lidar point (backward propagation) ***/
    auto it_pcl = pcl_out.points.end() - 1;
    for (auto it_kp = IMU_pose.end() - 1; it_kp != IMU_pose.begin(); it_kp--)
    {
        bool comp = false;
        if (it_kp == IMU_pose.begin() + 1)
        {
            comp = true;
        }
        auto head = it_kp - 1;
        auto tail = it_kp;
        R_Ii << MAT_FROM_ARRAY(head->rot);
        acc_imu << VEC_FROM_ARRAY(head->acc);
        // std::cout<<"head imu acc: "<<acc_imu.transpose()<<std::endl;
        v_Ii << VEC_FROM_ARRAY(head->vel);
        p_Ii << VEC_FROM_ARRAY(head->pos);
        angvel_avr << VEC_FROM_ARRAY(head->gyr);

        for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--)
        {
            dt = it_pcl->curvature / double(1000) - head->offset_time;
            // IMU state changes from the ith time to the j-th time
            Eigen::Matrix3d delta_R(Exp(angvel_avr, dt));
            // Imu's position in global at [t_j]
            Eigen::Vector3d p_Ij = p_Ii + v_Ii * dt + 0.5 * acc_imu * dt * dt; /// IMU state changes from the ith time to the end time in the global frame
            // IMU's rotation in global at [t_j]
            Eigen::Matrix3d R_Ij(R_Ii * delta_R); // IMU state  on the global time at  time end
                                                  // Reminder: state_out means the IMU state at lidar end time
                                                  // Description 2
            Eigen::Matrix4d T_L_in_I;
            T_L_in_I.block<3, 3>(0, 0) = ext_R_lid_in_imu;
            T_L_in_I.block<3, 1>(0, 3) = ext_t_lid_in_imu;
            T_L_in_I.block<1, 4>(3, 0) << 0, 0, 0, 1;

            Eigen::Matrix4d T_I_in_L;
            T_I_in_L.block<3, 3>(0, 0) = ext_R_lid_in_imu.transpose();
            T_I_in_L.block<3, 1>(0, 3) = -ext_R_lid_in_imu.transpose() * ext_t_lid_in_imu;
            T_I_in_L.block<1, 4>(3, 0) << 0, 0, 0, 1;

            Eigen::Matrix4d T_Ij;
            T_Ij.block<3, 3>(0, 0) = R_Ij;
            T_Ij.block<3, 1>(0, 3) = p_Ij;
            T_Ij.block<1, 4>(3, 0) << 0, 0, 0, 1;

            Eigen::Matrix4d T_G_in_Iend;
            T_G_in_Iend.block<3, 3>(0, 0) = state_out.rot_end.transpose();
            T_G_in_Iend.block<3, 1>(0, 3) = -state_out.rot_end.transpose() * state_out.pos_end;
            T_G_in_Iend.block<1, 4>(3, 0) << 0, 0, 0, 1;

            Eigen::Matrix4d T_Lj_in_Lend = T_I_in_L * T_G_in_Iend * T_Ij * T_L_in_I;
            Eigen::Vector4d p1_fj_in_Lj(it_pcl->x, it_pcl->y, it_pcl->z, 1);
            Eigen::Vector4d P_compensate1 = T_Lj_in_Lend * p1_fj_in_Lj;

            it_pcl->x = P_compensate1(0);
            it_pcl->y = P_compensate1(1);
            it_pcl->z = P_compensate1(2);
            if (it_pcl == pcl_out.points.begin())
                break;
        }
    }
}

/**
 * @note 通过测量数据meas(IMU数据和Lidar数据)计算LIO的状态和去畸变后的Lidar点云数据
 * @param meas : 输入的Lidar和IMU数据
 * @param stat : Lidar状态
 * @param cur_pcl_un_ : 去畸变后的点云
 */
void ImuProcess::Process(const MeasureGroup &meas, StatesGroup &stat, PointCloudXYZINormal::Ptr cur_pcl_un_)
{
    double t1, t2, t3;
    t1 = omp_get_wtime();

    if (meas.imu.empty()) // 判断IMU数据
    {
        std::cout << "no imu data" << std::endl;
        return;
    };
    if(meas.lidar == nullptr){
        ROS_INFO("no lidar data in the IMU process");
    } // 判断Lidar数据

    if (imu_need_init_) // 初始化IMU数据
    {

        IMU_Initial(meas, stat, init_iter_num);

        imu_need_init_ = true;

        last_imu_ = meas.imu.back(); // 获取当前处理的最后一个imu数据

        if (init_iter_num > MAX_INIT_ITER_NUM)
        {
            imu_need_init_ = false;
            ROS_INFO(
                "IMU Initials: Gravity: %.4f %.4f %.4f; state.bias_g: %.4f %.4f %.4f; state.bias_a: %.4f %.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f",
                stat.gravity[0], stat.gravity[1], stat.gravity[2], stat.bias_g[0], stat.bias_g[1], stat.bias_g[2], stat.bias_a[0],stat.bias_a[1],stat.bias_a[2],cov_acc[0],
                cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1], cov_gyr[2]);
            stat.if_state_inited = true;
        }

        return;
    }
    lic_point_cloud_undistort(meas, stat, *cur_pcl_un_);
    lic_state_propagate(meas, stat);
    last_imu_ = meas.imu.back();
}




void ImuProcess::gram_schmidt(const Eigen::Vector3d &gravity_inI, Eigen::Matrix3d &R_GtoI) {

    // This will find an orthogonal vector to gravity which is our local z-axis
    // We need to ensure we normalize after each one such that we obtain unit vectors
    Eigen::Vector3d z_axis = gravity_inI / gravity_inI.norm();
    Eigen::Vector3d x_axis, y_axis;
    Eigen::Vector3d e_1(1.0, 0.0, 0.0);
    // Eigen::Vector3d e_2(0.0, 1.0, 0.0);
    // double inner1 = e_1.dot(z_axis) / z_axis.norm();
    // double inner2 = e_2.dot(z_axis) / z_axis.norm();
    // if (fabs(inner1) < fabs(inner2)) {
    //   x_axis = z_axis.cross(e_1);
    //   x_axis = x_axis / x_axis.norm();
    //   y_axis = z_axis.cross(x_axis);
    //   y_axis = y_axis / y_axis.norm();
    // } else {
    //   x_axis = z_axis.cross(e_2);
    //   x_axis = x_axis / x_axis.norm();
    //   y_axis = z_axis.cross(x_axis);
    //   y_axis = y_axis / y_axis.norm();
    // }

    // Original method
    // https://en.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process
    x_axis = e_1 - z_axis * z_axis.transpose() * e_1;
    x_axis = x_axis / x_axis.norm();
    y_axis = z_axis.cross(x_axis);
    y_axis = y_axis / y_axis.norm();

    // Rotation from our global (where gravity is only along the z-axis) to the local one
    R_GtoI.block(0, 0, 3, 1) = x_axis;
    R_GtoI.block(0, 1, 3, 1) = y_axis;
    R_GtoI.block(0, 2, 3, 1) = z_axis;
  }

// /**
//  * @note Predict the state in RK4 method
//  * @param w_hat1,w_hat2 : two frames of the input of the angle vel 
//  * @param a_hat1,a_hat2 : two frames of the input of the acc
//  * @param state : the initial state to integrate
//  * @param new_q, new_v,new_p : output the integrated state
//  */
// void ImuProcess::predict_mean_rk4(std::shared_ptr<StatesGroup> state, double dt, const Eigen::Vector3d &w_hat1, const Eigen::Vector3d &a_hat1,
//                                   const Eigen::Vector3d &w_hat2, const Eigen::Vector3d &a_hat2, Eigen::Vector4d &new_q,
//                                   Eigen::Vector3d &new_v, Eigen::Vector3d &new_p) {

//   // Pre-compute things
//   Eigen::Vector3d w_hat = w_hat1;
//   Eigen::Vector3d a_hat = a_hat1;
//   Eigen::Vector3d w_alpha = (w_hat2 - w_hat1) / dt;
//   Eigen::Vector3d a_jerk = (a_hat2 - a_hat1) / dt;

//   // y0 ================
//   Eigen::Vector4d q_0 = state->_imu->quat();
//   Eigen::Vector3d p_0 = state->_imu->pos();
//   Eigen::Vector3d v_0 = state->_imu->vel();

//   // k1 ================
//   Eigen::Vector4d dq_0 = {0, 0, 0, 1};
//   Eigen::Vector4d q0_dot = 0.5 * Omega(w_hat) * dq_0;
//   Eigen::Vector3d p0_dot = v_0;
//   Eigen::Matrix3d R_Gto0 = quat_2_Rot(quat_multiply(dq_0, q_0));
//   Eigen::Vector3d v0_dot = R_Gto0.transpose() * a_hat - _gravity;

//   Eigen::Vector4d k1_q = q0_dot * dt;
//   Eigen::Vector3d k1_p = p0_dot * dt;
//   Eigen::Vector3d k1_v = v0_dot * dt;

//   // k2 ================
//   w_hat += 0.5 * w_alpha * dt;
//   a_hat += 0.5 * a_jerk * dt;

//   Eigen::Vector4d dq_1 = quatnorm(dq_0 + 0.5 * k1_q);
//   // Eigen::Vector3d p_1 = p_0+0.5*k1_p;
//   Eigen::Vector3d v_1 = v_0 + 0.5 * k1_v;

//   Eigen::Vector4d q1_dot = 0.5 * Omega(w_hat) * dq_1;
//   Eigen::Vector3d p1_dot = v_1;
//   Eigen::Matrix3d R_Gto1 = quat_2_Rot(quat_multiply(dq_1, q_0));
//   Eigen::Vector3d v1_dot = R_Gto1.transpose() * a_hat - _gravity;

//   Eigen::Vector4d k2_q = q1_dot * dt;
//   Eigen::Vector3d k2_p = p1_dot * dt;
//   Eigen::Vector3d k2_v = v1_dot * dt;

//   // k3 ================
//   Eigen::Vector4d dq_2 = quatnorm(dq_0 + 0.5 * k2_q);
//   // Eigen::Vector3d p_2 = p_0+0.5*k2_p;
//   Eigen::Vector3d v_2 = v_0 + 0.5 * k2_v;

//   Eigen::Vector4d q2_dot = 0.5 * Omega(w_hat) * dq_2;
//   Eigen::Vector3d p2_dot = v_2;
//   Eigen::Matrix3d R_Gto2 = quat_2_Rot(quat_multiply(dq_2, q_0));
//   Eigen::Vector3d v2_dot = R_Gto2.transpose() * a_hat - _gravity;

//   Eigen::Vector4d k3_q = q2_dot * dt;
//   Eigen::Vector3d k3_p = p2_dot * dt;
//   Eigen::Vector3d k3_v = v2_dot * dt;

//   // k4 ================
//   w_hat += 0.5 * w_alpha * dt;
//   a_hat += 0.5 * a_jerk * dt;

//   Eigen::Vector4d dq_3 = quatnorm(dq_0 + k3_q);
//   // Eigen::Vector3d p_3 = p_0+k3_p;
//   Eigen::Vector3d v_3 = v_0 + k3_v;

//   Eigen::Vector4d q3_dot = 0.5 * Omega(w_hat) * dq_3;
//   Eigen::Vector3d p3_dot = v_3;
//   Eigen::Matrix3d R_Gto3 = quat_2_Rot(quat_multiply(dq_3, q_0));
//   Eigen::Vector3d v3_dot = R_Gto3.transpose() * a_hat - _gravity;

//   Eigen::Vector4d k4_q = q3_dot * dt;
//   Eigen::Vector3d k4_p = p3_dot * dt;
//   Eigen::Vector3d k4_v = v3_dot * dt;

//   // y+dt ================
//   Eigen::Vector4d dq = quatnorm(dq_0 + (1.0 / 6.0) * k1_q + (1.0 / 3.0) * k2_q + (1.0 / 3.0) * k3_q + (1.0 / 6.0) * k4_q);
//   new_q = quat_multiply(dq, q_0);
//   new_p = p_0 + (1.0 / 6.0) * k1_p + (1.0 / 3.0) * k2_p + (1.0 / 3.0) * k3_p + (1.0 / 6.0) * k4_p;
//   new_v = v_0 + (1.0 / 6.0) * k1_v + (1.0 / 3.0) * k2_v + (1.0 / 3.0) * k3_v + (1.0 / 6.0) * k4_v;
// }