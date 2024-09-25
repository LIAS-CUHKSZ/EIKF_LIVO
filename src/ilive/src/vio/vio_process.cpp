#include "ilive.hpp"
#include "tools_mem_used.h"
#include "tools_logger.hpp"


// proscess the image from ros reader


Common_tools::Cost_time_logger g_cost_time_logger;
std::shared_ptr<Common_tools::ThreadPool> m_thread_pool_ptr;
double g_vio_frame_cost_time = 0;
double g_lio_frame_cost_time = 0;
int g_flag_if_first_rec_img = 1;

#define USING_CERES 0
void dump_lio_state_to_log(FILE *fp)
{
    if (fp != nullptr && g_camera_lidar_queue.m_if_dump_log)
    {
        Eigen::Vector3d rot_angle = Sophus::SO3d(Eigen::Quaterniond(g_lio_state.rot_end)).log();
        Eigen::Vector3d rot_ext_i2c_angle = Sophus::SO3d(Eigen::Quaterniond(g_lio_state.rot_ext_i2c)).log();
        fprintf(fp, "%lf ", g_lio_state.last_update_time - g_camera_lidar_queue.m_first_imu_time); // Time   [0]
        fprintf(fp, "%lf %lf %lf ", rot_angle(0), rot_angle(1), rot_angle(2));                     // Angle  [1-3]
        fprintf(fp, "%lf %lf %lf ", g_lio_state.pos_end(0), g_lio_state.pos_end(1),
                g_lio_state.pos_end(2));            // Pos    [4-6]
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0); // omega  [7-9]
        fprintf(fp, "%lf %lf %lf ", g_lio_state.vel_end(0), g_lio_state.vel_end(1),
                g_lio_state.vel_end(2));            // Vel    [10-12]
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0); // Acc    [13-15]
        fprintf(fp, "%lf %lf %lf ", g_lio_state.bias_g(0), g_lio_state.bias_g(1),
                g_lio_state.bias_g(2)); // Bias_g [16-18]
        fprintf(fp, "%lf %lf %lf ", g_lio_state.bias_a(0), g_lio_state.bias_a(1),
                g_lio_state.bias_a(2)); // Bias_a [19-21]
        fprintf(fp, "%lf %lf %lf ", g_lio_state.gravity(0), g_lio_state.gravity(1),
                g_lio_state.gravity(2)); // Gravity[22-24]
        fprintf(fp, "%lf %lf %lf ", rot_ext_i2c_angle(0), rot_ext_i2c_angle(1),
                rot_ext_i2c_angle(2)); // Rot_ext_i2c[25-27]
        fprintf(fp, "%lf %lf %lf ", g_lio_state.pos_ext_i2c(0), g_lio_state.pos_ext_i2c(1),
                g_lio_state.pos_ext_i2c(2)); // pos_ext_i2c [28-30]
        fprintf(fp, "%lf %lf %lf %lf ", g_lio_state.cam_intrinsic(0), g_lio_state.cam_intrinsic(1), g_lio_state.cam_intrinsic(2),
                g_lio_state.cam_intrinsic(3));       // Camera Intrinsic [31-34]
        fprintf(fp, "%lf ", g_lio_state.td_ext_i2c); // Camera Intrinsic [35]
        for (int idx = 0; idx < DIM_OF_STATES; idx++) // Cov    [36-64]
        {
            fprintf(fp, "%.9f ", sqrt(g_lio_state.cov(idx, idx)));
        }
        fprintf(fp, "%lf %lf ", g_lio_frame_cost_time, g_vio_frame_cost_time); // costime [65-66]
        fprintf(fp, "\r\n");
        fflush(fp);
    }
}

double g_last_stamped_mem_mb = 0;
std::string append_space_to_bits(std::string &in_str, int bits)
{
    while (in_str.length() < bits)
    {
        in_str.append(" ");
    }
    return in_str;
}


void ILIVE::print_dash_board()
{
    int mem_used_mb = (int)(Common_tools::get_RSS_Mb());

    std::string out_str_line_1;
    out_str_line_1 = std::string("| System-time | Valid LiDAR points | Valid camera points |  Pts in maps | Memory used (Mb) | LiDAR-frame | Camera-frame |") ;
    out_str_line_1.insert(58, ANSI_COLOR_YELLOW_BOLD, 7);
    out_str_line_1.insert(14, ANSI_COLOR_BLUE_BOLD, 7);
    out_str_line_1.insert(42, ANSI_COLOR_GREEN_BOLD, 7);
    out_str_line_1.insert(22, ANSI_COLOR_RED_BOLD, 7);
    out_str_line_1.insert(0, ANSI_COLOR_WHITE_BOLD, 7);
    //                                    1             16            30             45             60
    // clang-format off
    if( (mem_used_mb - g_last_stamped_mem_mb < 1024 ) && g_last_stamped_mem_mb != 0 )
    {
        cout  << ANSI_DELETE_CURRENT_LINE << ANSI_DELETE_LAST_LINE ;
    }
    else
    {
        cout << "\r\n" << endl;
        cout << ANSI_COLOR_WHITE_BOLD << "======================= ILIVE Dashboard ======================" << ANSI_COLOR_RESET << endl;
        g_last_stamped_mem_mb = mem_used_mb ;
        // cout << out_str_line_1 << endl;
        
    }
    std::string out_str_line_2;
    // out_str_line_1 = std::string("| System-time | Valid LiDAR points | Valid camera points |  Pts in maps | Memory used (Mb) |") ;
    //                                    1             16            30             45             60
    // clang-format on
    out_str_line_2.reserve(1e3);


    out_str_line_2.append("|   ").append(Common_tools::get_current_time_str());
    append_space_to_bits(out_str_line_2, 14);
    out_str_line_2.append("|    ").append(std::to_string(valid_lid_points));
    append_space_to_bits(out_str_line_2, 35);
    out_str_line_2.append("|    ").append(std::to_string(valid_cam_points));
    append_space_to_bits(out_str_line_2, 57);
    out_str_line_2.append("| ").append(std::to_string(m_map_rgb_pts.m_rgb_pts_vec.size()));
    append_space_to_bits(out_str_line_2, 72);
    out_str_line_2.append("|    ").append(std::to_string(mem_used_mb));
    append_space_to_bits(out_str_line_2, 90);
    out_str_line_2.append("|    ").append(std::to_string(g_LiDAR_frame_index));
    append_space_to_bits(out_str_line_2, 108);
    out_str_line_2.append("|    ").append(std::to_string(g_camera_frame_idx)).append(" |");

    out_str_line_2.insert(58, ANSI_COLOR_YELLOW, 7);
    out_str_line_2.insert(14, ANSI_COLOR_BLUE, 7);
    out_str_line_2.insert(42, ANSI_COLOR_GREEN, 7);
    out_str_line_2.insert(22, ANSI_COLOR_RED, 7);
    out_str_line_2.insert(0, ANSI_COLOR_WHITE, 7);

    // out_str_line_1.insert(58, ANSI_COLOR_YELLOW_BOLD, 7);
    // out_str_line_1.insert(14, ANSI_COLOR_BLUE_BOLD, 7);
    // out_str_line_1.insert(42, ANSI_COLOR_GREEN_BOLD, 7);
    // out_str_line_1.insert(22, ANSI_COLOR_RED_BOLD, 7);
    // out_str_line_1.insert(0, ANSI_COLOR_WHITE_BOLD, 7);
    cout << out_str_line_1 << endl;
    cout << out_str_line_2 << ANSI_COLOR_RESET << "          ";
    ANSI_SCREEN_FLUSH;
}

void ILIVE::set_initial_state_cov(StatesGroup &state)
{
    scope_color(ANSI_COLOR_RED_BOLD);

    state.cov.block(0, 0, 3, 3) = m_imu_cov*mat_3_3::Identity() * 1e-5;   // R
    state.cov.block(3, 3, 3, 3) = m_imu_cov*mat_3_3::Identity() * 1e-5;   // T
    state.cov.block(6, 6, 3, 3) = m_imu_cov*mat_3_3::Identity() * 1e-5;   // vel
    state.cov.block(9, 9, 3, 3) = m_imu_cov*mat_3_3::Identity() * 1e-4;   // bias_g
    state.cov.block(12, 12, 3, 3) = mat_3_3::Identity() * 1e-2; // bias_a

#if USE_dt
    state.cov(15,15) =0.00001; //ext_t_i2c时间偏移
#endif
}

cv::Mat ILIVE::generate_control_panel_img()
{
    int line_y = 40;
    int padding_x = 10;
    int padding_y = line_y * 0.7;
    cv::Mat res_image = cv::Mat(line_y * 3 + 1 * padding_y, 960, CV_8UC3, cv::Scalar::all(0));
    char temp_char[128];
    sprintf(temp_char, "Click this windows to enable the keyboard controls.");
    cv::putText(res_image, std::string(temp_char), cv::Point(padding_x, line_y * 0 + padding_y), cv::FONT_HERSHEY_COMPLEX, 1,
                cv::Scalar(0, 255, 255), 2, 8, 0);
    sprintf(temp_char, "Press 'S' or 's' key to save current map");
    cv::putText(res_image, std::string(temp_char), cv::Point(padding_x, line_y * 1 + padding_y), cv::FONT_HERSHEY_COMPLEX, 1,
                cv::Scalar(255, 255, 255), 2, 8, 0);
    sprintf(temp_char, "Press 'space' key to pause the mapping process");
    cv::putText(res_image, std::string(temp_char), cv::Point(padding_x, line_y * 2 + padding_y), cv::FONT_HERSHEY_COMPLEX, 1,
                cv::Scalar(255, 255, 255), 2, 8, 0);
    return res_image;
}

void ILIVE::set_initial_camera_parameter(StatesGroup &state, double *intrinsic_data, double *camera_dist_data, double *imu_camera_ext_R,
                                          double *imu_camera_ext_t, double cam_k_scale)
{
    scope_color(ANSI_COLOR_YELLOW_BOLD);
    g_cam_K << intrinsic_data[0] / cam_k_scale, intrinsic_data[1], intrinsic_data[2] / cam_k_scale, intrinsic_data[3],
        intrinsic_data[4] / cam_k_scale, intrinsic_data[5] / cam_k_scale, intrinsic_data[6], intrinsic_data[7], intrinsic_data[8];
    g_cam_dist = Eigen::Map<Eigen::Matrix<double, 5, 1>>(camera_dist_data);
    state.rot_ext_i2c = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(imu_camera_ext_R);
    state.pos_ext_i2c = Eigen::Map<Eigen::Matrix<double, 3, 1>>(imu_camera_ext_t);
    // state.pos_ext_i2c.setZero();

    // Lidar to camera parameters.
    m_mutex_lio_process.lock();

    m_inital_rot_ext_i2c = state.rot_ext_i2c;
    m_inital_pos_ext_i2c = state.pos_ext_i2c;
    state.cam_intrinsic(0) = g_cam_K(0, 0);
    state.cam_intrinsic(1) = g_cam_K(1, 1);
    state.cam_intrinsic(2) = g_cam_K(0, 2);
    state.cam_intrinsic(3) = g_cam_K(1, 2);
    set_initial_state_cov(state);
    m_mutex_lio_process.unlock();
}

void ILIVE::publish_track_img(cv::Mat &img, double frame_cost_time = -1)
{
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = ros::Time::now();               // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
    cv::Mat pub_image = img.clone();
    if (frame_cost_time > 0)
    {
        char fps_char[100];
        sprintf(fps_char, "Per-frame cost time: %.2f ms", frame_cost_time);
        // sprintf(fps_char, "%.2f ms", frame_cost_time);

        if (pub_image.cols <= 640)
        {
            cv::putText(pub_image, std::string(fps_char), cv::Point(30, 30), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255), 2, 8,
                        0); // 640 * 480
        }
        else if (pub_image.cols > 640)
        {
            cv::putText(pub_image, std::string(fps_char), cv::Point(30, 50), cv::FONT_HERSHEY_COMPLEX, 2, cv::Scalar(255, 255, 255), 2, 8,
                        0); // 1280 * 1080
        }
    }
    out_msg.image = pub_image; // Your cv::Mat
    pub_track_img.publish(out_msg);
}

void ILIVE::publish_raw_img(cv::Mat &img)
{
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = ros::Time::now();               // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
    out_msg.image = img;                                   // Your cv::Mat
    pub_raw_img.publish(out_msg);
}

int sub_image_typed = 0; // 0: TBD 1: sub_raw, 2: sub_comp
std::mutex mutex_image_callback;

std::deque<sensor_msgs::CompressedImageConstPtr> g_received_compressed_img_msg;
std::deque<sensor_msgs::ImageConstPtr> g_received_img_msg;
std::shared_ptr<std::thread> g_thr_process_image;

void ILIVE::service_process_img_buffer()
{
    while (1)
    {
        if (m_queue_image_with_pose.size() > 4)
        {
            while (m_queue_image_with_pose.size() > 4)
            {
                ros::spinOnce();
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                std::this_thread::yield();
            }
        }

        cv::Mat image_get;   // ROS格式转换为CV::MAT格式的图片
        double img_rec_time; // 图片的时间戳

        // 处理压缩后的图像
        if (sub_image_typed == 2)
        {
            
            // cout<<g_received_compressed_img_msg.size()<<endl;
            while (g_received_compressed_img_msg.size() == 0)
            {   
                ros::spinOnce();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                std::this_thread::yield();
            }

            // 从队列中取出这帧图像
            sensor_msgs::CompressedImageConstPtr msg = g_received_compressed_img_msg.front();
            try
            {
                // msg -> cv::mat
                //  将图像编码转换为BGR8
                cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                img_rec_time = msg->header.stamp.toSec();

                image_get = cv_ptr_compressed->image;
                cv_ptr_compressed->image.release();
            }
            catch (cv_bridge::Exception &e)
            {
                printf("Could not convert from '%s' to 'bgr8' !!! ", msg->format.c_str());
            }
            mutex_image_callback.lock();

            // 处理完这帧图像后，在队列中弹出
            g_received_compressed_img_msg.pop_front();
            mutex_image_callback.unlock();
        }
        // 未压缩的图像，操作同上
        else
        {

            while (g_received_img_msg.size() == 0)
            {
                ros::spinOnce();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                std::this_thread::yield();
            }
            sensor_msgs::ImageConstPtr msg = g_received_img_msg.front();
            image_get = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image.clone();
            img_rec_time = msg->header.stamp.toSec();
            mutex_image_callback.lock();
            g_received_img_msg.pop_front();
            mutex_image_callback.unlock();
        }

        // 对图像进行处理，去畸变、图像直方图均衡化等操作
        process_image(image_get, img_rec_time);
    }
}

// 压缩图像回调函数，例程
void ILIVE::image_comp_callback(const sensor_msgs::CompressedImageConstPtr &msg)
{
    if (!g_lio_state.if_state_inited){
       return;
    }
    //-----------------------

    std::unique_lock< std::mutex > lock2( mutex_image_callback );
    if ( sub_image_typed == 1 )
    {
        return; // Avoid subscribe the same image twice.
    }
    sub_image_typed = 2;
    g_received_compressed_img_msg.push_back( msg );
    if ( g_flag_if_first_rec_img )
    {
        g_flag_if_first_rec_img = 0;
        m_thread_pool_ptr->commit_task( &ILIVE::service_process_img_buffer, this );
    }
    //-------------------------------------------------------------------
    if(msg->header.stamp.toSec()==0){
        throw std::invalid_argument( "Image has no timestamp!" );
    }

    //俺加的
    // cv::Mat temp_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image.clone();
    // process_image(temp_img, msg->header.stamp.toSec());
    return;
}

// ANCHOR - image_callback
void ILIVE::image_callback(const sensor_msgs::ImageConstPtr &msg)
{
    if (!g_lio_state.if_state_inited){
       return;
    }
    std::unique_lock<std::mutex> lock(mutex_image_callback);
    if (sub_image_typed == 2)
    {
        return; // Avoid subscribe the same image twice.
    }
    sub_image_typed = 1;

    if (g_flag_if_first_rec_img)
    {
        g_flag_if_first_rec_img = 0;
        m_thread_pool_ptr->commit_task(&ILIVE::service_process_img_buffer, this);
    }


    if(msg->header.stamp.toSec()==0){
        throw std::invalid_argument( "Image has no timestamp!" );
    }
    cv::Mat temp_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image.clone();
    process_image(temp_img, msg->header.stamp.toSec());
}

double last_accept_time = 0;
int buffer_max_frame = 0;
int total_frame_count = 0;
void ILIVE::process_image(cv::Mat &temp_img, double msg_time)
{
    
    cv::Mat img_get;

    if (!g_lio_state.if_state_inited){
         return;
    }
    // 图像有效性检验
    if (temp_img.rows == 0)
    {
        cout << "Process image error, image rows =0 " << endl;
        return;
    }

    // 时间检验
    if (msg_time < last_accept_time)
    {
        cout << "Error, image time revert!!" << endl;
        return;
    }

    // 如果两帧图像间隔小于9ms，与实际20Hz的频率相差太大，跳过这帧图像。
    // m_control_image_freq 100hz
    if ((msg_time - last_accept_time) < (1.0 / m_control_image_freq) * 1.2)
    {
         return;
    }

    last_accept_time = msg_time;

    // m_camera_start_ros_tim 只在这儿用到了，初始值为-无穷
    // 仅对第一帧图像进行如下操作
    if (m_camera_start_ros_tim < 0)
    {
        m_camera_start_ros_tim = msg_time;

        m_vio_scale_factor = m_vio_image_width * m_image_downsample_ratio / temp_img.cols; // 320 * 24

        set_initial_camera_parameter(g_lio_state, m_camera_intrinsic.data(), m_camera_dist_coeffs.data(), m_camera_ext_R.data(),
                                     m_camera_ext_t.data(), m_vio_scale_factor);

        // Eigen内参和畸变系数 转换为OPENCV格式
        cv::eigen2cv(g_cam_K, intrinsic);
        cv::eigen2cv(g_cam_dist, dist_coeffs);

        initUndistortRectifyMap(intrinsic, dist_coeffs, cv::Mat(), intrinsic, cv::Size(m_vio_image_width / m_vio_scale_factor, m_vio_image_heigh / m_vio_scale_factor),
                                CV_16SC2, m_ud_map1, m_ud_map2);

        // 发布地图线程
        m_thread_pool_ptr->commit_task(&ILIVE::service_pub_rgb_maps, this);
        // VIO线程
        if (m_vio_f2f_init||m_vio_f2map_init){
         m_thread_pool_ptr->commit_task(&ILIVE::service_VIO_update, this);
        }
        // 地图路径与指针初始化
        m_mvs_recorder.init(g_cam_K, m_vio_image_width / m_vio_scale_factor, &m_map_rgb_pts);
        m_mvs_recorder.set_working_dir(m_map_output_dir);
    }

    // 未缩放
    if (m_image_downsample_ratio != 1.0)
    {
        cv::resize(temp_img, img_get, cv::Size(m_vio_image_width / m_vio_scale_factor, m_vio_image_heigh / m_vio_scale_factor));
    }
    else
    {
        // 不操作
        img_get = temp_img; // clone ?
    }

    // 使用内参初始化当前图像的位姿
    std::shared_ptr<Image_frame> img_pose = std::make_shared<Image_frame>(g_cam_K);
    if (m_if_pub_raw_img)
    {
        // 记录图像
        img_pose->m_raw_img = img_get;
    }

    //去畸变
    // img_pose->m_img 为去畸变后的图像
    cv::remap(img_get, img_pose->m_img, m_ud_map1, m_ud_map2, cv::INTER_LINEAR);


    // 保存时间
    img_pose->m_timestamp = msg_time;

    // 彩色图像转换为灰度图
    img_pose->init_cubic_interpolation();

    // 对灰度图及彩色图进行直方图均衡化，直方图均衡化后的图像的清晰度、对比度、图像质感有提高
    img_pose->image_equalize();

    m_camera_data_mutex.lock();

    // 把经过去畸变和直方图均值化后的图片丢到队列中，等待service_VIO_update线程来处理
    m_queue_image_with_pose.push_back(img_pose);
    m_camera_data_mutex.unlock();

    total_frame_count++;

    // 队列中最大积累了多少张图片没及时处理(应该是调试用的)
    if (m_queue_image_with_pose.size() > buffer_max_frame)
    {
        buffer_max_frame = m_queue_image_with_pose.size();
    }

    // cout << "Image queue size = " << m_queue_image_with_pose.size() << endl;
    // cout << "Max frame buffer" << buffer_max_frame <<endl;
}

void ILIVE::load_parameters(StateParameter &params)
{

    std::vector<double> camera_intrinsic_data, camera_dist_coeffs_data, camera_ext_R_data, camera_ext_t_data;
    std::vector<double> t_lid_in_imu, R_lid_in_imu;
    m_ros_node_handle.getParam("ilive_vio/image_width", m_vio_image_width);
    m_ros_node_handle.getParam("ilive_vio/image_height", m_vio_image_heigh);
    m_ros_node_handle.getParam("ilive_vio/camera_intrinsic", camera_intrinsic_data);
    m_ros_node_handle.getParam("ilive_vio/camera_dist_coeffs", camera_dist_coeffs_data);
    m_ros_node_handle.getParam("ilive_vio/camera_ext_R", camera_ext_R_data);
    m_ros_node_handle.getParam("ilive_vio/camera_ext_t", camera_ext_t_data);
    m_ros_node_handle.getParam("ilive_lio/ext_t_lid_in_imu", t_lid_in_imu);
    m_ros_node_handle.getParam("ilive_lio/ext_R_lid_in_imu", R_lid_in_imu);
    m_ros_node_handle.getParam("ilive_IMU/max_IMU_calibration_num", params.max_IMU_calibration_num);
    m_ros_node_handle.getParam("ilive_IMU/COV_ACC_NOISE_DIAG", params.COV_ACC_NOISE_DIAG);
    m_ros_node_handle.getParam("ilive_IMU/COV_OMEGA_NOISE_DIAG", params.COV_OMEGA_NOISE_DIAG);
    m_ros_node_handle.getParam("ilive_IMU/COV_BIAS_ACC_NOISE_DIAG", params.COV_BIAS_ACC_NOISE_DIAG);
    m_ros_node_handle.getParam("ilive_IMU/COV_BIAS_GYRO_NOISE_DIAG", params.COV_BIAS_GYRO_NOISE_DIAG );
    std::string default_method="EKF";
    get_ros_parameter(m_ros_node_handle, "ilive_common/method", params.m_method, default_method);

    if ((R_lid_in_imu.size() != 9) || (t_lid_in_imu.size() != 3))
    {

        cout << ANSI_COLOR_RED_BOLD << "Load LIO parameter fail!!!, please check!!!" << endl;
        std::this_thread::sleep_for(std::chrono::seconds(3000000));
    }

    ext_R_lid_in_imu = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(R_lid_in_imu.data());
    ext_t_lid_in_imu = Eigen::Map<Eigen::Matrix<double, 3, 1>>(t_lid_in_imu.data());
    params.Lidar_IN_IMU_Rotation=ext_R_lid_in_imu;
    params.Lidar_IN_IMU_Position=ext_t_lid_in_imu;

    CV_Assert((m_vio_image_width != 0 && m_vio_image_heigh != 0));
    // cout<<camera_intrinsic_data.size()<<endl;



    if ((camera_intrinsic_data.size() != 9) || (camera_dist_coeffs_data.size() != 5) || (camera_ext_R_data.size() != 9) ||
        (camera_ext_t_data.size() != 3))
    {
       
        cout << ANSI_COLOR_RED_BOLD << "Load VIO parameter fail!!!, please check!!!" << endl;
        printf("Load camera data size = %d, %d, %d, %d\n", (int)camera_intrinsic_data.size(), camera_dist_coeffs_data.size(),
               camera_ext_R_data.size(), camera_ext_t_data.size());
        cout << ANSI_COLOR_RESET << endl;
        std::this_thread::sleep_for(std::chrono::seconds(3000000));
    }

    m_camera_intrinsic = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(camera_intrinsic_data.data());
    m_camera_dist_coeffs = Eigen::Map<Eigen::Matrix<double, 5, 1>>(camera_dist_coeffs_data.data());
    m_camera_ext_R = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(camera_ext_R_data.data());
    m_camera_ext_t = Eigen::Map<Eigen::Matrix<double, 3, 1>>(camera_ext_t_data.data());

    params.Camera_IN_IMU_Rotation=m_camera_ext_R;
    params.Camera_IN_IMU_Position=m_camera_ext_t;

    cout << "[Ros_parameter]: ilive_vio/Camera Intrinsic: " << endl;
    cout << m_camera_intrinsic << endl;
    cout << "[Ros_parameter]: ilive_vio/Camera distcoeff: " << m_camera_dist_coeffs.transpose() << endl;
    cout << "[Ros_parameter]: ilive_vio/Camera extrinsic R: " << endl;
    cout << m_camera_ext_R << endl;
    cout << "[Ros_parameter]: ilive_vio/Camera extrinsic T: " << m_camera_ext_t.transpose() << endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    //及的删掉
    cout << "[Ros_parameter]: ilive_IMU/COV_ACC_NOISE_DIAG: " <<  params.COV_ACC_NOISE_DIAG << endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
        cout << "[Ros_parameter]: ilive_IMU/COV_ACC_NOISE_DIAG: " << params.COV_BIAS_ACC_NOISE_DIAG << endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));






}

void ILIVE::set_image_pose(std::shared_ptr<Image_frame> &image_pose, const StatesGroup &state)
{
    // 将IMU的位姿取出，根据外参转换为相机的位姿
    mat_3_3 rot_mat = state.rot_end;
    vec_3 t_vec = state.pos_end;
    vec_3 pose_t = rot_mat * state.pos_ext_i2c + t_vec;
    mat_3_3 R_w2c = rot_mat * state.rot_ext_i2c;

    // 设置世界系到相机系下的位姿
    image_pose->set_pose(eigen_q(R_w2c), pose_t);
    image_pose->fx = state.cam_intrinsic(0);
    image_pose->fy = state.cam_intrinsic(1);
    image_pose->cx = state.cam_intrinsic(2);
    image_pose->cy = state.cam_intrinsic(3);

    image_pose->m_cam_K << image_pose->fx, 0, image_pose->cx, 0, image_pose->fy, image_pose->cy, 0, 0, 1;

    // 设置输出的颜色 调试用
    scope_color(ANSI_COLOR_CYAN_BOLD);
}

void ILIVE::publish_camera_odom(std::shared_ptr<Image_frame> &image, double msg_time)
{
    eigen_q odom_q = image->m_pose_w2c_q;
    vec_3 odom_t = image->m_pose_w2c_t;
    nav_msgs::Odometry camera_odom;
    camera_odom.header.frame_id = "world";
    camera_odom.child_frame_id = "/aft_mapped";
    camera_odom.header.stamp = ros::Time::now(); // ros::Time().fromSec(last_timestamp_lidar);
    camera_odom.pose.pose.orientation.x = odom_q.x();
    camera_odom.pose.pose.orientation.y = odom_q.y();
    camera_odom.pose.pose.orientation.z = odom_q.z();
    camera_odom.pose.pose.orientation.w = odom_q.w();
    camera_odom.pose.pose.position.x = odom_t(0);
    camera_odom.pose.pose.position.y = odom_t(1);
    camera_odom.pose.pose.position.z = odom_t(2);
    pub_odom_cam.publish(camera_odom);

    geometry_msgs::PoseStamped msg_pose;
    msg_pose.header.stamp = ros::Time().fromSec(msg_time);///////??????????????????????
    msg_pose.header.frame_id = "world";
    msg_pose.pose.orientation.x = odom_q.x();
    msg_pose.pose.orientation.y = odom_q.y();
    msg_pose.pose.orientation.z = odom_q.z();
    msg_pose.pose.orientation.w = odom_q.w();
    msg_pose.pose.position.x = odom_t(0);
    msg_pose.pose.position.y = odom_t(1);
    msg_pose.pose.position.z = odom_t(2);
    camera_path.header.frame_id = "world";
    camera_path.poses.push_back(msg_pose);
    pub_path_cam.publish(camera_path);
}

void ILIVE::publish_track_pts(Rgbmap_tracker &tracker)
{
    pcl::PointXYZRGB temp_point;
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud_for_pub;

    for (auto it : tracker.m_map_rgb_pts_in_current_frame_pos)
    {
        vec_3 pt = ((RGB_pts *)it.first)->get_pos();
        cv::Scalar color = ((RGB_pts *)it.first)->m_dbg_color;
        temp_point.x = pt(0);
        temp_point.y = pt(1);
        temp_point.z = pt(2);
        temp_point.r = color(2);
        temp_point.g = color(1);
        temp_point.b = color(0);
        pointcloud_for_pub.points.push_back(temp_point);
    }
    sensor_msgs::PointCloud2 ros_pc_msg;
    pcl::toROSMsg(pointcloud_for_pub, ros_pc_msg);
    ros_pc_msg.header.stamp = ros::Time::now(); //.fromSec(last_timestamp_lidar);
    ros_pc_msg.header.frame_id = "world";       // world; camera_init
    m_pub_visual_tracked_3d_pts.publish(ros_pc_msg);
}

// ANCHOR - VIO preintegration
bool ILIVE::vio_preintegration(StatesGroup &state_in, StatesGroup &state_out, double current_frame_time)
{
    state_out = state_in;

    if (current_frame_time <= state_in.last_update_time)
    {
        return false;
    }
    mtx_buffer.lock();

    // 取出imu_buffer_vio中的信息
    std::deque<sensor_msgs::Imu::ConstPtr> vio_imu_queue;
    for (auto it = imu_buffer_vio.begin(); it != imu_buffer_vio.end(); it++)
    {
        vio_imu_queue.push_back(*it);

        // 预测当前帧的状态，肯定不能用未来的数据
        if ((*it)->header.stamp.toSec() > current_frame_time)
        {
            break;
        }
    }

    //
    while (!imu_buffer_vio.empty())
    {
        double imu_time = imu_buffer_vio.front()->header.stamp.toSec();

        if (imu_time < current_frame_time - 0.2)//
        {
            imu_buffer_vio.pop_front();
        }
        else
        {
            break;
        }
    }

    // cout << "Current VIO_imu buffer size = " << imu_buffer_vio.size() << endl;

    // imu预积分 更新状态变量
    state_out = m_imu_process->imu_preintegration(state_out, vio_imu_queue, current_frame_time - vio_imu_queue.back()->header.stamp.toSec());
    
    
    // 输出增量 调试用
    eigen_q q_diff(state_out.rot_end.transpose() * state_in.rot_end);
    
    mtx_buffer.unlock();
    state_out.last_update_time = current_frame_time;
    return true;
}

void ILIVE::service_pub_rgb_maps()
{
    int last_publish_map_idx = -3e8;
    int sleep_time_aft_pub = 10;
    int number_of_pts_per_topic = 1000;
    if (number_of_pts_per_topic < 0)
    {
        return;
    }
    while (1)
    {
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        pcl::PointCloud<pcl::PointXYZRGB> pc_rgb;
        sensor_msgs::PointCloud2 ros_pc_msg;
        int pts_size = m_map_rgb_pts.m_rgb_pts_vec.size();
        pc_rgb.resize(number_of_pts_per_topic);
        // for (int i = pts_size - 1; i > 0; i--)
        int pub_idx_size = 0;
        int cur_topic_idx = 0;
        if (last_publish_map_idx == m_map_rgb_pts.m_last_updated_frame_idx)
        {
            continue;
        }
        last_publish_map_idx = m_map_rgb_pts.m_last_updated_frame_idx;
        for (int i = 0; i < pts_size; i++)
        {
            if (m_map_rgb_pts.m_rgb_pts_vec[i]->m_N_rgb < 1)
            {
                continue;
            }
            pc_rgb.points[pub_idx_size].x = m_map_rgb_pts.m_rgb_pts_vec[i]->m_pos[0];
            pc_rgb.points[pub_idx_size].y = m_map_rgb_pts.m_rgb_pts_vec[i]->m_pos[1];
            pc_rgb.points[pub_idx_size].z = m_map_rgb_pts.m_rgb_pts_vec[i]->m_pos[2];
            pc_rgb.points[pub_idx_size].r = m_map_rgb_pts.m_rgb_pts_vec[i]->m_rgb[2];
            pc_rgb.points[pub_idx_size].g = m_map_rgb_pts.m_rgb_pts_vec[i]->m_rgb[1];
            pc_rgb.points[pub_idx_size].b = m_map_rgb_pts.m_rgb_pts_vec[i]->m_rgb[0];
            // pc_rgb.points[i].intensity = m_map_rgb_pts.m_rgb_pts_vec[i]->m_obs_dis;
            pub_idx_size++;
            if (pub_idx_size == number_of_pts_per_topic)
            {
                pub_idx_size = 0;
                pcl::toROSMsg(pc_rgb, ros_pc_msg);
                ros_pc_msg.header.frame_id = "world";
                ros_pc_msg.header.stamp = ros::Time::now();
                if (m_pub_rgb_render_pointcloud_ptr_vec[cur_topic_idx] == nullptr)
                {
                    m_pub_rgb_render_pointcloud_ptr_vec[cur_topic_idx] =
                        std::make_shared<ros::Publisher>(m_ros_node_handle.advertise<sensor_msgs::PointCloud2>(
                            std::string("/RGB_map_").append(std::to_string(cur_topic_idx)), 100));
                }
                m_pub_rgb_render_pointcloud_ptr_vec[cur_topic_idx]->publish(ros_pc_msg);
                std::this_thread::sleep_for(std::chrono::microseconds(sleep_time_aft_pub));
                ros::spinOnce();
                cur_topic_idx++;
            }
        }

        pc_rgb.resize(pub_idx_size);
        pcl::toROSMsg(pc_rgb, ros_pc_msg);
        ros_pc_msg.header.frame_id = "world";
        ros_pc_msg.header.stamp = ros::Time::now();
        if (m_pub_rgb_render_pointcloud_ptr_vec[cur_topic_idx] == nullptr)
        {
            m_pub_rgb_render_pointcloud_ptr_vec[cur_topic_idx] =
                std::make_shared<ros::Publisher>(m_ros_node_handle.advertise<sensor_msgs::PointCloud2>(
                    std::string("/RGB_map_").append(std::to_string(cur_topic_idx)), 100));
        }
        std::this_thread::sleep_for(std::chrono::microseconds(sleep_time_aft_pub));
        ros::spinOnce();
        m_pub_rgb_render_pointcloud_ptr_vec[cur_topic_idx]->publish(ros_pc_msg);
        cur_topic_idx++;
        if (cur_topic_idx >= 45) // Maximum pointcloud topics = 45.
        {
            number_of_pts_per_topic *= 1.5;
            sleep_time_aft_pub *= 1.5;
        }
    }
}

void ILIVE::publish_render_pts(ros::Publisher &pts_pub, Global_map &m_map_rgb_pts)
{
    pcl::PointCloud<pcl::PointXYZRGB> pc_rgb;
    sensor_msgs::PointCloud2 ros_pc_msg;
    pc_rgb.reserve(1e7);
    m_map_rgb_pts.m_mutex_m_box_recent_hitted->lock();
    std::unordered_set<std::shared_ptr<RGB_Voxel>> boxes_recent_hitted = m_map_rgb_pts.m_voxels_recent_visited;
    m_map_rgb_pts.m_mutex_m_box_recent_hitted->unlock();

    for (Voxel_set_iterator it = boxes_recent_hitted.begin(); it != boxes_recent_hitted.end(); it++)
    {
        for (int pt_idx = 0; pt_idx < (*it)->m_pts_in_grid.size(); pt_idx++)
        {
            pcl::PointXYZRGB pt;
            std::shared_ptr<RGB_pts> rgb_pt = (*it)->m_pts_in_grid[pt_idx];
            pt.x = rgb_pt->m_pos[0];
            pt.y = rgb_pt->m_pos[1];
            pt.z = rgb_pt->m_pos[2];
            pt.r = rgb_pt->m_rgb[2];
            pt.g = rgb_pt->m_rgb[1];
            pt.b = rgb_pt->m_rgb[0];
            if (rgb_pt->m_N_rgb > m_pub_pt_minimum_views)
            {
                pc_rgb.points.push_back(pt);
            }
        }
    }
    pcl::toROSMsg(pc_rgb, ros_pc_msg);
    ros_pc_msg.header.frame_id = "world";       // world; camera_init
    ros_pc_msg.header.stamp = ros::Time::now(); //.fromSec(last_timestamp_lidar);
    pts_pub.publish(ros_pc_msg);
}

char ILIVE::cv_keyboard_callback()
{
    char c = cv_wait_key(1);
    // return c;
    if (c == 's' || c == 'S')
    {
        scope_color(ANSI_COLOR_GREEN_BOLD);
        cout << "I capture the keyboard input!!!" << endl;
        m_mvs_recorder.export_to_mvs(m_map_rgb_pts);
        // m_map_rgb_pts.save_and_display_pointcloud( m_map_output_dir, std::string("/rgb_pt"), std::max(m_pub_pt_minimum_views, 5) );
        m_map_rgb_pts.save_and_display_pointcloud(m_map_output_dir, std::string("/rgb_pt"), m_pub_pt_minimum_views);
    }
    return c;
}
