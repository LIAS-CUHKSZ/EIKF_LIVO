#include "ilive.hpp"




void ILIVE::imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
    double timestamp = msg->header.stamp.toSec(); // 获取IMU的当前时间
    g_camera_lidar_queue.imu_in(timestamp);       // 设置IMU的当前时刻时间
    mtx_buffer.lock();                            // imu上锁

    if (timestamp < last_timestamp_imu)           // last_timestamp_imu初始为-1
    {
        ROS_ERROR("imu loop back, clear buffer");
        imu_buffer_lio.clear();
        imu_buffer_vio.clear();
        flg_reset = true;
    }

    last_timestamp_imu = timestamp;


    if ( g_camera_lidar_queue.m_if_acc_mul_G )
    {
        msg->linear_acceleration.x *= G_m_s2;
        msg->linear_acceleration.y *= G_m_s2;
        msg->linear_acceleration.z *= G_m_s2;
    }

    imu_buffer_lio.push_back(msg);
    imu_buffer_vio.push_back(msg);

    mtx_buffer.unlock(); // imu解锁
    sig_buffer.notify_all();
}

    

void printf_field_name(sensor_msgs::PointCloud2::ConstPtr &msg)
{
    cout << "Input pointcloud field names: [" << msg->fields.size() << "]: ";
    for (size_t i = 0; i < msg->fields.size(); i++)
    {
        cout << msg->fields[i].name << ", ";
    }
    cout << endl;
}

bool ILIVE::get_pointcloud_data_from_ros_message(sensor_msgs::PointCloud2::ConstPtr &msg, pcl::PointCloud<pcl::PointXYZINormal> &pcl_pc)
{

    // printf("Frame [%d] %.3f ", g_LiDAR_frame_index,  msg->header.stamp.toSec() - g_camera_lidar_queue.m_first_imu_time);
    pcl::PointCloud<pcl::PointXYZI> res_pc;
    pcl::PointCloud<pcl::PointXYZ> pc_tmp;

    scope_color(ANSI_COLOR_YELLOW_BOLD);
    // printf_field_name(msg);
    if (msg->fields.size() < 3)
    {
        cout << "Get pointcloud data from ros messages fail!!!" << endl;
        scope_color(ANSI_COLOR_RED_BOLD);
        printf_field_name(msg);
        return false;
    }
    else
    {

        if ((msg->fields.size() == 8) && (msg->fields[3].name == "intensity"))  // Input message type is pcl::PointXYZINormal
        {
            pcl::fromROSMsg(*msg, pcl_pc);
            return true;
        }

        else if ((msg->fields.size() == 5) && (msg->fields[3].name == "intensity"))  // Input message type is pcl::PointXYZINormal
        {
            pcl::fromROSMsg(*msg, pc_tmp);
            pcl::copyPointCloud(pc_tmp,pcl_pc);   
            return true;
        }
        else if ((msg->fields.size() == 4) && (msg->fields[3].name == "intensity"))
        {
            pcl::fromROSMsg(*msg, pc_tmp);
            pcl::copyPointCloud(pc_tmp,pcl_pc);   
            return true;

        }

        else if ((msg->fields.size() == 5) && (msg->fields[3].name == "time"))
        {
            pcl::fromROSMsg(*msg, pc_tmp);
            pcl::copyPointCloud(pc_tmp,pcl_pc);   
            return true;

        }


        else if ((msg->fields.size() == 9) && (msg->fields[3].name == "intensity"))  // Input message type is pcl::PointXYZINormal
        {
            pcl::fromROSMsg(*msg, pc_tmp);
            pcl::copyPointCloud(pc_tmp,pcl_pc);   
            return true;
        }

        else if ((msg->fields.size() == 6) && (msg->fields[3].name == "intensity"))  // Input message type is pcl::PointXYZINormal
        {
            pcl::fromROSMsg(*msg, pc_tmp);
            pcl::copyPointCloud(pc_tmp,pcl_pc);   
            return true;
        }       

        else if ((msg->fields.size() == 4) && (msg->fields[3].name == "rgb"))
        {
            double maximum_range = 5;
            get_ros_parameter<double>(m_ros_node_handle, "iros_range", maximum_range, 5);
            pcl::PointCloud<pcl::PointXYZRGB> pcl_rgb_pc;
            pcl::fromROSMsg(*msg, pcl_rgb_pc);
            double lidar_point_time = msg->header.stamp.toSec();
            int pt_count = 0;
            pcl_pc.resize(pcl_rgb_pc.points.size());
            for (int i = 0; i < pcl_rgb_pc.size(); i++)
            {
                pcl::PointXYZINormal temp_pt;
                temp_pt.x = pcl_rgb_pc.points[i].x;
                temp_pt.y = pcl_rgb_pc.points[i].y;
                temp_pt.z = pcl_rgb_pc.points[i].z;
                double frame_dis = sqrt(temp_pt.x * temp_pt.x + temp_pt.y * temp_pt.y + temp_pt.z * temp_pt.z);
                if (frame_dis > maximum_range)
                {
                    continue;
                }
                temp_pt.intensity = (pcl_rgb_pc.points[i].r + pcl_rgb_pc.points[i].g + pcl_rgb_pc.points[i].b) / 3.0;
                temp_pt.curvature = 0;
                pcl_pc.points[pt_count] = temp_pt;
                pt_count++;
            }
 
            pcl_pc.points.resize(pt_count);
            return true;
        }

        
        else // TODO, can add by yourself
        {
            cout << "Get pointcloud data from ros messages fail!!! ";
            scope_color(ANSI_COLOR_RED_BOLD);
            printf_field_name(msg);
            return false;
        }
       
    }
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool ILIVE::sync_packages(MeasureGroup &meas)
{
    //(1)判断lidar和imu的数据buffer是否为空
    if (lidar_buffer.empty() || imu_buffer_lio.empty())
    {
        return false;
    }

    /*** push lidar frame 取出lidar数据***/
    if (!lidar_pushed) // lidar_pushed初始false
    {
        meas.lidar.reset(new PointCloudXYZINormal());
        // 从lidar_buffer中获取点云数据(出队列: 取队列队首数据)
        if (get_pointcloud_data_from_ros_message(lidar_buffer.front(), *(meas.lidar)) == false)
        {
            return false;
        }
        // pcl::fromROSMsg(*(lidar_buffer.front()), *(meas.lidar));
        meas.lidar_beg_time = lidar_buffer.front()->header.stamp.toSec(); // 当前lidar帧的开始时间
        lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
        meas.lidar_end_time = lidar_end_time; // 当前lidar帧的结束时间
        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer 取出imu数据***/
    double imu_time = imu_buffer_lio.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer_lio.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = imu_buffer_lio.front()->header.stamp.toSec();
        if (imu_time > lidar_end_time + 0.00001)
            break;
        meas.imu.push_back(imu_buffer_lio.front());
        imu_buffer_lio.pop_front();
    }

    lidar_buffer.pop_front();
    lidar_pushed = false;

    return true;
}

// project lidar frame to world - 将参数1-pi转换至世界坐标系下,并赋值给参数2-po
void ILIVE::pointBodyToWorld(PointType const *const pi, PointType *const po)
{
    Eigen::Vector3d p_body(pi->x, pi->y, pi->z);
    Eigen::Vector3d p_global(g_lio_state.rot_end * (ext_R_lid_in_imu* p_body + ext_t_lid_in_imu) + g_lio_state.pos_end);//////

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void ILIVE::RGBpointBodyToWorld(PointType const *const pi, pcl::PointXYZI *const po)
{
    Eigen::Vector3d p_body(pi->x, pi->y, pi->z);
    Eigen::Vector3d p_global(g_lio_state.rot_end * (ext_R_lid_in_imu * p_body + ext_t_lid_in_imu) + g_lio_state.pos_end);/////
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;

    float intensity = pi->intensity;
    intensity = intensity - std::floor(intensity);

    int reflection_map = intensity * 10000;
}


void ILIVE::feat_points_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg_in)
{
    mtx_buffer.lock(); // imu上锁
    sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2(*msg_in));
    msg->header.stamp = ros::Time(msg_in->header.stamp.toSec() - m_lidar_imu_time_delay); 
    if (g_camera_lidar_queue.lidar_in(msg_in->header.stamp.toSec() + 0.1) == 0)
    {
        return;
    }
    // std::cout<<"got feature"<<std::endl;
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    { // last_timestamp_lidar初始=-1
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    lidar_buffer.push_back(msg);
    // cout<< "Lidar_buffer size"<< lidar_buffer.size()<<endl;
    last_timestamp_lidar = msg->header.stamp.toSec();
    mtx_buffer.unlock();     // imu解锁
    sig_buffer.notify_all(); // 唤醒LIO处理线程,进行Lidar数据处理
}



void ILIVE::wait_render_thread_finish()
{
    if (m_render_thread != nullptr)
    {
        m_render_thread->get(); // wait render thread to finish.
    }
}
