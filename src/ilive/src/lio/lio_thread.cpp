#include "ilive.hpp"
#define debug_LIO (1)
/**
 * @note LIO子系统处理线程
 */

boost::array<double, 36> matrixToBoostArray(const Eigen::Matrix<double, 6, 6>& matrix) {
    boost::array<double, 36> result;
    int index = 0;
    for (int i = 0; i < matrix.rows(); ++i) {
        for (int j = 0; j < matrix.cols(); ++j) {
            result[index++] = matrix(i, j);
        }
    }
    return result;
}




int ILIVE::service_LIO_update()
{
    //--------Variable definition and initialization-----------------------------------
    PointCloudXYZINormal::Ptr feats_undistort(new PointCloudXYZINormal()); // 去除畸变后的点云
    PointCloudXYZINormal::Ptr feats_down(new PointCloudXYZINormal());      // 保存下采样后的点云
    PointCloudXYZINormal::Ptr laserCloudOri(new PointCloudXYZINormal());   // 存放找到了最近平面的M个点的容器

    nav_msgs::Path path;                  // Lidar的路径 : 主要有两个成员变量: header和pose
    path.header.stamp = ros::Time::now(); // header的时间
    path.header.frame_id = "/world";      // header的id

    /*** variables initialize ***/
    FOV_DEG = fov_deg + 10;                                         // fov_deg=360
    HALF_FOV_COS = std::cos((fov_deg + 10.0) * 0.5 * PI_M / 180.0); // cos(185)


    std::shared_ptr<ImuProcess> p_imu(new ImuProcess(g_parames)); // 定义用于前向/后向传播的IMU处理器
    m_imu_process = p_imu;
    //------------------------------------------------------------------------------------------------------
    ros::Rate rate(5000);
    bool status = ros::ok();
    g_camera_lidar_queue.m_liar_frame_buf = &lidar_buffer; // 取出lidar数据
    set_initial_state_cov(g_lio_state);                    // 初始化g_lio_state的状态协方差矩阵

    while (ros::ok()) // 运行LIO线程循环
    {
        if (flg_exit)
            break;
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        while (g_camera_lidar_queue.if_lidar_can_process() == false)
        {
            ros::spinOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(THREAD_SLEEP_TIM));
            std::this_thread::yield();
        }

        std::unique_lock<std::mutex> lock(m_mutex_lio_process);
        // printf_line;
        Common_tools::Timer tim;

        if (sync_packages(Measures) == 0)
        {
            continue; // 提取数据失败
        }
        // process IMU DATA and align the IMU data with the LiDAR data
        p_imu->Process(Measures, g_lio_state, feats_undistort);


/*** Compute the euler angle 这里的euler_cur就是当前的lidar里程计的旋转信息,后面需要用kalman迭代更新,最后发布到ros中***/
        Eigen::Matrix<double, 3, 3> R_lidar = g_lio_state.rot_end * ext_R_lid_in_imu;
        Eigen::Vector3d euler_cur = RotMtoEuler(g_lio_state.rot_end); //publish imu rot

        if (!g_lio_state.if_state_inited){
            continue;
        }

        int lidar_can_update = 1;
        g_lidar_star_tim = frame_first_pt_time;
        if (flg_reset) // 判断重置标识
        {
            ROS_WARN("reset when rosbag play back");
            p_imu->Reset(); // 重置前向/后向传播用的处理器 : 重置处理时用到的期望和方差等变量
            flg_reset = false;
            continue;
        }
        g_LiDAR_frame_index++; // lidar帧++
        tim.tic("Preprocess"); // time_current : 获取当前时间
        double t0, t1, t2, t3, t4, t5;
        // 重置处理时用于记录各块处理时间的变量
        kdtree_search_time = 0;
        t0 = omp_get_wtime();

    
        g_camera_lidar_queue.g_noise_cov_acc = p_imu->cov_acc;  // 获取加速度误差状态传递的协方差
        g_camera_lidar_queue.g_noise_cov_gyro = p_imu->cov_gyr; // 获取角速度误差状态传递的协方差

        // 输出lio上一帧更新的时间 : 上一帧更新记录时间 - lidar开始时间
        if (feats_undistort->empty() || (feats_undistort == NULL)) // 没有成功去除点云运动畸变
        {
            frame_first_pt_time = Measures.lidar_beg_time;
            std::cout << "not ready for odometry" << std::endl;
            continue;
        }

        if ((Measures.lidar_beg_time - frame_first_pt_time) < INIT_TIME) // INIT_TIME=0
        {
            flg_EKF_inited = false;
            std::cout << "||||||||||Initiallizing LiDAR||||||||||" << std::endl;
        }
        else // 时间满足关系,开始EKF过程
        {
            flg_EKF_inited = true;
        }


#ifdef DEBUG_PRINT // 默认注释了DEBUG_PRINT的定义
        std::cout << "current lidar time " << Measures.lidar_beg_time << " "
                  << "first lidar time " << frame_first_pt_time << std::endl;
        // 打印预积分后的结果(最后时刻IMU的状态) : 旋转向量(1rad=57.3度), 位置向量, 速度向量, 角速度bias向量, 加速度bias量
        std::cout << "pre-integrated states: " << euler_cur.transpose() * 57.3 << " " << g_lio_state.pos_end.transpose() << " "
                  << g_lio_state.vel_end.transpose() << " " << g_lio_state.bias_g.transpose() << " " << g_lio_state.bias_a.transpose()
                  << std::endl;
#endif


        downSizeFilterSurf.setInputCloud(feats_undistort); // 构建三维体素栅格
        downSizeFilterSurf.filter(*feats_down);            // 下采样滤波

        if ((feats_down->points.size() > 1) && (ikdtree.Root_Node == nullptr))
        {
            // std::vector<PointType> points_init = feats_down->points;
            ikdtree.set_downsample_param(filter_size_map_min); // filter_size_map_min默认=0.4
            ikdtree.Build(feats_down->points);                 // 构造idk树
            flg_map_initialized = true;
            continue; // 进入下一次循环
        }

        if (ikdtree.Root_Node == nullptr) // 构造ikd树失败
        {
            flg_map_initialized = false;
            std::cout << "~~~~~~~ Initialize Map iKD-Tree Failed! ~~~~~~~" << std::endl;
            continue;
        }
        int featsFromMapNum = ikdtree.size();            // ikd树的节点数
        int feats_down_size = feats_down->points.size(); // 下采样过滤后的点数

        PointCloudXYZINormal::Ptr feats_down_updated(new PointCloudXYZINormal(*feats_down));

        if (featsFromMapNum >= 5) // ***重点*** : 正式开始ICP和迭代Kalman : ikd树上至少有5个点才进行操作
        {
            t1 = omp_get_wtime();

            /**
             * @note (2-7-1) : 在ros上发布特征点云数据 - 默认不发布
             */
            if (m_if_publish_feature_map)
            {
                PointVector().swap(ikdtree.PCL_Storage);
                // flatten会将需要删除的点放入Points_deleted或Multithread_Points_deleted中
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;

                sensor_msgs::PointCloud2 laserCloudMap;
                pcl::toROSMsg(*featsFromMap, laserCloudMap);   // 将点云数据格式转换为发布的消息格式
                laserCloudMap.header.stamp = ros::Time::now(); // ros::Time().fromSec(last_timestamp_lidar);
                // laserCloudMap.header.stamp.fromSec(Measures.lidar_end_time); // ros::Time().fromSec(last_timestamp_lidar);
                laserCloudMap.header.frame_id = "world";
                pubLaserCloudMap.publish(laserCloudMap);
            }

            t2 = omp_get_wtime();

            FeatsDown feats_down_struct;
            feats_down_struct.original = feats_down;
            feats_down_struct.updated = feats_down_updated;
            feats_down_struct.size = feats_down_size;

            if (m_lio_init)
            {
                lio_update(tim, laserCloudOri, feats_down_struct, euler_cur, g_lio_state);
            }

            t3 = omp_get_wtime();

            PointVector points_history;                     // 将ikd树中需要移除的点放入points_history中
            ikdtree.acquire_removed_points(points_history); // 从Points_deleted和Multithread_Points_deleted获取点
            // 1> : 更新维持的固定大小的map立方体 (参考FAST-LIO2:V.A地图管理)


            // 2-1> : 将Kalman更新后的新lidar帧特征点先转世界坐标系
            for (int i = 0; i < feats_down_size; i++)
            {
                /* transform to world frame */
                pointBodyToWorld(&(feats_down->points[i]), &(feats_down_updated->points[i]));
            }
            t4 = omp_get_wtime();
            // 2-2> : 将特征点加入世界坐标中
            ikdtree.Add_Points(feats_down_updated->points, true); // 存入ikd树中

            kdtree_incremental_time = omp_get_wtime() - t4 + readd_time + readd_box_time + delete_box_time;
            t5 = omp_get_wtime();
        } // (2-7) ICP迭代+Kalman更新完成 */

        /**ros::ok()
         * @note (2-8) : Publish current frame points in world coordinates:
         *              发布当前帧的点云数据
         */
        // std::cout << "publishing current frame points in world coordinates" << endl;
        // std::cout<< g_lio_state.rot_end<< endl;9
        laserCloudFullRes2->clear();
        *laserCloudFullRes2 = dense_map_en ? (*feats_undistort) : (*feats_down); // 去畸变or下采样点
        int laserCloudFullResNum = laserCloudFullRes2->points.size();            // 发布点数量

        pcl::PointXYZI temp_point;
        laserCloudFullResColor->clear();
        {
            for (int i = 0; i < laserCloudFullResNum; i++)
            { // 将laserCloudFullRes2的点转到世界坐标系下,再存入laserCloudFullResColor
                RGBpointBodyToWorld(&laserCloudFullRes2->points[i], &temp_point);
                laserCloudFullResColor->push_back(temp_point);
            }
            sensor_msgs::PointCloud2 laserCloudFullRes3; // 将laserCloudFullResColor转为发布形式
            pcl::toROSMsg(*laserCloudFullResColor, laserCloudFullRes3);
            laserCloudFullRes3.header.stamp.fromSec(Measures.lidar_end_time);
            //laserCloudFullRes3.header.stamp.fromSec(Measures.lidar_end_time);
            laserCloudFullRes3.header.frame_id = "world"; // world; camera_init
            pubLaserCloudFullRes.publish(laserCloudFullRes3);
        }

        if (g_camera_lidar_queue.m_if_have_camera_data || (g_LiDAR_frame_index < 500))//g_LIDAR_frame_index太小的话会在ikd tree 报错ERROR!!!!
        {
            static std::vector<double> stastic_cost_time;
            Common_tools::Timer tim;
            // tim.tic();
            // ANCHOR - RGB maps update
            wait_render_thread_finish();
            if (m_if_record_mvs) //用来标记是否记录offline map
            {
                std::vector<std::shared_ptr<RGB_pts>> pts_last_hitted;
                pts_last_hitted.reserve(1e6); //容器

                m_number_of_new_visited_voxel = m_map_rgb_pts.append_points_to_global_map(  //
                    *laserCloudFullResColor, Measures.lidar_end_time - g_camera_lidar_queue.m_first_imu_time, &pts_last_hitted,
                    m_append_global_map_point_step);
                m_map_rgb_pts.m_mutex_pts_last_visited->lock();
                m_map_rgb_pts.m_pts_last_hitted = pts_last_hitted;
                m_map_rgb_pts.m_mutex_pts_last_visited->unlock();
            }
            else
            {
                m_number_of_new_visited_voxel = m_map_rgb_pts.append_points_to_global_map(
                    *laserCloudFullResColor, Measures.lidar_end_time - g_camera_lidar_queue.m_first_imu_time, nullptr,
                    m_append_global_map_point_step);
            }
            stastic_cost_time.push_back(tim.toc(" ", 0));
        }

        if (0) // Uncomment this code scope to enable the publish of effective points.
        {
            /******* Publish effective points *******/
            laserCloudFullResColor->clear();
            pcl::PointXYZI temp_point;
            for (int i = 0; i < laserCloudSelNum; i++)
            {
                RGBpointBodyToWorld(&laserCloudOri->points[i], &temp_point);
                laserCloudFullResColor->push_back(temp_point);
            }
            sensor_msgs::PointCloud2 laserCloudFullRes3;
            pcl::toROSMsg(*laserCloudFullResColor, laserCloudFullRes3);
            laserCloudFullRes3.header.stamp = ros::Time::now(); //.fromSec(last_timestamp_lidar);
            // laserCloudFullRes3.header.stamp.fromSec(Measures.lidar_end_time); //.fromSec(last_timestamp_lidar);
            laserCloudFullRes3.header.frame_id = "world";
            pubLaserCloudEffect.publish(laserCloudFullRes3);
        }

        /**
         * @note (2-11)***** Publish Maps:  发布地图******
         */
        sensor_msgs::PointCloud2 laserCloudMap;
        pcl::toROSMsg(*featsFromMap, laserCloudMap);
        laserCloudMap.header.stamp.fromSec(Measures.lidar_end_time); // ros::Time().fromSec(last_timestamp_lidar);
        laserCloudMap.header.frame_id = "world";
        pubLaserCloudMap.publish(laserCloudMap);


        /**
         * @note (2-12)***** Publish Odometry 发布里程计*****
         */
        geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));
        odomAftMapped.header.frame_id = "world";
        odomAftMapped.child_frame_id = "/aft_mapped";
        odomAftMapped.header.stamp= ros::Time().fromSec(Measures.lidar_end_time);
        odomAftMapped.pose.pose.orientation.x = geoQuat.x;
        odomAftMapped.pose.pose.orientation.y = geoQuat.y;
        odomAftMapped.pose.pose.orientation.z = geoQuat.z;
        odomAftMapped.pose.pose.orientation.w = geoQuat.w;
        odomAftMapped.pose.pose.position.x = g_lio_state.pos_end(0);
        odomAftMapped.pose.pose.position.y = g_lio_state.pos_end(1);
        odomAftMapped.pose.pose.position.z = g_lio_state.pos_end(2);

        boost::array<double, 36> pose_cov_now;
        Eigen::Matrix<double, 6, 6> subMatrix = g_lio_state.cov.block<6, 6>(0, 0);
        pose_cov_now=matrixToBoostArray(subMatrix);

        odomAftMapped.pose.covariance = pose_cov_now;

        pubOdomAftMapped.publish(odomAftMapped);//这里发布的仅仅是前面六维度的pose的协防差

//全维度协防差的发布//-//
        //将eigen矩阵转化为ROS消息
        std_msgs::Float64MultiArray covArray;
        covArray.layout.dim.push_back(std_msgs::MultiArrayDimension());
        covArray.layout.dim.push_back(std_msgs::MultiArrayDimension());
        covArray.layout.dim[0].size = 16; // 行数
        covArray.layout.dim[1].size = 16; // 列数
        covArray.layout.dim[0].stride = 16 * 16; // 行的步长
        covArray.layout.dim[1].stride = 16; // 列的步长
        covArray.layout.dim[0].label = "rows";
        covArray.layout.dim[1].label = "cols";
        covArray.data.resize(16 * 16); // 调整数据大小以匹配矩阵大小
 
        for (int i = 0; i < 16; ++i) { //赋值
            for (int j = 0; j < 16; ++j) {
                covArray.data[i * 16 + j] = g_lio_state.cov(i, j);
            }
        }

        pubAllCov.publish(covArray);

//-//

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Quaternion q;
        transform.setOrigin(
            tf::Vector3(odomAftMapped.pose.pose.position.x, odomAftMapped.pose.pose.position.y, odomAftMapped.pose.pose.position.z));
        q.setW(odomAftMapped.pose.pose.orientation.w);
        q.setX(odomAftMapped.pose.pose.orientation.x);
        q.setY(odomAftMapped.pose.pose.orientation.y);
        q.setZ(odomAftMapped.pose.pose.orientation.z);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec( Measures.lidar_end_time ), "world", "/aft_mapped"));

        msg_body_pose.header.stamp = ros::Time::now();
       // msg_body_pose.header.frame_id = "/camera_odom_frame";
        msg_body_pose.pose.position.x = g_lio_state.pos_end(0);
        msg_body_pose.pose.position.y = g_lio_state.pos_end(1);
        msg_body_pose.pose.position.z = g_lio_state.pos_end(2);
        msg_body_pose.pose.orientation.x = geoQuat.x;
        msg_body_pose.pose.orientation.y = geoQuat.y;
        msg_body_pose.pose.orientation.z = geoQuat.z;
        msg_body_pose.pose.orientation.w = geoQuat.w;
        msg_body_pose.header.frame_id = "world";

        if (frame_num > 10)
        {
            path.poses.push_back(msg_body_pose);
        }
        pubPath.publish(path);

        frame_num++;
        aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
        // aver_time_consu = aver_time_consu * 0.8 + (t5 - t0) * 0.2;
        T1[time_log_counter] = Measures.lidar_beg_time;
        s_plot[time_log_counter] = aver_time_consu;
        s_plot2[time_log_counter] = kdtree_incremental_time;
        s_plot3[time_log_counter] = kdtree_search_time;
        s_plot4[time_log_counter] = fov_check_time;
        s_plot5[time_log_counter] = t5 - t0;
        s_plot6[time_log_counter] = readd_box_time;
        time_log_counter++;
        fprintf(m_lio_costtime_fp, "%.5f %.5f\r\n", g_lio_state.last_update_time - g_camera_lidar_queue.m_first_imu_time, t5 - t0);
        fflush(m_lio_costtime_fp);

        status = ros::ok();
        rate.sleep();
    }
    return 0;
}
