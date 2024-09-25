#include "ilive.hpp"
#include "Basis/tools/tools_mem_used.h"
#include "Basis/tools/tools_logger.hpp"
#define USING_CERES 1

void SigintHandler(int sig)
{
  ros::shutdown();
}

void ILIVE::service_VIO_update()
{
    // Init cv windows for debug
    op_track.set_intrinsic( g_cam_K, 0*g_cam_dist , cv::Size( m_vio_image_width / m_vio_scale_factor, m_vio_image_heigh / m_vio_scale_factor ) );
    op_track.m_maximum_vio_tracked_pts = m_maximum_vio_tracked_pts;
    m_map_rgb_pts.m_minimum_depth_for_projection = m_tracker_minimum_depth;
    m_map_rgb_pts.m_maximum_depth_for_projection = m_tracker_maximum_depth;

    if (if_save){
    cv::imshow( "Control panel", generate_control_panel_img().clone() );
    }

    Common_tools::Timer tim;
    cv::Mat             img_get;
    while ( ros::ok() )
    {   
        // cv_keyboard_callback();
        while ( g_camera_lidar_queue.m_if_have_lidar_data == 0 )
        {
            ros::spinOnce();
            std::this_thread::sleep_for( std::chrono::milliseconds( THREAD_SLEEP_TIM ) );
            std::this_thread::yield();
            continue;
        }

        if ( m_queue_image_with_pose.size() == 0 )
        {
            ros::spinOnce();
            std::this_thread::sleep_for( std::chrono::milliseconds( THREAD_SLEEP_TIM ) );
            std::this_thread::yield();
            continue;
        }

        m_camera_data_mutex.lock();
        
        while ( m_queue_image_with_pose.size() > m_maximum_image_buffer )
        {
            cout << ANSI_COLOR_BLUE_BOLD << "=== Pop image! current queue size = " << m_queue_image_with_pose.size() << " ===" << ANSI_COLOR_RESET
                 << endl;
            op_track.track_img( m_queue_image_with_pose.front(), -20 );
            m_queue_image_with_pose.pop_front();
        }

        std::shared_ptr< Image_frame > img_pose = m_queue_image_with_pose.front();
        double message_time = img_pose->m_timestamp;
        m_queue_image_with_pose.pop_front();
        m_camera_data_mutex.unlock();
      
        g_camera_lidar_queue.m_camera_imu_td = g_lio_state.td_ext_i2c;
        g_camera_lidar_queue.m_last_visual_time = img_pose->m_timestamp+g_camera_lidar_queue.m_camera_imu_td;

        img_pose->set_frame_idx( g_camera_frame_idx );
        tim.tic( "Frame" );

        if ( g_camera_frame_idx == 0 )
        {
            std::vector< cv::Point2f >                pts_2d_vec;
            std::vector< std::shared_ptr< RGB_pts > > rgb_pts_vec;
            // while ( ( m_map_rgb_pts.is_busy() ) || ( ( m_map_rgb_pts.m_rgb_pts_vec.size() <= 100 ) ) )
            while (  m_map_rgb_pts.m_rgb_pts_vec.size() <= 100 )
            {
                ros::spinOnce();
                std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );
            }
            set_image_pose( img_pose, g_lio_state ); // For first frame pose, we suppose that the motion is static.
            m_map_rgb_pts.selection_points_for_projection( img_pose, &rgb_pts_vec, &pts_2d_vec, m_track_windows_size / m_vio_scale_factor );
            op_track.init( img_pose, rgb_pts_vec, pts_2d_vec );
            g_camera_frame_idx++;
            continue;
        }

        g_camera_frame_idx++;
        tim.tic( "Wait" );
        
        while ( g_camera_lidar_queue.if_camera_can_process() == false )
        {
            ros::spinOnce();
            std::this_thread::sleep_for( std::chrono::milliseconds( THREAD_SLEEP_TIM ) );
            std::this_thread::yield();
            cv_keyboard_callback();
        }
        g_cost_time_logger.record( tim, "Wait" );
        m_mutex_lio_process.lock();
        tim.tic( "Frame" );
        tim.tic( "Track_img" );
        StatesGroup state_out;

        m_cam_measurement_weight = std::max( m_cov_scale* 0.005, std::min( m_number_of_new_visited_voxel/1500.0, m_cov_scale* 0.01 ) );
        if ( vio_preintegration( g_lio_state, state_out, img_pose->m_timestamp + g_lio_state.td_ext_i2c ) == false )
        {
            m_mutex_lio_process.unlock();
            continue;
        }

      




        set_image_pose( img_pose, state_out );

        op_track.track_img( img_pose, -20 );
        g_cost_time_logger.record( tim, "Track_img" );
        tim.tic( "Ransac" );
        //set_image_pose( img_pose, state_out );

        // ANCHOR -  remove point using PnP.
        if ( op_track.remove_outlier_using_ransac_pnp( img_pose ) == 0 )
        {
           cout << ANSI_COLOR_RED_BOLD << "****** Remove_outlier_using_ransac_pnp error*****" << ANSI_COLOR_RESET << endl;
        }
        g_cost_time_logger.record( tim, "Ransac" );
         tim.tic( "Vio_f2m" );
        if(m_vio_f2map_init)
        {
            vio_photometric( state_out, op_track, img_pose );
        }
        g_cost_time_logger.record( tim, "Vio_f2m" );
        tim.tic( "Vio_f2f" );
        wait_render_thread_finish(); // 等待地图点渲染完成
        if(m_vio_f2f_init)
        {
            vio_projection( state_out, op_track );
        }

        g_cost_time_logger.record( tim, "Vio_f2f" );
        set_image_pose( img_pose, state_out );

    
        g_lio_state = state_out;
        // print_dash_board();
        set_image_pose( img_pose, state_out );

        if ( 1)
        {
            if ( 1 ) // Using multiple threads for rendering
            {
                m_map_rgb_pts.m_if_get_all_pts_in_boxes_using_mp = 0;
                m_render_thread = std::make_shared< std::shared_future< void > >( m_thread_pool_ptr->commit_task(
                    render_pts_in_voxels_mp, img_pose, &m_map_rgb_pts.m_voxels_recent_visited, img_pose->m_timestamp ) );
            }
            else
            {
                m_map_rgb_pts.m_if_get_all_pts_in_boxes_using_mp = 0;
            }
            m_map_rgb_pts.m_last_updated_frame_idx = img_pose->m_frame_idx;

            if ( m_if_record_mvs )
            {
                m_mvs_recorder.insert_image_and_pts( img_pose, m_map_rgb_pts.m_pts_last_hitted );
            }
        }
        // ANCHOR - render point cloud
        dump_lio_state_to_log( m_lio_state_fp );
        m_mutex_lio_process.unlock();
        // cout << "Solve image pose cost " << tim.toc("Solve_pose") << endl;
        m_map_rgb_pts.update_pose_for_projection( img_pose, -0.4 );
        op_track.update_and_append_track_pts( img_pose, m_map_rgb_pts, m_track_windows_size / m_vio_scale_factor, 1000000);
        g_cost_time_logger.record( tim, "Frame" );
        double frame_cost = tim.toc( "Frame" );
        g_image_vec.push_back( img_pose );
        frame_cost_time_vec.push_back( frame_cost );
        if ( g_image_vec.size() > 10 )
        {
            g_image_vec.pop_front();
            frame_cost_time_vec.pop_front();
        }
        tim.tic( "Pub" );
        double display_cost_time = std::accumulate( frame_cost_time_vec.begin(), frame_cost_time_vec.end(), 0.0 ) / frame_cost_time_vec.size();
        g_vio_frame_cost_time = display_cost_time;
        publish_camera_odom( img_pose, message_time );
        publish_track_img( img_pose->m_raw_img, display_cost_time );

        if ( m_if_pub_raw_img )
        {
            publish_raw_img( img_pose->m_raw_img );
        }

        if ( g_camera_lidar_queue.m_if_dump_log )
        {
            g_cost_time_logger.flush();
        }

        signal(SIGINT, SigintHandler);
    }
}