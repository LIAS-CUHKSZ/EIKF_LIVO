#include "ilive.hpp"
#include "Basis/consistent_solution/cPnP.h"
#include "Basis/tools/tools_mem_used.h"
#include "Basis/tools/tools_logger.hpp"
#include "so3_math.h"
#define USING_CERES 0

double get_huber_loss_scale(double reprojection_error, double outlier_threshold = 1.0)
{
    double scale = 1.0;
    if (reprojection_error / outlier_threshold < 1.0)
    {
        scale = 1.0;
    }
    else
    {
        scale = (2 * sqrt(reprojection_error) / sqrt(outlier_threshold) - 1.0) / reprojection_error;
    }
    return scale;
}

const int minimum_iteration_pts = 10;
bool ILIVE::vio_projection(StatesGroup &state_in, Rgbmap_tracker &op_track)
{
   Common_tools::Timer tim;
    tim.tic();
    scope_color(ANSI_COLOR_BLUE_BOLD);
    StatesGroup state_iter = state_in;
    state_iter.cam_intrinsic << g_cam_K(0, 0), g_cam_K(1, 1), g_cam_K(0, 2), g_cam_K(1, 2);
    state_iter.pos_ext_i2c = m_inital_pos_ext_i2c;
    state_iter.rot_ext_i2c = m_inital_rot_ext_i2c;

    Eigen::Matrix<double, -1, -1> H_mat;
    Eigen::Matrix<double, -1, 1> meas_vec;
    Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> H_T_H, I_STATE;
    Eigen::Matrix<double, DIM_OF_STATES, 1> solution;
    Eigen::Matrix<double, -1, -1> K, KH;

    Eigen::SparseMatrix<double> H_mat_spa, H_T_H_spa, KH_spa, vec_spa, I_STATE_spa,temp_inv_mat;
    I_STATE.setIdentity();
    I_STATE_spa = I_STATE.sparseView();
    double fx, fy, cx, cy, time_td;

    int total_pt_size = op_track.m_map_rgb_pts_in_current_frame_pos.size();

    valid_cam_points = total_pt_size;

   // cout<<"f2f  number of tracked vision:"<<endl;
    //cout<<total_pt_size<<endl;

    std::vector<double> last_reprojection_error_vec(total_pt_size), current_reprojection_error_vec(total_pt_size);

    if (total_pt_size < minimum_iteration_pts)
    {
        state_in = state_iter;
        return false;
    }
    H_mat.resize(total_pt_size * 2, DIM_OF_STATES);
    meas_vec.resize(total_pt_size * 2, 1);
    double last_repro_err = 3e8;
    int avail_pt_count = 0;
    double last_avr_repro_err = 0;

    double acc_reprojection_error = 0;
    double img_res_scale = 1.0;
    fx = state_iter.cam_intrinsic(0);
    fy = state_iter.cam_intrinsic(1);
    cx = state_iter.cam_intrinsic(2);
    cy = state_iter.cam_intrinsic(3);
    time_td = state_iter.td_ext_i2c_delta;

    Eigen::Matrix<double, 9, 9> l_jacobian;
    l_jacobian.setIdentity(9,9);
    int iter_sum= vio_proj_iter_times;
    mat_3_3 R_c2w;
    vec_3 t_c2w;
    mat_3_3 R_imu;
    vec_3 t_imu;
    vec_3 t_w2c; 
    mat_3_3 R_w2c;
    // std::vector<cv::Point2f> points_2d;
    // std::vector<cv::Point3f> points_3d;
    std::vector<Eigen::Vector2d> points_2d;
    std::vector<Eigen::Vector3d> points_3d;
    double camera_measurement_covariance= 1.0/m_cam_measurement_weight;

    if (m_method=="EIKF")
    {
        std::vector<double> params;
        params.push_back(fx);
        params.push_back(fy);
        params.push_back(cx);
        params.push_back(cy);
        // cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 
        //                         fx, 0, cx,
        //                         0, fy, cy,
        //                         0, 0, 1);
        // cv::Mat rvec; // 旋转向量
        // cv::Mat tvec; // 平移向量
        for (auto tem = op_track.m_map_rgb_pts_in_last_frame_pos.begin(); tem != op_track.m_map_rgb_pts_in_last_frame_pos.end(); tem++)
        {
            points_3d.push_back(Eigen::Vector3d(((RGB_pts *)tem->first)->get_pos()));
            points_2d.push_back(Eigen::Vector2d(tem->second.x, tem->second.y));
            // const Eigen::Vector3d pt_3d_w = ((RGB_pts *)tem->first)->get_pos();
            // points_3d.push_back(cv::Point3f(pt_3d_w.x(),pt_3d_w.y(),pt_3d_w.z()));
            // points_2d.push_back(tem->second);
        }
        ROS_ASSERT(points_2d.size()==points_3d.size());
        if (points_2d.size()>150)
        {
        Eigen::Matrix3d R_cam_consistent;
        Eigen::Vector3d t_cam_consistent;
        if(pnpsolver::CPnP(points_2d, points_3d, params, R_cam_consistent, t_cam_consistent, camera_measurement_covariance))
        {
            ROS_INFO("PnP success");
       
        //cv::Mat inliers;
        //pnpsolver::estimatePoseRANSAC(points_3d, points_2d, cameraMatrix, R_cam_consistent, t_cam_consistent,inliers,2,10,0.9);
        t_w2c=t_cam_consistent;
        R_w2c=R_cam_consistent;
        //m_cam_measurement_weight = std::max(0.001,std::min(0.01,m_cam_measurement_weight));
        R_c2w = R_cam_consistent.transpose();
        t_c2w = -R_cam_consistent.transpose() * t_cam_consistent;
        R_imu = R_c2w*state_iter.rot_ext_i2c.transpose();
        t_imu = t_c2w - R_imu * state_iter.pos_ext_i2c;
        state_iter.rot_end=R_imu;
        state_iter.pos_end=t_imu;
        iter_sum = 1;}
        }else{
            ROS_INFO("PnP failed");
            iter_sum= vio_proj_iter_times;
        }
        // for (auto tem = op_track.m_map_rgb_pts_in_last_frame_pos.begin(); tem != op_track.m_map_rgb_pts_in_last_frame_pos.end(); tem++)
        // {
        //     points_3d.push_back(Eigen::Vector3d(((RGB_pts *)tem->first)->get_pos()));
        //     points_2d.push_back(Eigen::Vector2d(tem->second.x, tem->second.y));
        // }
        // if(points_2d.size()>100){
        // pnpsolver::CPnP(points_2d, points_3d, params, R_cam_consistent, t_cam_consistent, camera_measurement_covariance);
        // t_w2c=t_cam_consistent;
        // R_w2c=R_cam_consistent;
        // //m_cam_measurement_weight = std::max(0.001,std::min(0.01,m_cam_measurement_weight));
        // R_c2w = R_cam_consistent.transpose();
        // t_c2w = -R_cam_consistent.transpose() * t_cam_consistent;
        // R_imu = R_c2w*state_iter.rot_ext_i2c.transpose();
        // t_imu = t_c2w - R_imu * state_iter.pos_ext_i2c;
        // state_iter.rot_end=R_imu;
        // state_iter.pos_end=t_imu;
        // iter = 1;
    }
   
    for (int iter_count = 0; iter_count < iter_sum; iter_count++)
    {
        R_imu = state_iter.rot_end;
        t_imu = state_iter.pos_end;
        t_c2w = R_imu * state_iter.pos_ext_i2c + t_imu;
        R_c2w = R_imu * state_iter.rot_ext_i2c; 
        t_w2c = -R_c2w.transpose() * t_c2w;
        R_w2c = R_c2w.transpose();

        int pt_idx = -1;
        acc_reprojection_error = 0;
        vec_3 pt_3d_w, pt_3d_cam;
        vec_2 pt_img_measure, pt_img_proj, pt_img_vel;
        eigen_mat_d<2, 3> mat_pre;
        eigen_mat_d<3, 3> mat_A, mat_B, mat_C, mat_D, pt_hat;
        H_mat.setZero();
        solution.setZero();
        meas_vec.setZero();
        avail_pt_count = 0;
        time_td = state_iter.td_ext_i2c_delta;


        for (auto it = op_track.m_map_rgb_pts_in_last_frame_pos.begin(); it != op_track.m_map_rgb_pts_in_last_frame_pos.end(); it++)
        {

            pt_3d_w = ((RGB_pts *)it->first)->get_pos();
            pt_img_vel = ((RGB_pts *)it->first)->m_img_vel;
            pt_img_measure = vec_2(it->second.x, it->second.y);
            pt_3d_cam = R_w2c * pt_3d_w + t_w2c;
            pt_img_proj = vec_2(fx * pt_3d_cam(0) / pt_3d_cam(2) + cx, fy * pt_3d_cam(1) / pt_3d_cam(2) + cy)+ time_td * pt_img_vel;
            double repro_err = (pt_img_proj - pt_img_measure).norm();
            double huber_loss_scale = get_huber_loss_scale(repro_err); // huber loss,对误差进行调制
            pt_idx++;
            acc_reprojection_error += repro_err;
            last_reprojection_error_vec[pt_idx] = repro_err;

            avail_pt_count++;
            // Appendix E of r2live_Supplementary_material.
            // https://github.com/hku-mars/r2live/blob/master/supply/r2live_Supplementary_material.pdf
            mat_pre << fx / pt_3d_cam(2), 0, -fx * pt_3d_cam(0) / (pt_3d_cam(2)*pt_3d_cam(2)), 0, fy / pt_3d_cam(2), -fy * pt_3d_cam(1) / (pt_3d_cam(2)*pt_3d_cam(2));

            meas_vec.block(pt_idx * 2, 0, 2, 1) = (pt_img_proj - pt_img_measure) * huber_loss_scale / img_res_scale;

            
            if(m_method=="InEKF"||m_method=="EIKF"){
                mat_B = -state_iter.rot_ext_i2c.transpose() * R_imu.transpose(); // translation jacobian,为了运算效率仅计算一次IRc * RiT
                Eigen::Vector3d tmp(pt_3d_w);
                mat_A = -mat_B * vec_to_hat(tmp); // rotation jacobian
            }else{
                pt_hat = Sophus::SO3d::hat((R_imu.transpose() * (pt_3d_w - t_imu)));
                mat_A = state_iter.rot_ext_i2c.transpose() * pt_hat;
                mat_B = -state_iter.rot_ext_i2c.transpose() * (R_imu.transpose());
            }
                H_mat.block(pt_idx * 2, 0, 2, 3) = mat_pre * mat_A * huber_loss_scale;
                H_mat.block(pt_idx * 2, 3, 2, 3) = mat_pre * mat_B * huber_loss_scale;

                if( DIM_OF_STATES >15){ //estimate dt
                    H_mat.block(pt_idx * 2, 15,2,1)=pt_img_vel *huber_loss_scale;
                }
            }
            H_mat = H_mat / img_res_scale;
            acc_reprojection_error /= total_pt_size;
            last_avr_repro_err = acc_reprojection_error;
            if (avail_pt_count < minimum_iteration_pts)
                break;

    // calc delta of state
            if(m_method=="InEKF"||m_method=="EIKF"){
                H_mat_spa = H_mat.sparseView();
                Eigen::SparseMatrix<double> Hsub_T_temp_mat = H_mat_spa.transpose();
                vec_spa = (state_iter - state_in).sparseView();
                Eigen::Vector3d rot_delta;
                rot_delta = vec_spa.toDense().block<3, 1>(0, 0);
                Eigen::Vector3d pos_delta;
                pos_delta = vec_spa.toDense().block<3, 1>(3, 0);
                Eigen::Vector3d vel_delta=vec_spa.toDense().block<3, 1>(6, 0); 

                l_jacobian=left_jacobian_of_rotation_matrix(rot_delta, pos_delta,vel_delta);
                //TODO: 可以优化！！！！
                Eigen::Matrix<double, 9, 9> l_jacobian_inv = l_jacobian.inverse();

                Eigen::Matrix<double,DIM_OF_STATES, DIM_OF_STATES> P_inverse_jaco=eigen_mat<-1, -1>(state_in.cov).inverse()*camera_measurement_covariance;
                P_inverse_jaco.block<9,9>(0,0)=l_jacobian_inv.transpose()*P_inverse_jaco.block<9,9>(0,0)*l_jacobian_inv;
                H_T_H_spa = Hsub_T_temp_mat * H_mat_spa;
                // Notice that we have combine some matrix using () in order to boost the matrix multiplication.
                temp_inv_mat =
                    ((H_T_H_spa.toDense() + P_inverse_jaco).inverse()).sparseView();
                //K=J-1(HtR-1H+J-1tP-1J-1)-1HtR-1
                KH_spa = temp_inv_mat * (H_T_H_spa);
                solution = (temp_inv_mat * (Hsub_T_temp_mat *(-1 * meas_vec.sparseView())) + KH_spa * vec_spa).toDense();
                solution.block<9,1>(0,0)=l_jacobian_inv*solution.block<9,1>(0,0);
                state_iter = state_in + solution;
            }
            else{   
                H_mat_spa = H_mat.sparseView();
                Eigen::SparseMatrix<double> Hsub_T_temp_mat = H_mat_spa.transpose();
                vec_spa = (state_iter - state_in).sparseView();
                H_T_H_spa = Hsub_T_temp_mat * H_mat_spa;
                // Notice that we have combine some matrix using () in order to boost the matrix multiplication.
                temp_inv_mat =
                    ((H_T_H_spa.toDense() + eigen_mat<-1, -1>(state_in.cov).inverse()*camera_measurement_covariance).inverse()).sparseView();
                KH_spa = temp_inv_mat * (H_T_H_spa);
                solution = (temp_inv_mat * (Hsub_T_temp_mat * ((-1 * meas_vec.sparseView()))) - (I_STATE_spa - KH_spa) * vec_spa).toDense();
                state_iter = state_iter + solution;
            }
            if (fabs(acc_reprojection_error - last_repro_err) < 0.01)
                break;
            last_repro_err = acc_reprojection_error;
        }

    // finish iteration, update the covariance
    if (avail_pt_count >= minimum_iteration_pts) // calc Pk|k=J-1(I-KHJ-1)Pk|k-1J-1
    {
        if(m_method=="InEKF"||m_method=="EIKF"){
            state_iter.cov = temp_inv_mat.toDense()*camera_measurement_covariance;
        }
        else{        
            state_iter.cov = temp_inv_mat.toDense()*camera_measurement_covariance;
        }
        Eigen::VectorXd diags = state_iter.cov.diagonal();
        for (int i = 0; i < diags.rows(); i++) {
            if (diags(i) < 0.0) {
            printf("negative diags",i, diags(i));
            ROS_ERROR("negative diags of %d", i);
            }
        }
    }
        state_iter.td_ext_i2c += state_iter.td_ext_i2c_delta;
        state_iter.td_ext_i2c_delta = 0;        
        state_in = state_iter; // update state
        return true;
}

bool ILIVE::vio_photometric(StatesGroup &state_in, Rgbmap_tracker &op_track, std::shared_ptr<Image_frame> &image)
{
    Common_tools::Timer tim;
    tim.tic();
    StatesGroup state_iter = state_in;

    state_iter.cam_intrinsic << g_cam_K(0, 0), g_cam_K(1, 1), g_cam_K(0, 2), g_cam_K(1, 2);
    state_iter.pos_ext_i2c = m_inital_pos_ext_i2c;
    state_iter.rot_ext_i2c = m_inital_rot_ext_i2c;

    Eigen::Matrix<double, -1, -1> H_mat, R_mat_inv;
    Eigen::Matrix<double, -1, 1> meas_vec;
    Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> G, H_T_H, I_STATE;
    Eigen::Matrix<double, DIM_OF_STATES, 1> solution;
    Eigen::Matrix<double, -1, -1> K, KH;
    Eigen::SparseMatrix<double> H_mat_spa, H_T_H_spa, R_mat_inv_spa, K_spa, KH_spa, vec_spa, I_STATE_spa;
    I_STATE.setIdentity();
    I_STATE_spa = I_STATE.sparseView();
    double fx, fy, cx, cy, time_td;

    int total_pt_size = op_track.m_map_rgb_pts_in_current_frame_pos.size();
    std::vector<double> last_reprojection_error_vec(total_pt_size), current_reprojection_error_vec(total_pt_size);
    if (total_pt_size < minimum_iteration_pts)
    {
        state_in = state_iter;
        return false;
    }

    int err_size = 3;
    H_mat.resize(total_pt_size * err_size, DIM_OF_STATES);
    meas_vec.resize(total_pt_size * err_size, 1);
    R_mat_inv.resize(total_pt_size * err_size, total_pt_size * err_size);

    double last_repro_err = 3e8;
    int avail_pt_count = 0;
    double last_avr_repro_err = 0;
    int if_esikf = 1;

    double acc_photometric_error = 0;

    fx = state_iter.cam_intrinsic(0);
    fy = state_iter.cam_intrinsic(1);
    cx = state_iter.cam_intrinsic(2);
    cy = state_iter.cam_intrinsic(3);

    Eigen::Matrix<double, 9, 9> l_jacobian;
    l_jacobian.setIdentity(9,9);
    for (int iter_count = 0; iter_count < vio_f2map_iter_times; iter_count++)
    {
        time_td = state_iter.td_ext_i2c_delta;

        mat_3_3 R_imu = state_iter.rot_end;
        vec_3 t_imu = state_iter.pos_end;
        vec_3 t_c2w = R_imu * state_iter.pos_ext_i2c + t_imu;
        mat_3_3 R_c2w = R_imu * state_iter.rot_ext_i2c; // world to camera frame


        vec_3 t_w2c = -R_c2w.transpose() * t_c2w;
        mat_3_3 R_w2c = R_c2w.transpose();
        int pt_idx = -1;
        acc_photometric_error = 0;
        vec_3 pt_3d_w, pt_3d_cam;
        vec_2 pt_img_measure, pt_img_proj, pt_img_vel;
        eigen_mat_d<2, 3> mat_pre;
        eigen_mat_d<3, 2> mat_photometric;
        eigen_mat_d<3, 3> mat_d_pho_d_img;
        eigen_mat_d<3, 3> mat_A, mat_B, pt_hat;
        R_mat_inv.setZero();
        H_mat.setZero();
        solution.setZero();
        meas_vec.setZero();
        avail_pt_count = 0;
        int iter_layer = 0;
        tim.tic("Build_cost");
        for (auto it = op_track.m_map_rgb_pts_in_last_frame_pos.begin(); it != op_track.m_map_rgb_pts_in_last_frame_pos.end(); it++)
        {
            if (((RGB_pts *)it->first)->m_N_rgb < 3)
            {
                continue;
            }
            pt_idx++;
            pt_3d_w = ((RGB_pts *)it->first)->get_pos();
            pt_img_vel = ((RGB_pts *)it->first)->m_img_vel;
            pt_img_measure = vec_2(it->second.x, it->second.y);
            pt_3d_cam = R_w2c * pt_3d_w + t_w2c;
            pt_img_proj = vec_2( fx * pt_3d_cam( 0 ) / pt_3d_cam( 2 ) + cx, fy * pt_3d_cam( 1 ) / pt_3d_cam( 2 ) + cy ) + time_td * pt_img_vel;

            vec_3 pt_rgb = ((RGB_pts *)it->first)->get_rgb();
            mat_3_3 pt_rgb_info = mat_3_3::Zero();
            mat_3_3 pt_rgb_cov = ((RGB_pts *)it->first)->get_rgb_cov();
            for (int i = 0; i < 3; i++)
            {
                pt_rgb_info(i, i) = 1.0 / pt_rgb_cov(i, i);
                R_mat_inv(pt_idx * err_size + i, pt_idx * err_size + i) = pt_rgb_info(i, i);
                // R_mat_inv( pt_idx * err_size + i, pt_idx * err_size + i ) =  1.0;
            }
            vec_3 obs_rgb_dx, obs_rgb_dy;
            vec_3 obs_rgb = image->get_rgb(pt_img_proj(0), pt_img_proj(1), 0, &obs_rgb_dx, &obs_rgb_dy);
            vec_3 photometric_err_vec = (obs_rgb - pt_rgb);
            mat_photometric.setZero();
            mat_photometric.col(0) = obs_rgb_dx;
            mat_photometric.col(1) = obs_rgb_dy;

            double huber_loss_scale = get_huber_loss_scale(photometric_err_vec.norm());
            photometric_err_vec *= huber_loss_scale;
            double photometric_err = photometric_err_vec.transpose() * pt_rgb_info * photometric_err_vec;

            acc_photometric_error += photometric_err;

            last_reprojection_error_vec[pt_idx] = photometric_err;

            avail_pt_count++;
            mat_pre << fx / pt_3d_cam(2), 0, -fx * pt_3d_cam(0) / (pt_3d_cam(2)*pt_3d_cam(2)), 0, fy / pt_3d_cam(2), -fy * pt_3d_cam(1) / (pt_3d_cam(2)*pt_3d_cam(2));
            mat_d_pho_d_img = mat_photometric * mat_pre;

            meas_vec.block(pt_idx * 3, 0, 3, 1) = photometric_err_vec;
if(m_method=="InEKF"||m_method=="EIKF"){
            mat_B = -state_iter.rot_ext_i2c.transpose() * R_imu.transpose(); // translation jacobian,为了运算效率仅计算一次IRc * RiT
            Eigen::Vector3d tmp(pt_3d_w);
            mat_A = -mat_B * vec_to_hat(tmp); // rotation jacobian
}
else{
            pt_hat = Sophus::SO3d::hat((R_imu.transpose() * (pt_3d_w - t_imu)));
            mat_A = state_iter.rot_ext_i2c.transpose() * pt_hat;
            mat_B = -state_iter.rot_ext_i2c.transpose() * (R_imu.transpose());
}
            H_mat.block(pt_idx * 3, 0, 3, 3) = mat_d_pho_d_img * mat_A * huber_loss_scale;
            H_mat.block(pt_idx * 3, 3, 3, 3) = mat_d_pho_d_img * mat_B * huber_loss_scale;
        }
        R_mat_inv_spa = R_mat_inv.sparseView();

        last_avr_repro_err = acc_photometric_error;
        if (avail_pt_count < minimum_iteration_pts)
        {
            break;
        }
        // Esikf
        tim.tic("Iter");
        if (if_esikf)
        {
if(m_method=="InEKF"||m_method=="EIKF"){
            H_mat_spa = H_mat.sparseView();
            Eigen::SparseMatrix<double> Hsub_T_temp_mat = H_mat_spa.transpose();
            vec_spa = (state_iter - state_in).sparseView();
            Eigen::Vector3d rot_delta(vec_spa.toDense().block<3, 1>(0, 0));
            Eigen::Vector3d pos_delta(vec_spa.toDense().block<3, 1>(3, 0));
            Eigen::Vector3d vel_delta(vec_spa.toDense().block<3, 1>(6, 0));
            l_jacobian=left_jacobian_of_rotation_matrix(rot_delta, pos_delta,vel_delta);
            Eigen::Matrix<double, 9, 9> l_jacobian_inv = l_jacobian.inverse();

            Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> P_jaco=eigen_mat<-1, -1>(state_in.cov * m_cam_measurement_weight).inverse();
            P_jaco.block<9,9>(0,0)=l_jacobian_inv.transpose()*P_jaco.block<9,9>(0,0)*l_jacobian_inv;

            H_T_H_spa = Hsub_T_temp_mat * R_mat_inv_spa * H_mat_spa;
            Eigen::SparseMatrix<double> temp_inv_mat =
                (H_T_H_spa.toDense() + P_jaco).inverse().sparseView();
            Eigen::SparseMatrix<double> Ht_R_inv = (Hsub_T_temp_mat * R_mat_inv_spa);
            K_spa = temp_inv_mat * Ht_R_inv;
            int column = K_spa.toDense().cols();
            KH_spa = K_spa*H_mat_spa;
            K_spa.toDense().block(0,0,9,column)=l_jacobian_inv*K_spa.toDense().block(0,0,9,column);
            solution = (K_spa * ((-1 * meas_vec.sparseView()) + H_mat_spa * vec_spa)).toDense();
            state_iter = state_in + solution;

}
else{
            H_mat_spa = H_mat.sparseView();
            Eigen::SparseMatrix<double> Hsub_T_temp_mat = H_mat_spa.transpose();
            vec_spa = (state_iter - state_in).sparseView();
            H_T_H_spa = Hsub_T_temp_mat * R_mat_inv_spa * H_mat_spa;
            Eigen::SparseMatrix<double> temp_inv_mat =
                (H_T_H_spa.toDense() + (state_in.cov * m_cam_measurement_weight).inverse()).inverse().sparseView();
            Eigen::SparseMatrix<double> Ht_R_inv = (Hsub_T_temp_mat * R_mat_inv_spa);
            KH_spa = temp_inv_mat * Ht_R_inv * H_mat_spa;
            solution = (temp_inv_mat * (Ht_R_inv * ((-1 * meas_vec.sparseView()))) - (I_STATE_spa - KH_spa) * vec_spa).toDense();
            state_iter = state_iter + solution;
}
        }


        if ((acc_photometric_error / total_pt_size) < 10) // By experience.
        {
            break;
        }
        if (fabs(acc_photometric_error - last_repro_err) < 0.01)
        {
            break;
        }
        last_repro_err = acc_photometric_error;
    }
    if (if_esikf && avail_pt_count >= minimum_iteration_pts)
    {
         if(m_method=="InEKF"||m_method=="EIKF"){
                state_iter.cov.block<9,9>(0,0) = l_jacobian*state_iter.cov.block<9,9>(0,0)*l_jacobian.transpose();
                state_iter.cov = ((I_STATE_spa - KH_spa) * state_iter.cov.sparseView()).toDense();
        }
        else{        
            state_iter.cov = ((I_STATE_spa - KH_spa) * state_iter.cov.sparseView()).toDense();
        }
    }
    state_iter.td_ext_i2c += state_iter.td_ext_i2c_delta;

    state_iter.td_ext_i2c_delta = 0;
    state_in = state_iter;

    return true;
}
