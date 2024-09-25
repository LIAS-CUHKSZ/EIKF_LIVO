#include "ilive.hpp"
#include "so3_math.h"
#include <chrono>
#define DiffMuch 1

ofstream Save_ba("/home/slam/workspace/ws_ILIVE/Evaluation_ILIVE/ILIVE/ba.txt",ios::ate);
ofstream Save_bg("/home/slam/workspace/ws_ILIVE/Evaluation_ILIVE/ILIVE/bg.txt",ios::ate);

void ILIVE::lio_update(Common_tools::Timer tim, PointCloudXYZINormal::Ptr &laserCloudOri, FeatsDown &feats_down, Eigen::Vector3d &euler_cur, StatesGroup &state_in)
{
    Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> G, H_T_H, I_STATE;
    Eigen::Matrix<double, 9, 9> l_jacobian;
    G.setZero();
    H_T_H.setZero();
    I_STATE.setIdentity();
    l_jacobian.setIdentity();

    PointCloudXYZINormal::Ptr coeffSel(new PointCloudXYZINormal()); 
    PointCloudXYZINormal::Ptr coeffSel_tmpt(new PointCloudXYZINormal(*feats_down.original));

    std::vector<bool> point_selected_surf(feats_down.size, true); 
    std::vector<PointVector> Nearest_Points(feats_down.size);  
    std::vector<double> res_last(feats_down.size, 1000.0);    
    std::vector<double> res_uTq(feats_down.size, 1000.0);        // initial : save the residual of the u^T*q
    std::vector<double> res_uTq_selected;   

    StatesGroup state_propagate(state_in); // 状态传播值(先验):通过计算得到的状态实例化一个StatesGroup变量

    bool rematch_en = 0;
    int rematch_num = 0;
    double maximum_pt_range = 0.0;

    flg_EKF_converged = 0;
    deltaR = 0.0;
    deltaT = 0.0;

    double match_time, solve_time, pca_time, match_start, solve_start;
    match_time = 0;
    solve_time = 0;
    pca_time = 0;
    int num_iter = NUM_MAX_ITERATIONS;

    // if (m_method=="EIKF"){
    //     num_iter = 1;
    // }

 //   if (m_method=="EIKF"){
  //  NUM_MAX_ITERATIONS=3;// if you choose use EIEKF, you can just use one step for iterated steps of Kalman filtering
 //   }
    for (int iterCount = 0; iterCount < num_iter; iterCount++) 
    {
        tim.tic("Iter"); 
        match_start = omp_get_wtime();
        laserCloudOri->clear(); // clear
        coeffSel->clear();      // clear
        res_uTq_selected.clear();

        for (int i = 0; i < feats_down.size; i += m_lio_update_point_step) // steps of sampling for iterated steps. if is 1, it means each feas will be used.
        {
            double search_start = omp_get_wtime();
            PointType &pointOri_tmpt = feats_down.original->points[i]; 
            double ori_pt_dis = sqrt(pointOri_tmpt.x * pointOri_tmpt.x + pointOri_tmpt.y * pointOri_tmpt.y + pointOri_tmpt.z * pointOri_tmpt.z);
            maximum_pt_range = std::max(ori_pt_dis, maximum_pt_range); 
            PointType &pointSel_tmpt = feats_down.updated->points[i];  
            pointBodyToWorld(&pointOri_tmpt, &pointSel_tmpt); 
            std::vector<float> pointSearchSqDis_surf;        

            auto &points_near = Nearest_Points[i]; 

            if (iterCount == 0 || rematch_en) 
            {
                point_selected_surf[i] = true;
                /** Find the closest surfaces in the map 在地图中找到最近的平面**/
                // NUM_MATCH_POINTS=5
                ikdtree.Nearest_Search(pointSel_tmpt, NUM_MATCH_POINTS, points_near, pointSearchSqDis_surf);
                float max_distance = pointSearchSqDis_surf[NUM_MATCH_POINTS - 1]; // 最近点集的最后一个元素自然最远
                if (max_distance > m_maximum_pt_kdtree_dis)                       // 超出限定距离,放弃为这个点寻找平面
                {
                    point_selected_surf[i] = false; // 当前点寻找平面失败
                }
            }

            kdtree_search_time += omp_get_wtime() - search_start;
            if (point_selected_surf[i] == false) // 当前点寻找平面失败,进入下一个点的寻找流程
                continue;

            double pca_start = omp_get_wtime();

            cv::Mat matA0(NUM_MATCH_POINTS, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matB0(NUM_MATCH_POINTS, 1, CV_32F, cv::Scalar::all(-1));
            cv::Mat matX0(NUM_MATCH_POINTS, 1, CV_32F, cv::Scalar::all(0));

            for (int j = 0; j < NUM_MATCH_POINTS; j++)
            {
                matA0.at<float>(j, 0) = points_near[j].x;
                matA0.at<float>(j, 1) = points_near[j].y;
                matA0.at<float>(j, 2) = points_near[j].z;
            }

            cv::solve(matA0, matB0, matX0, cv::DECOMP_QR); // TODO

            float pa = matX0.at<float>(0, 0);
            float pb = matX0.at<float>(1, 0);
            float pc = matX0.at<float>(2, 0);
            float pd = 1;

            float ps = sqrt(pa * pa + pb * pb + pc * pc);
            pa /= ps;
            pb /= ps;
            pc /= ps;
            pd /= ps;

            bool planeValid = true;
            for (int j = 0; j < NUM_MATCH_POINTS; j++)
            {
                if (fabs(pa * points_near[j].x + pb * points_near[j].y + pc * points_near[j].z + pd) >
                    m_planar_check_dis && (ori_pt_dis < maximum_pt_range * 0.90 || (ori_pt_dis < m_long_rang_pt_dis))) // m_planar_check_dis? maximum_pt_range? m_long_rang_pt_dis？
                {
                        planeValid = false;
                        point_selected_surf[i] = false;
                        break;
                }
            }
            if (planeValid)
            {

                float pd2 = pa * pointSel_tmpt.x + pb * pointSel_tmpt.y + pc * pointSel_tmpt.z + pd;
                float s = 1 - 0.9 * fabs(pd2) /
                                  sqrt(sqrt(pointSel_tmpt.x * pointSel_tmpt.x + pointSel_tmpt.y * pointSel_tmpt.y +
                                            pointSel_tmpt.z * pointSel_tmpt.z));
                double acc_distance = (ori_pt_dis < m_long_rang_pt_dis) ? m_maximum_res_dis : 1.0;
                if (pd2 < acc_distance) 
                {
                    point_selected_surf[i] = true;   // succeed to find the surf
                    coeffSel_tmpt->points[i].x = pa; // log the coeffiecient of the surf
                    coeffSel_tmpt->points[i].y = pb;
                    coeffSel_tmpt->points[i].z = pc;
                    coeffSel_tmpt->points[i].intensity = pd2;// log the residual
                    res_last[i] = std::abs(pd2); // 当前特征点代入平面方程产生的残差
                    res_uTq[i] = -pd;
                }
                else
                {
                    point_selected_surf[i] = false; // failed to find the surf                 
                    }
            }
            pca_time += omp_get_wtime() - pca_start;
        }
        tim.tic("Stack");
        laserCloudSelNum = 0;

        for (int i = 0; i < coeffSel_tmpt->points.size(); i++) 
        {
            if (point_selected_surf[i] && (res_last[i] <= 2.0))
            {                                                             
                laserCloudOri->push_back(feats_down.original->points[i]);
                coeffSel->push_back(coeffSel_tmpt->points[i]);  
                res_uTq_selected.push_back(res_uTq[i]);          
                laserCloudSelNum++;                                       
            }
        }

        match_time += omp_get_wtime() - match_start;
        solve_start = omp_get_wtime();

        Eigen::MatrixXd Hsub(laserCloudSelNum, 6);  
        Eigen::VectorXd meas_vec(laserCloudSelNum); 
        Hsub.setZero();
        Eigen::Matrix3d R_lidar = Eigen::Matrix3d::Identity();

        valid_lid_points =  laserCloudSelNum;
    // if(m_method=="EIKF"){
    // if (laserCloudSelNum > 500){
    //     Pose Lidar_consistent_pose; 
    //    Lidar_consistent_pose = consistent_solution_to_LIDAR(laserCloudOri, coeffSel,laserCloudSelNum,res_uTq_selected,LASER_POINT_COV);
    //    R_lidar = Lidar_consistent_pose.R;
    //    Eigen::Vector3d t_lidar = Lidar_consistent_pose.t;
    //    StatesGroup state_go_back(state_in); 
    //    state_in.rot_end=R_lidar*ext_R_lid_in_imu.transpose();
    //    state_in.pos_end=t_lidar-R_lidar*ext_R_lid_in_imu.transpose()*ext_t_lid_in_imu;
    //    auto temp = state_in-state_propagate;
    //    if(temp.block<6, 1>(0, 0).norm()>DiffMuch){
    //     state_in = state_go_back;
    //     LASER_POINT_COV=0.00015;
    //    }
    // }else{
    //     LASER_POINT_COV=0.00015;
    // }
    // }
        for (int i = 0; i < laserCloudSelNum; i++)
        {
            const PointType &laser_p = laserCloudOri->points[i];          
            Eigen::Vector3d point_lidar(laser_p.x, laser_p.y, laser_p.z); 
            Eigen::Vector3d point_world(state_in.rot_end * (ext_R_lid_in_imu* point_lidar + ext_t_lid_in_imu) + state_in.pos_end);
            const PointType &norm_p = coeffSel->points[i];               
            Eigen::Vector3d norm_vec(norm_p.x, norm_p.y, norm_p.z);    
            // for invariantEKF
            if(m_method=="InEKF"||m_method=="EIKF"){
            Eigen::Matrix3d Rimu(state_in.rot_end);
            Eigen::Vector3d timu(state_in.pos_end);
            Eigen::Vector3d rot_sub = -(Rimu * (ext_R_lid_in_imu * point_lidar + ext_t_lid_in_imu) + timu); 
            Eigen::Matrix3d r_hat;                                                                        
            r_hat << SKEW_SYM_MATRIX(rot_sub);
            Eigen::Vector3d res_1_3 = norm_vec.transpose() * r_hat;
            Hsub.row(i) << VEC_FROM_ARRAY(res_1_3), VEC_FROM_ARRAY(norm_vec); 
            //meas_vec(i) = - norm_p.intensity;
           meas_vec(i) = -(point_world.dot(norm_vec)-res_uTq_selected[i]);    
            }    
        else{
            Eigen::Matrix3d point_crossmat;
            point_lidar = ext_R_lid_in_imu * point_lidar + ext_t_lid_in_imu; // Lidar和IMU的偏移
            point_crossmat << SKEW_SYM_MATRIX(point_lidar);                  // 将点转为反对称矩阵用于叉乘
            Eigen::Vector3d A(point_crossmat * state_in.rot_end.transpose() * norm_vec);
            Hsub.row(i) << VEC_FROM_ARRAY(A), norm_p.x, norm_p.y, norm_p.z; // row(i)=A[0],A[1],A[2],norm_p.x, norm_p.y, norm_p.z

            /*** Measuremnt: distance to the closest surface/corner ***/
            meas_vec(i) = -norm_p.intensity;
}
        }

        Eigen::Vector3d rot_add, t_add ;                    // 更新量:旋转,平移
        Eigen::Matrix<double, DIM_OF_STATES, 1> solution;   // 最终解 : 29维 / 15
        Eigen::MatrixXd K(DIM_OF_STATES, laserCloudSelNum); // kalman增益

        if (!flg_EKF_inited) // 未初始化时初始化 - 前面已经初始化了
        {
            cout << ANSI_COLOR_RED_BOLD << "Run EKF init" << ANSI_COLOR_RESET << endl;
            /*** only run in initialization period ***/
            set_initial_state_cov(state_in); //////////////////////////
        }
        else
        {
            if(m_method=="InEKF"||m_method=="EIKF"){
            auto &&Hsub_T = Hsub.transpose(); // H转置 : 6xn
            H_T_H.block<6, 6>(0, 0) = Hsub_T * Hsub;  //6*6

            auto vec = state_in-state_propagate; 
            Eigen::Vector3d rot_delta(vec.block<3, 1>(0, 0));
            Eigen::Vector3d pos_delta(vec.block<3, 1>(3, 0));

            Eigen::Vector3d vel_delta(vec.block<3, 1>(6, 0));
            l_jacobian=left_jacobian_of_rotation_matrix(rot_delta, pos_delta,vel_delta);
            Eigen::Matrix<double, 9, 9> l_jacobian_inv = l_jacobian.inverse();
            Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> P_jaco=(state_in.cov / LASER_POINT_COV).inverse();
            Eigen::Matrix<double, 9, 9>  &&P_tem=l_jacobian_inv.transpose()*P_jaco.block<9,9>(0,0)*l_jacobian_inv;
             P_jaco.block<9,9>(0,0)=P_tem ;
            Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> &&K_1 =
                (H_T_H + P_jaco).inverse();
            K = K_1.block<DIM_OF_STATES, 6>(0, 0) * Hsub_T;             // K = (15xn) = (15x6) * (6xn),后面9维没必要计算,H里都是0
            auto &&K_tem=l_jacobian_inv* K.block(0,0,9,laserCloudSelNum);
            K.block(0,0,9,laserCloudSelNum)=K_tem;
            solution = K * (meas_vec+ Hsub * vec.block<6, 1>(0, 0)); // kalman增益
            state_in = state_propagate +solution;  
            }
else{
            auto &&Hsub_T = Hsub.transpose();        // H转置 : 6xn = (nx6)^T
            H_T_H.block<6, 6>(0, 0) = Hsub_T * Hsub; //(0,0)处6x6块.H^T*T
            Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> &&K_1 =
                (H_T_H + (state_in.cov / LASER_POINT_COV).inverse()).inverse();
            K = K_1.block<DIM_OF_STATES, 6>(0, 0) * Hsub_T; // K = (29x6) * (6xn) = (29xn)

            auto vec = state_propagate - state_in; // state_propagate初始=g_lio_state
            // 5>:求公式(18)的中间和右边部分(有出入:I什么的都省略了)
            solution = K * (meas_vec - Hsub * vec.block<6, 1>(0, 0)); // kalman增益
            state_in = state_propagate + solution;                 // kalman增益后的状态结果
}
            
      //      print_dash_board();

            // 7>:判断是否收敛
            rot_add = solution.block<3, 1>(0, 0);                  // 旋转增量
            t_add = solution.block<3, 1>(3, 0);                    // 平移增量
            flg_EKF_converged = false;                             // 收敛标识
            if (((rot_add.norm() * 57.3 - deltaR) < 0.01) && ((t_add.norm() * 100 - deltaT) < 0.015))
            {
                flg_EKF_converged = true; // 通过旋转和平移增量与上一次迭代的差值,判断是否收敛
            }
            // 8>:旋转和平移增量转换单位
            deltaR = rot_add.norm() * 57.3; // 角度单位
            deltaT = t_add.norm() * 100;    // 厘米单位
        }
        // printf_line;
        state_in.last_update_time = Measures.lidar_end_time;
        R_lidar = state_in.rot_end * ext_R_lid_in_imu;
        euler_cur = RotMtoEuler(R_lidar); // 获得当前lidar的里程计信息,最后这个需要发布到ros中去
        dump_lio_state_to_log(m_lio_state_fp);

        /*** Rematch Judgement 重匹配判断 ***/
        rematch_en = false;
        if (flg_EKF_converged || ((rematch_num == 0) && (iterCount == (NUM_MAX_ITERATIONS - 2))))
        {
            rematch_en = true;
            rematch_num++;
        }

        if (rematch_num >= 2 || (iterCount == NUM_MAX_ITERATIONS - 1))
        {
            if (flg_EKF_inited) // calc covariance : Pk=J-1(I-KHJ-1)Pk-1JT
            {
            if(m_method=="InEKF"||m_method=="EIKF"){
               // K.block<6,6>(0,0)=l_jacobian* K.block<6,6>(0,0);
                G.block<DIM_OF_STATES, 6>(0, 0) = K * Hsub;                                                     // K*H , 15x6
                G.block<DIM_OF_STATES, 9>(0, 0) = G.block<DIM_OF_STATES, 9>(0, 0) * l_jacobian;       // K*H*J-1 
                Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> tmp((I_STATE - G ) * state_in.cov);      // (I-KHJ-1)Pk-1
                tmp.block<9, 9>(0, 0) = l_jacobian * tmp.block<9, 9>(0, 0) * l_jacobian.transpose(); // J-1(I-KHJ-1)Pk-1JT
                state_in.cov = tmp;   }                                                                      // 公式(19): (单位阵-K*H)*Cur_协方差
            else{
                G.block<DIM_OF_STATES, 6>(0, 0) = K * Hsub;        // 对应公式(19)中 : K * H
                state_in.cov = (I_STATE - G) * state_in.cov; // 公式(19): (单位阵-K*H)*Cur_协方差
            }
               total_distance += (state_in.pos_end - position_last).norm(); // 两次state间的距离
                position_last = state_in.pos_end;
            }
            solve_time += omp_get_wtime() - solve_start;
            break;
        }
        solve_time += omp_get_wtime() - solve_start;
    }

    Save_ba << state_in.bias_a.transpose()<<std::endl;
    Save_bg << state_in.bias_g.transpose()<<std::endl;
    return;
}
 

Pose pro_to_SE3(const Eigen::VectorXd &x){
    Eigen::Matrix3d A;
    A<<x(0),x(3),x(6),
      x(1),x(4),x(7),
      x(2),x(5),x(8);
Eigen::JacobiSVD<Eigen::Matrix3d> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

Eigen::Matrix3d U = svd.matrixU();
Eigen::Matrix3d V = svd.matrixV();

Eigen::Matrix3d UVt = U * V.transpose();
Eigen::Matrix3d R = svd.matrixU() * Eigen::DiagonalMatrix<double, 3>(1, 1, UVt.determinant()) * svd.matrixV().transpose();

Pose pose;

if (R.determinant() < 0)
{
    R *= -1;
    pose.R=R;
    pose.t <<-x(9),-x(10),-x(11);

}
else
{
    pose.R=R;
    pose.t <<x(9),x(10),x(11);
}
return pose;
}

Pose ILIVE::consistent_solution_to_LIDAR(PointCloudXYZINormal::Ptr laserCloudOri, PointCloudXYZINormal::Ptr coeffSel, int N, const std::vector<double> &uTq, double &LASER_POINT_COV)
{

auto start = std::chrono::high_resolution_clock::now();

//std::cout<<"N:"<<N<<std::endl;
Eigen::MatrixXd A(N, 12);
Eigen::VectorXd b(N);
Eigen::MatrixXd U(N, 3);
// step 1:构造A矩阵和b矩阵
for (int i = 0; i < N; i++)
{
    const PointType &laser_p = laserCloudOri->points[i];
    Eigen::Vector3d point_lidar(laser_p.x, laser_p.y, laser_p.z);
    const PointType &norm_p = coeffSel->points[i];
    Eigen::Vector3d norm_vec(norm_p.x, norm_p.y, norm_p.z);
    A.row(i) << point_lidar(0) * norm_vec(0), point_lidar(0) * norm_vec(1), point_lidar(0) * norm_vec(2), point_lidar(1) * norm_vec(0), point_lidar(1) * norm_vec(1), point_lidar(1) * norm_vec(2), point_lidar(2) * norm_vec(0), point_lidar(2) * norm_vec(1), point_lidar(2) * norm_vec(2), VEC_FROM_ARRAY(norm_vec);
    b[i] = uTq[i];
    U.row(i) << VEC_FROM_ARRAY(norm_vec);
}

// step 2:估计最大特征值
Eigen::MatrixXd arg_A(N, 13);
arg_A.block(0, 0, N, 12) = A;
arg_A.block(0, 12, N, 1) = b;
Eigen::MatrixXd A_T_A_bar(13, 13);
Eigen::MatrixXd arg_A1 = arg_A.block(0, 0, N,7);
Eigen::MatrixXd arg_A2 = arg_A.block(0, 7, N,6);
A_T_A_bar.block<7, 7>(0, 0) = arg_A1.transpose() * arg_A1;
A_T_A_bar.block<6,6>(7,7) = arg_A2.transpose() * arg_A2;
A_T_A_bar.block<7, 6>(0, 7) = arg_A1.transpose() * arg_A2;
A_T_A_bar.block<6, 7>(7, 0) = arg_A2.transpose() * arg_A1;


auto &&U_T = U.transpose();
Eigen::Matrix3d q = U_T * U/N;
Eigen::MatrixXd Q_bar(12, 12);
Q_bar.setZero();
Q_bar.block<3, 3>(0, 0) = q;
Q_bar.block<3, 3>(3, 3) = q;
Q_bar.block<3, 3>(6, 6) = q;
Eigen::Matrix<double,13,13> Q;


Q.setZero();
Q.block<12, 12>(0, 0) = Q_bar;
Eigen::MatrixXd X = A_T_A_bar.inverse() * Q*N;
Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(X);
Eigen::VectorXd eigenvalues = solver.eigenvalues();
double max_eigval = eigenvalues.real().maxCoeff();
LASER_POINT_COV = 1 / abs(max_eigval);


auto end = std::chrono::high_resolution_clock::now();

// cout << "consistent_solution_to_Lidar time:" << std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count() << "ms" << endl;


// step 3:求解
auto &&A_T = A.transpose();

Eigen::MatrixXd ATA = Eigen::MatrixXd(12,12);
ATA.block<6,6>(0,0)=A.block(0, 0, N, 6).transpose() * A.block(0, 0, N, 6);
ATA.block<6,6>(0,6)=A.block(0, 0, N, 6).transpose() * A.block(0, 6, N, 6);
ATA.block<6,6>(6,0)=A.block(0, 6, N, 6).transpose() * A.block(0, 0, N, 6);
ATA.block<6,6>(6,6)=A.block(0, 6, N, 6).transpose() * A.block(0, 6, N, 6);
Eigen::MatrixXd temp = ATA/ N - LASER_POINT_COV * Q_bar;

Eigen::MatrixXd temp_inv = temp.inverse();
Eigen::VectorXd x = temp_inv * A_T * b / N;

// step 4:转换为旋转矩阵
Pose pose = pro_to_SE3(x);
//std::cout<<"Pose:"<<pose.R<<std::endl;
return pose;
}



