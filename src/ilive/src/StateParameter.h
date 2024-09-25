//
// Created by slam on 2022/11/25.
//
#include <Eigen/Eigen>
#ifndef ILIVE_STATEPARAMETER_H
#define ILIVE_STATEPARAMETER_H
struct StateParameter{
    //load Lidar-IMU extinction-Rotation
    Eigen::Matrix3d Lidar_IN_IMU_Rotation;
    //load Lidar-IMU extinction-Position
    Eigen::Vector3d Lidar_IN_IMU_Position;
    //load Camera-IMU extinction-Rotation
    Eigen::Matrix3d Camera_IN_IMU_Rotation;
    //load Camera-IMU extinction-Position
    Eigen::Vector3d Camera_IN_IMU_Position;
    //T.B.D other parameters
    Eigen::Vector3d Gravity;
    //initial acceleration at the stationary state
    bool if_online_calibration_camera;
    //if online calibration camera
    int max_IMU_calibration_num;

    std::string m_method;
    double COV_ACC_NOISE_DIAG;
    double COV_OMEGA_NOISE_DIAG;
    double COV_BIAS_ACC_NOISE_DIAG;
    double COV_BIAS_GYRO_NOISE_DIAG;

};
#endif //ILIVE_STATEPARAMETER_H
