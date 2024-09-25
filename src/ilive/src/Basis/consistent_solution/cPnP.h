#pragma once
#include <Eigen/Eigen>
#include <vector>
#include <opencv2/opencv.hpp>


namespace pnpsolver {

/**
 * @brief  Estimate the camera pose from 2D-3D correspondences
 * 
 * The CPnP algorithm is taken from "CPnP: Consistent Pose Estimator for Perspective-n-Point 
 * Problem with Bias Elimination" https://arxiv.org/pdf/2209.05824
 * 
 * @param points_2d 2D image points
 * @param points_3d 3D world points
 * @param params    camera params [fx, fy, cx, cy]
 * @param qvec      Quaternion (qw, qx, qy, qz) from world to camera
 * @param tvec      Translation (tx, ty, tz) from world to camera
 * @param qvec_GN   Refined quaternion (qw, qx, qy, qz) from world to camera
 * @param tvec_GN   Refined translation (tx, ty, tz) from world to camera
 * @return true 
 * @return false 
 */
bool CPnP(const std::vector<Eigen::Vector2d>& points_2d,
          const std::vector<Eigen::Vector3d>& points_3d,
          const std::vector<double>& params,
          Eigen::Matrix3d& R,
          Eigen::Vector3d& t, double& sigma2_est);
void estimatePoseRANSAC(const std::vector<cv::Point3f> &list_points3d, // list with model 3D coordinates
 const std::vector<cv::Point2f> &list_points2d, // list with scene 2D coordinates 
const cv::Mat &K, // intrinsic camera calibration matrix
Eigen::Matrix3d &_R, // computed rotation matrix
Eigen::Vector3d &_t, // computed translation matrix
cv::Mat &inliers, int flags=cv::SOLVEPNP_EPNP, int iterationsCount=2, // PnP method; inliers container
 float reprojectionError=8.0, float confidence=0.90 );
}  // namespace pnpsolver