#include "cPnP.h"
#include <so3_math.h>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <random>

namespace pnpsolver {

/**
 * @brief Generalized Eigenvalues Solver, taken from "Eigenvalue and Generalized Eigenvalue Problems: Tutorial"
 * https://arxiv.org/pdf/1903.11240
 * 
 * det(A - lambda * B) = 0
 * 
 * @param A 
 * @param B 
 * @param eigen_vector 
 * @param eigen_values 
 * @return true 
 * @return false 
 */
static bool GeneralizedEigenSolver(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                                   Eigen::MatrixXd& eigen_vector, Eigen::MatrixXd& eigen_values) {
  int N = B.rows();
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(B);
  Eigen::VectorXd lambda_B = es.eigenvalues();
  Eigen::MatrixXd phi_B = es.eigenvectors();

  Eigen::MatrixXd lambda_B_sqrt = 1e-4 * Eigen::MatrixXd::Identity(N, N);
  for (size_t i = 0; i < N; i++) {
    lambda_B_sqrt(i, i) += sqrt(lambda_B(i));
  }

  Eigen::MatrixXd phi_B_hat = phi_B * lambda_B_sqrt.inverse();
  Eigen::MatrixXd A_hat = phi_B_hat.transpose() * A * phi_B_hat;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es_hat(A_hat);
  eigen_values = es_hat.eigenvalues();
  eigen_vector = phi_B_hat * es_hat.eigenvectors();

  return true;
}

// Function to calculate the distance between a point and a plane
bool fitPlaneRANSAC(const std::vector<Eigen::Vector3d>& points, int maxIterations,
                    double distanceThreshold) {
    if (points.size() < 3) {
        return false; // Not enough points to fit a plane
    }

    int numPoints = points.size();
    int maxInliers = 0;
    Eigen::Vector3d bestPlaneNormal;
    double bestDParameter;
    std::vector<int> sampledIndices(numPoints);

    for (int iteration = 0; iteration < maxIterations; ++iteration) {
        // Randomly sample 3 unique points
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> dist(0, numPoints - 1);

        int index1, index2, index3;
        do {
            index1 = dist(gen);
        } while (sampledIndices[index1]);
        do {
            index2 = dist(gen);
        } while (sampledIndices[index2]);
        do {
            index3 = dist(gen);
        } while (sampledIndices[index3]);

        sampledIndices[index1] = sampledIndices[index2] = sampledIndices[index3] = 1;

        Eigen::Vector3d p1 = points[index1];
        Eigen::Vector3d p2 = points[index2];
        Eigen::Vector3d p3 = points[index3];

        // Compute the plane parameters (normal vector and d) from the 3 points
        Eigen::Vector3d v1 = p2 - p1;
        Eigen::Vector3d v2 = p3 - p1;
        Eigen::Vector3d planeNormal = v1.cross(v2).normalized();
        double dParameter = -planeNormal.dot(p1);

        int inlierCount = 0;

        // Count inliers (points close to the plane)
        for (int i = 0; i < numPoints; ++i) {
            if (sampledIndices[i]) continue; // Skip sampled points
            double distance = std::abs(planeNormal.dot(points[i]) + dParameter) / planeNormal.norm();
            if (distance < distanceThreshold) {
                inlierCount++;
            }
        }

        if (inlierCount > maxInliers) {
            maxInliers = inlierCount;
            bestPlaneNormal = planeNormal;
            bestDParameter = dParameter;

            if (maxInliers >= (numPoints / 2)) {
               
                return true; // Found a plane model with enough inliers
            }
        }
    }

    if (maxInliers > (numPoints / 2)) {
        return true; // Found a plane model
    }

    return false; // No clear plane model found
}

 bool CPnP(const std::vector<Eigen::Vector2d>& points_2d,
          const std::vector<Eigen::Vector3d>& points_3d,
          const std::vector<double>& params,
          Eigen::Matrix3d& R,
          Eigen::Vector3d& t, double &sigma2_est) {
  if(points_2d.size() != points_3d.size()){
    return false;
  }

  // if(fitPlaneRANSAC(points_3d, 2, 0.1)){
  //   return false;
  // }else{
  //   return true;
  // }

  const int N = points_2d.size();

  Eigen::Vector3d bar_p3d = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < N; ++i) {
    bar_p3d += points_3d[i];
  }
  bar_p3d /= N;

  /// Step1: Calculate decentralized pixel coordinates
  Eigen::MatrixXd obs = Eigen::MatrixXd(2 * N, 1);
  for (size_t i = 0; i < points_2d.size(); ++i) {
    obs(2 * i) = points_2d[i](0) - params[2];
    obs(2 * i + 1) = points_2d[i](1) - params[3];
  }

  /// Step2: Estimate the variance of projection noises
  Eigen::MatrixXd Ones_2n = Eigen::MatrixXd::Ones(2 * N, 1);
  Eigen::Matrix2d W = Eigen::Matrix2d::Zero();
  W(0, 0) = params[0];
  W(1, 1) = params[1];

  // A, G
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2 * N, 11);
  Eigen::MatrixXd G = Eigen::MatrixXd::Zero(2 * N, 11);
  for (size_t i = 0; i < N; i++) {
    Eigen::Vector3d p3d_de = points_3d[i] - bar_p3d;
    G.block<1, 3>(2 * i, 0) = -p3d_de.transpose();
    G.block<1, 3>(2 * i + 1, 0) = -p3d_de.transpose();

    A.block<1, 3>(2 * i, 0) = -obs(2 * i) * p3d_de.transpose();
    A.block<1, 3>(2 * i, 3) = params[0] * points_3d[i].transpose();
    A(2 * i, 6) = params[0];

    A.block<1, 3>(2 * i + 1, 0) = -obs(2 * i + 1) * p3d_de.transpose();
    A.block<1, 3>(2 * i + 1, 7) = params[1] * points_3d[i].transpose();
    A(2 * i + 1, 10) = params[1];
  }

  Eigen::MatrixXd Phi = Eigen::MatrixXd(12, 12);
  Eigen::MatrixXd A1 = A.block(0, 0, 2*N, 6);
  Eigen::MatrixXd A2 = A.block(0, 6, 2*N, 5);
  Eigen::MatrixXd ATA = Eigen::MatrixXd(11, 11);
  ATA.block<6, 6>(0, 0) = A1.transpose() * A1;
  ATA.block<5,5>(6,6) = A2.transpose() * A2;
  ATA.block<6, 5>(0, 6) = A1.transpose() * A2;
  ATA.block<5, 6>(6, 0) = A2.transpose() * A1;
  Phi.block<11, 11>(0, 0) = ATA;
  Phi.block<11, 1>(0, 11) = A.transpose() * obs;
  Phi.block<1, 11>(11, 0) = obs.transpose() * A;
  Phi.block<1, 1>(11, 11) = obs.transpose() * obs;
  Phi /= (2 * N);
  Eigen::MatrixXd GTG =  Eigen::MatrixXd(11, 11);
  Eigen::MatrixXd G1 = G.block(0, 0, 2*N, 6);
  Eigen::MatrixXd G2 = G.block(0, 6, 2*N, 5);
  GTG.block<6, 6>(0, 0) = G1.transpose() * G1;
  GTG.block<5,5>(6,6) = G2.transpose() * G2;
  GTG.block<6, 5>(0, 6) = G1.transpose() * G2;
  GTG.block<5, 6>(6, 0) = G2.transpose() * G1;
  Eigen::MatrixXd Delta = Eigen::MatrixXd(12, 12);
  Delta.block<11, 11>(0, 0) = GTG;
  Delta.block<11, 1>(0, 11) = G.transpose() * Ones_2n;
  Delta.block<1, 11>(11, 0) = Ones_2n.transpose() * G;
  Delta(11, 11) = 2 * N;
  Delta /= (2 * N);


  Eigen::GeneralizedEigenSolver<Eigen::MatrixXd> ges;
  ges.compute(Phi, Delta);
  Eigen::VectorXcd eigenvalues = ges.eigenvalues();
  sigma2_est = 100.0;
  for (int i = 0; i < eigenvalues.rows(); ++i) {
    if (sigma2_est > abs(eigenvalues(i).real())) {
      sigma2_est = abs(eigenvalues(i).real());
    }
  }


  /// Step3: Calculate the bias-eliminated solution
  Eigen::VectorXd est_bias_eli = (ATA - sigma2_est * GTG).inverse() *
                                 (A.transpose() * obs - sigma2_est * G.transpose() * Ones_2n);
  /// Step4: Recover R and t
  Eigen::Matrix3d R_bias_eli;
  R_bias_eli.block<1, 3>(0, 0) = est_bias_eli.segment<3>(3).transpose();
  R_bias_eli.block<1, 3>(1, 0) = est_bias_eli.segment<3>(7).transpose();
  R_bias_eli.block<1, 3>(2, 0) = est_bias_eli.segment<3>(0).transpose();

  Eigen::Vector3d t_bias_eli;
  t_bias_eli << est_bias_eli(6), est_bias_eli(10),
      1 - bar_p3d.transpose() * est_bias_eli.segment<3>(0);

  double normalize_factor = pow(R_bias_eli.determinant(), 1.0 / 3.0);
  R_bias_eli /= normalize_factor;
  t_bias_eli /= normalize_factor;

  /// Step5: Project the rotation matrix into SO(3) using SVD
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(R_bias_eli, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::MatrixXd UVt = svd.matrixU() * svd.matrixV().transpose();
  R_bias_eli = svd.matrixU() * Eigen::DiagonalMatrix<double, 3>(1, 1, UVt.determinant()) * svd.matrixV().transpose();
  if (R_bias_eli.determinant() < 0) {
    R = -R_bias_eli;
  }else{
    R = R_bias_eli;
  }
  t = t_bias_eli;
  return true;
}

void estimatePoseRANSAC( const std::vector<cv::Point3f> &list_points3d, // list with model 3D coordinates
const std::vector<cv::Point2f> &list_points2d, // list with scene 2D coordinates 
const cv::Mat &K, // intrinsic camera calibration matrix
Eigen::Matrix3d &_R, // computed rotation matrix
Eigen::Vector3d &_t, // computed translation matrix
cv::Mat &inliers,int flags, int iterationsCount, // PnP method; inliers container
float reprojectionError, float confidence ) // RANSAC parameters
{
 cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_32FC1); // vector of distortion coefficients
 cv::Mat rvec = cv::Mat::zeros(3, 1, CV_32FC1); // output rotation vector
 cv::Mat tvec = cv::Mat::zeros(3, 1, CV_32FC1); // output translation vector
 bool useExtrinsicGuess = false; // if true the function uses the provided rvec and tvec values as
 // initial approximations of the rotation and translation vectors
//  cv::solvePnP( list_points3d, list_points2d, K, distCoeffs, rvec, tvec,false, cv::SOLVEPNP_EPNP );
 cv::solvePnPRansac( list_points3d, list_points2d, K, distCoeffs, rvec, tvec,
 useExtrinsicGuess, iterationsCount, reprojectionError, confidence,
 inliers, flags );
 cv::Matx33d _R_matrix;
 cv::Rodrigues(rvec,_R_matrix); // converts Rotation Vector to Matrix
 Eigen::Map<Eigen::Matrix3d> eigenT( _R_matrix.val ); 
  _R = eigenT.cast<double>();
  _t = Eigen::Vector3d( tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2) );
}
} 