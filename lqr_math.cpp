#include "nav2_time_aware_controller/time_aware_lqr_controller.hpp"

#include <utility>

#include <Eigen/Dense>

namespace nav2_time_aware_controller
{

std::pair<double, double> TimeAwareLQRController::solveLQR(
  double e_s,
  double e_y,
  double e_psi,
  double v_ref,
  double dt) const
{
  Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
  A(1, 2) = v_ref * dt;

  Eigen::Matrix<double, 3, 2> B = Eigen::Matrix<double, 3, 2>::Zero();
  B(0, 0) = dt;
  B(2, 1) = dt;

  Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
  Q(0, 0) = q_s_;
  Q(1, 1) = q_y_;
  Q(2, 2) = q_psi_;

  Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
  R(0, 0) = r_v_;
  R(1, 1) = r_w_;

  Eigen::Matrix3d P = Eigen::Matrix3d::Zero();
  P(0, 0) = qf_s_;
  P(1, 1) = qf_y_;
  P(2, 2) = qf_psi_;

  Eigen::Matrix<double, 2, 3> K = Eigen::Matrix<double, 2, 3>::Zero();
  for (int i = 0; i < lqr_horizon_steps_; ++i) {
    const Eigen::Matrix2d S = R + B.transpose() * P * B;
    const Eigen::Matrix<double, 2, 3> rhs = B.transpose() * P * A;
    K = S.ldlt().solve(rhs);
    P = Q + A.transpose() * P * (A - B * K);
  }

  Eigen::Vector3d x;
  x << e_s, e_y, e_psi;
  const Eigen::Vector2d u = -K * x;
  return {u(0), u(1)};
}

}