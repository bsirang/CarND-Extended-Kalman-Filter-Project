#include "tools.h"

#include <iostream>
#include <stdexcept>

using Eigen::MatrixXd;
using Eigen::VectorXd;

VectorXd Tools::CalculateRMSE(const std::vector<VectorXd> &estimations,
                              const std::vector<VectorXd> &ground_truth) {
  if (estimations.size() != ground_truth.size()) {
    throw std::invalid_argument("CalculateRMSE - input vector size mismatch!");
  }
  size_t n = estimations.size();
  if (n == 0) {
    throw std::invalid_argument("CalculateRMSE - input vectors are empty!");
  }

  VectorXd rmse(estimations[0].size());
  rmse << 0, 0, 0, 0;

  for (size_t i = 0; i < n; ++i) {
    VectorXd temp = (estimations[i] - ground_truth[i]);
    temp = temp.array() * temp.array();
    rmse = rmse + temp;
  }

  rmse = rmse.array() / static_cast<float>(n);
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
  if (x_state.size() != 4) {
    throw std::invalid_argument("CalculateJacobian - Invalid state vector size. Must equal 4.");
  }
  MatrixXd Hj(3, 4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // check division by zero
  if (px == 0.0f && py == 0.0f) {
    std::cout << "Avoiding divide by zero!" << std::endl;
    return MatrixXd::Zero(3, 4);
  }

  // compute the Jacobian matrix

  float px2 = px * px;
  float py2 = py * py;
  float mag = ::sqrt(px2 + py2);
  float mag2 = mag * mag;

  float mag3_2 = (mag2 * mag2 * mag2) * mag;

  Hj(0, 0) = px / mag;
  Hj(0, 1) = py / mag;
  Hj(0, 2) = 0.0f;
  Hj(0, 3) = 0.0f;

  Hj(1, 0) = -py / mag2;
  Hj(1, 1) = px / mag2;
  Hj(1, 2) = 0.0f;
  Hj(1, 3) = 0.0f;

  Hj(2, 0) = py * (vx * py - vy * px) / mag3_2;
  Hj(2, 1) = px * (vy * px - vx * py) / mag3_2;
  Hj(2, 2) = px / mag;
  Hj(2, 3) = py / mag;

  return Hj;
}

VectorXd Tools::CartesianToPolar(const VectorXd &x) {
  VectorXd r(3);
  float px = x(0);
  float py = x(1);
  float vx = x(2);
  float vy = x(3);

  r(0) = ::sqrt(px * px + py * py);
  r(1) = WrapAngle(::atan2(py, px));
  r(2) = (px * vx + py * vy) / r(0);
  return r;
}

double Tools::WrapAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2 * M_PI;
  }
  return angle;
}
