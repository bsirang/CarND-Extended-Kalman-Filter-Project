#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>

#include "Eigen/Dense"
#include "Eigen/src/Core/util/DisableStupidWarnings.h"

namespace Tools {
/**
 * A helper function to calculate RMSE.
 */
Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                              const std::vector<Eigen::VectorXd> &ground_truth);

/**
 * A helper function to calculate Jacobians.
 */
Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd &x_state);

/**
 * A helper function to calculate polar coordinates
 */
Eigen::VectorXd CartesianToPolar(const Eigen::VectorXd &x_state);

/**
 * A helper function to wrap a radian angle between -PI and PI
 */
double WrapAngle(double angle);

};  // namespace Tools

#endif  // TOOLS_H_
