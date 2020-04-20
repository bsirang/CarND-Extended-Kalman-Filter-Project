#ifndef RADAR_LIDAR_EKF_H_
#define RADAR_LIDAR_EKF_H_

#include "Eigen/Dense"
#include "measurement_package.h"
#include <functional>

class RadarLidarEKF {
  using VectorXd = Eigen::VectorXd;
  using MatrixXd = Eigen::MatrixXd;
public:
  RadarLidarEKF();
  void processMeasurement(const MeasurementPackage &measurement_pack);

  const VectorXd & getState() const { return x_; };
private:
  using update_fn = std::function<VectorXd(const VectorXd & x, const VectorXd & z)>;

  void initializeWithRadar(const MeasurementPackage &measurement_pack);
  void initializeWithLidar(const MeasurementPackage &measurement_pack);

  void predict(double dt);

  void update(const MeasurementPackage &measurement_pack);
  void updateRadar(const VectorXd &z);
  void updateLidar(const VectorXd &z);
  void updateFilter(const VectorXd &z, const MatrixXd &H, const MatrixXd &R, update_fn fn);

  MatrixXd generateF(double dt);
  MatrixXd generateQ(double dt);

  enum States {
    pX,
    pY,
    vX,
    vY,
  };
  static constexpr size_t kNumStates = 4;
  static constexpr size_t kNumLidarStates = 2; // position in x/y
  static constexpr size_t kNumRadarStates = 3;

  static constexpr double kNoiseAx = 9.0;
  static constexpr double kNoiseAy = 9.0;

  bool is_initialized_{false};
  long long previous_timestamp_{0};

  // state vector
  VectorXd x_;

  // state covariance matrix
  MatrixXd P_;
};

#endif
