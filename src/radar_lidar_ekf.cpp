
#include "radar_lidar_ekf.h"

#include "tools.h"

RadarLidarEKF::RadarLidarEKF() : x_(kNumStates), P_(kNumStates, kNumStates) {
  x_ << 0, 0, 0, 0;

  P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;
}

void RadarLidarEKF::processMeasurement(const MeasurementPackage &measurement_pack) {
  if (!is_initialized_) {
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      initializeWithRadar(measurement_pack);
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      initializeWithLidar(measurement_pack);
    } else {
      throw std::invalid_argument("RadarLidarEKF: Unsupported sensor type!");
    }
    return;
  }

  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  predict(dt);
  update(measurement_pack);
}

void RadarLidarEKF::initializeWithRadar(const MeasurementPackage &measurement_pack) {
  previous_timestamp_ = measurement_pack.timestamp_;
  VectorXd cart = Tools::PolarToCartesian2D(measurement_pack.raw_measurements_);
  x_ << cart(0), cart(1), 0.0, 0.0;
  is_initialized_ = true;
}

void RadarLidarEKF::initializeWithLidar(const MeasurementPackage &measurement_pack) {
  previous_timestamp_ = measurement_pack.timestamp_;
  x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0.0, 0.0;
  is_initialized_ = true;
}

void RadarLidarEKF::predict(double dt) {
  MatrixXd F = generateF(dt);
  MatrixXd Q = generateQ(dt);
  MatrixXd Ft = F.transpose();

  x_ = F * x_;
  P_ = F * P_ * Ft + Q;
}

Eigen::MatrixXd RadarLidarEKF::generateF(double dt) {
  MatrixXd F(kNumStates, kNumStates);
  F << 1, 0, dt, 0,
       0, 1, 0, dt,
       0, 0, 1, 0,
       0, 0, 0, 1;
  return F;
}

Eigen::MatrixXd RadarLidarEKF::generateQ(double dt) {
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  MatrixXd Q(kNumStates, kNumStates);
  Q << dt_4 / 4 * kNoiseAx, 0, dt_3 / 2 * kNoiseAx, 0, 0, dt_4 / 4 * kNoiseAy, 0,
      dt_3 / 2 * kNoiseAy, dt_3 / 2 * kNoiseAx, 0, dt_2 * kNoiseAx, 0, 0, dt_3 / 2 * kNoiseAy, 0,
      dt_2 * kNoiseAy;
  return Q;
}

void RadarLidarEKF::update(const MeasurementPackage &measurement_pack) {
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    updateRadar(measurement_pack.raw_measurements_);
  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    updateLidar(measurement_pack.raw_measurements_);
  } else {
    throw std::invalid_argument("RadarLidarEKF: Unsupported sensor type!");
  }
}

void RadarLidarEKF::updateRadar(const VectorXd &z) {
  MatrixXd R(kNumRadarStates, kNumRadarStates);
  R << 0.09, 0, 0,
       0, 0.0009, 0,
       0, 0, 0.09;

  MatrixXd Hj = Tools::CalculateJacobian(x_);
  updateFilter(z, Hj, R, [](const Eigen::VectorXd &x, const Eigen::VectorXd &z) {
    VectorXd y = z - Tools::CartesianToPolar(x);
    y(1) = Tools::WrapAngle(y(1));
    return y;
  });
}

void RadarLidarEKF::updateLidar(const VectorXd &z) {
  MatrixXd R(kNumLidarStates, kNumLidarStates);
  R << 0.0225, 0, 0, 0.0225;
  MatrixXd H(kNumLidarStates, kNumStates);
  H << 1, 0, 0, 0, 0, 1, 0, 0;
  updateFilter(z, H, R, [H](const Eigen::VectorXd &x, const Eigen::VectorXd &z) {
    VectorXd y = z - (H * x);
    return y;
  });
}

void RadarLidarEKF::updateFilter(const VectorXd &z, const MatrixXd &H, const MatrixXd &R,
                                 update_fn fn) {
  VectorXd y = fn(x_, z);
  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P_ * Ht + R;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H) * P_;
}
