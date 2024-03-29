#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // set H laser
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // initialize state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

  // initialize process matrix Q
  ekf_.Q_ = MatrixXd(4, 4);

  // initialize F
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  noise_ax = 9.0;
  noise_ay = 9.0;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      double rho = measurement_pack.raw_measurements_[0];
      double theta = measurement_pack.raw_measurements_[1];
      ekf_.x_ << rho * cos(theta),
                 rho * sin(theta),
                 0,
                 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // Initialize state.
      ekf_.x_ << measurement_pack.raw_measurements_[0],
                 measurement_pack.raw_measurements_[1],
                 0,
                 0;
    }
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // set the process covariance matrix Q
  ekf_.Q_ <<  pow(dt, 4.0)/4.0*noise_ax, 0, pow(dt, 3.0)/2.0*noise_ax, 0,
              0, pow(dt, 4.0)/4.0*noise_ay, 0, pow(dt, 3.0)/2.0*noise_ay,
              pow(dt, 3.0)/2.0*noise_ax, 0, pow(dt, 2.0)*noise_ax, 0,
              0, pow(dt, 3.0)/2.0*noise_ay, 0, pow(dt, 2.0)*noise_ay;

  // Update the state transition matrix F according to the new elapsed time.
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  /**
   * Prediction
   */
  ekf_.Predict();

  /**
   * Update
   */

  /**
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    double rho = measurement_pack.raw_measurements_[0];
    double theta = measurement_pack.raw_measurements_[1];
    double ro_dot = measurement_pack.raw_measurements_[2];
    VectorXd z_(3);
    z_ << rho, theta, ro_dot;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(z_);
  } else {
    VectorXd z_ = VectorXd(2);
    z_ << measurement_pack.raw_measurements_[0],
          measurement_pack.raw_measurements_[1];
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(z_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
