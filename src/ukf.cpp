#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  //set state dimension
  n_x_ = 5;

  //set augmented dimension
  n_aug_ = 7;

  //set measurement dimension, radar can measure r, phi, and r_dot
  n_z_rad_ = 3;

  //set measurement dimension, laser can measure p_x and p_y
  n_z_lsr_ = 2;

  //define spreading parameter
  lambda_ = 3 - n_aug_;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  //create augmented mean vector
  x_aug_ = VectorXd(n_aug_);

  //create augmented state covariance
  P_aug_ = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);
    
  //create vector for weights
  weights_ = VectorXd(2 * n_aug_ + 1);

  //create sigma point matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  //create example vector for incoming radar measurement
  z_rad_ = VectorXd(n_z_rad_);

  //create example vector for incoming radar measurement
  z_lsr_ = VectorXd(n_z_lsr_);

  //create matrix for sigma points in measurement space
  Zsig_rad_ = MatrixXd(n_z_rad_, 2 * n_aug_ + 1);

  //create matrix for sigma points in measurement space
  Zsig_lsr_ = MatrixXd(n_z_lsr_, 2 * n_aug_ + 1);

  //mean predicted measurement
  z_pred_rad_ = VectorXd(n_z_rad_);

  //mean predicted measurement
  z_pred_lsr_ = VectorXd(n_z_lsr_);

  //innovation covariance matrix S
  S_rad_ = MatrixXd(n_z_rad_, n_z_rad_);

  //innovation covariance matrix S
  S_lsr_ = MatrixXd(n_z_lsr_, n_z_lsr_);

  //add measurement noise covariance matrix
  R_rad_ = MatrixXd(n_z_rad_, n_z_rad_);

  //add measurement noise covariance matrix
  R_lsr_ = MatrixXd(n_z_lsr_, n_z_lsr_);

  //create matrix for cross correlation Tc_
  Tc_rad_ = MatrixXd(n_x_, n_z_rad_);

  //create matrix for cross correlation Tc_
  Tc_lsr_ = MatrixXd(n_x_, n_z_lsr_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
//  std_a_ = 30;
  std_a_ = 1;

  // Process noise standard deviation yaw acceleration in rad/s^2
//  std_yawdd_ = 30;
  std_yawdd_ = 1;

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  /**
  TODO: (DONE?)

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  is_initialized_ = false;
  previous_timestamp_ = 0;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO: (DONE)

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  /*****************************************************************************
  *  Initialization
  ****************************************************************************/
  if (!is_initialized_) {

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

      // Convert radar from polar to cartesian coordinates and initialize state with initial location and zero velocity.
      float rho = meas_package.raw_measurements_[0];
      float phi = meas_package.raw_measurements_[1];
      x_ << rho * sin(phi), rho * cos(phi), 0, 0, 0;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {

      //set the state with the initial location and zero velocity
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
    }
    
    // Set the state covariance matrix to the identity matrix
    P_.setIdentity();
    P_ << 1, 0, 0, 0, 0,
          0, 0.1, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;
    
    previous_timestamp_ = meas_package.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
  *  Prediction
  ****************************************************************************/

  //compute the time elapsed between the current and previous measurements
  float dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = meas_package.timestamp_;

  Prediction(dt);

  /*****************************************************************************
  *  Update
  ****************************************************************************/

  // Check if the signal is from a Radar or Laser
  if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) && (use_radar_ == true)) {
    // Radar updates
    UpdateRadar(meas_package);
  }
  else if ((meas_package.sensor_type_ == MeasurementPackage::LASER) && (use_laser_ == true)) {
    // Laser updates
    UpdateLidar(meas_package);
  }

  // print the output
  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;
}


/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO: (DONE)

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  //create augmented mean state
  x_aug_.head(5) = x_;
  x_aug_(5) = 0;
  x_aug_(6) = 0;

  //create augmented covariance matrix
  P_aug_.fill(0.0);
  P_aug_.topLeftCorner(5, 5) = P_;
  P_aug_(5, 5) = std_a_ * std_a_;
  P_aug_(6, 6) = std_yawdd_ * std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug_.llt().matrixL();

  //create augmented sigma points
  Xsig_aug_.col(0) = x_aug_;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug_.col(i + 1) = x_aug_ + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug_.col(i + 1 + n_aug_) = x_aug_ - sqrt(lambda_ + n_aug_) * L.col(i);
  }

  //predict augmented sigma points
  for (int i = 0; i< 2 * n_aug_ + 1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug_(0, i);
    double p_y = Xsig_aug_(1, i);
    double v = Xsig_aug_(2, i);
    double yaw = Xsig_aug_(3, i);
    double yawd = Xsig_aug_(4, i);
    double nu_a = Xsig_aug_(5, i);
    double nu_yawdd = Xsig_aug_(6, i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
      px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    }
    else {
      px_p = p_x + v * delta_t*cos(yaw);
      py_p = p_y + v * delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a * delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }

  // set weights
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights_(0) = weight_0;
  for (int i = 1; i<2 * n_aug_ + 1; i++) {  //2n+1 weights
    double weight = 0.5 / (n_aug_ + lambda_);
    weights_(i) = weight;
  }

  //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    std::cout << Xsig_pred_.col(i)(3) << std::endl;
    std::cout << x_(3) << std::endl;
    std::cout << x_diff(3) << std::endl;
    while (x_diff(3)> M_PI) x_diff(3) -= 2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3) += 2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  // Import the current measurement in polar coordinates
  float x = meas_package.raw_measurements_[0];
  float y = meas_package.raw_measurements_[1];
  z_lsr_ << x, y;

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

                                              // extract values for better readibility
    double p_x_ = Xsig_pred_(0, i);
    double p_y_ = Xsig_pred_(1, i);

    // measurement model
    Zsig_lsr_(0, i) = p_x_;                                 //x
    Zsig_lsr_(1, i) = p_y_;                                 //y
  }

  z_pred_lsr_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    z_pred_lsr_ = z_pred_lsr_ + weights_(i) * Zsig_lsr_.col(i);
  }

  S_lsr_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
                                              //residual
    VectorXd z_diff_ = Zsig_lsr_.col(i) - z_pred_lsr_;

    S_lsr_ = S_lsr_ + weights_(i) * z_diff_ * z_diff_.transpose();
  }

  R_lsr_ << std_laspx_ * std_laspx_, 0,
    0, std_laspy_ * std_laspy_;
  S_lsr_ = S_lsr_ + R_lsr_;

  //calculate cross correlation matrix
  Tc_lsr_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

                                              //residual
    VectorXd z_diff_ = Zsig_lsr_.col(i) - z_pred_lsr_;

    // state difference
    VectorXd x_diff_ = Xsig_pred_.col(i) - x_;

    Tc_lsr_ = Tc_lsr_ + weights_(i) * x_diff_ * z_diff_.transpose();
  }

  //Kalman gain K;
  MatrixXd K_lsr_ = Tc_lsr_ * S_lsr_.inverse();

  //residual
  VectorXd z_diff_ = z_lsr_ - z_pred_lsr_;
  std::cout << z_lsr_(1) << std::endl;
  std::cout << z_pred_lsr_(1) << std::endl;
  std::cout << z_diff_(1) << std::endl;

  //update state mean and covariance matrix
  x_ = x_ + K_lsr_ * z_diff_;
  P_ = P_ - K_lsr_ * S_lsr_*K_lsr_.transpose();

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  // Import the current measurement in polar coordinates
  float rho = meas_package.raw_measurements_[0];
  float phi = meas_package.raw_measurements_[1];
  float rhod = meas_package.raw_measurements_[2];
  z_rad_ << rho, phi, rhod;

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x_ = Xsig_pred_(0, i);
    double p_y_ = Xsig_pred_(1, i);
    double v_ = Xsig_pred_(2, i);
    double yaw_ = Xsig_pred_(3, i);

    double v1_ = cos(yaw_)*v_;
    double v2_ = sin(yaw_)*v_;

    // measurement model
    Zsig_rad_(0, i) = sqrt(p_x_*p_x_ + p_y_ * p_y_);                        //r
    Zsig_rad_(1, i) = atan2(p_y_, p_x_);                                 //phi
    Zsig_rad_(2, i) = (p_x_*v1_ + p_y_ * v2_) / sqrt(p_x_*p_x_ + p_y_ * p_y_);   //r_dot
  }

  z_pred_rad_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    z_pred_rad_ = z_pred_rad_ + weights_(i) * Zsig_rad_.col(i);
  }

  S_rad_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
                                              //residual
    VectorXd z_diff_ = Zsig_rad_.col(i) - z_pred_rad_;

    //angle normalization
    while (z_diff_(1)> M_PI) z_diff_(1) -= 2.*M_PI;
    while (z_diff_(1)<-M_PI) z_diff_(1) += 2.*M_PI;

    S_rad_ = S_rad_ + weights_(i) * z_diff_ * z_diff_.transpose();
  }

  R_rad_ << std_radr_ * std_radr_, 0, 0,
    0, std_radphi_*std_radphi_, 0,
    0, 0, std_radrd_*std_radrd_;
  S_rad_ = S_rad_ + R_rad_;

  //calculate cross correlation matrix
  Tc_rad_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

                                              //residual
    VectorXd z_diff_ = Zsig_rad_.col(i) - z_pred_rad_;
    //angle normalization
    while (z_diff_(1)> M_PI) z_diff_(1) -= 2.*M_PI;
    while (z_diff_(1)<-M_PI) z_diff_(1) += 2.*M_PI;

    // state difference
    VectorXd x_diff_ = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff_(3)> M_PI) x_diff_(3) -= 2.*M_PI;
    while (x_diff_(3)<-M_PI) x_diff_(3) += 2.*M_PI;

    Tc_rad_ = Tc_rad_ + weights_(i) * x_diff_ * z_diff_.transpose();
  }

  //Kalman gain K;
  MatrixXd K_ = Tc_rad_ * S_rad_.inverse();

  //residual
  VectorXd z_diff_ = z_rad_ - z_pred_rad_;
  std::cout << z_rad_(1) << std::endl;
  std::cout << z_pred_rad_(1) << std::endl;
  std::cout << z_diff_(1) << std::endl;

  //angle normalization
  while (z_diff_(1)> M_PI) z_diff_(1) -= 2.*M_PI;
  while (z_diff_(1)<-M_PI) z_diff_(1) += 2.*M_PI;


  //update state mean and covariance matrix
  x_ = x_ + K_ * z_diff_;
  P_ = P_ - K_ * S_rad_*K_.transpose();
}
