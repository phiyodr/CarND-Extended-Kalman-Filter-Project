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

  /**
   * TODO: Finish initializing the FusionEKF.  (x, F, H_laser, H_jacobian, P, etc.)
   	    + H_laser_
        + Hj_
   * TODO: Set the process and measurement noises
        + ?
        
   */
  // Initializing H: Measurement matrices
   H_laser_ << 1, 0, 0, 0,
               0, 1, 0, 0;

   Hj_ << 1, 1, 0, 0,
          1, 1, 0, 0,
          1, 1, 1, 1; 

  // Initializing P: State Covariance matrix
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;
  
  // Initializing F: Transition matrix
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;
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
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    ekf_.x_ = VectorXd(4);
    //ekf_.x_ << 1, 1, 1, 1;
    ekf_.x_ << 0,0,0,0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      cout << "EKF: LASER init " << endl;
      // TODO: Initialize state.
      ekf_.x_ << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1), 0, 0;
    }
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
       cout << "EKF: RADAR init " << endl;
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      double rho     = measurement_pack.raw_measurements_(0);
      double phi     = measurement_pack.raw_measurements_(1);
      double rho_dot = measurement_pack.raw_measurements_(2);
      ekf_.x_(0) = rho     * cos(phi);
      ekf_.x_(1) = rho     * sin(phi);      
      //ekf_.x_(2) = rho_dot * cos(phi);
      //ekf_.x_(3) = rho_dot * sin(phi);
    }
    
    previous_timestamp_ = measurement_pack.timestamp_ ;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   *      + Update F
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   *      + Update Q
   */

   //      + Update F: Transition matrix with dt
   double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
   previous_timestamp_ = measurement_pack.timestamp_;
  
   ekf_.F_(0, 2) = dt;
   ekf_.F_(1, 3) = dt;
   //      + Update Q: Process covariance matrix
  
   double noise_ax = 9.;
   double noise_ay = 9.;
  
   double dt_2 = dt * dt;
   double dt_3 = dt_2 * dt;
   double dt_4 = dt_3 * dt;

   ekf_.Q_ = MatrixXd(4, 4);
   ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
              0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
              dt_3 / 2 * noise_ax,  0, dt_2 * noise_ax, 0,
              0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;  
  
   /***************************************************
   * Prediction
   ****************************************************/
  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    // TODO: Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
      
  } else {
    // TODO: Radar updates
    Tools tools;
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
