#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

#define PI 3.14159265

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd Hx = H_ * x_;
  VectorXd y = z - Hx;
  
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_len = x_.size();
  MatrixXd I = MatrixXd::Identity(x_len, x_len);
  P_ = (I - K * H_) * P_;
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  //VectorXd z_pred = H_ * x_;
  //update the state by using Extended Kalman Filter equations

  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);


  double rho = sqrt(px * px + py * py);
  if (fabs(rho) < 0.00001)
  {
    rho = 0.00001;
  }
  double phi = atan2(py, px);
  double rho_dot = (px*vx+py*vy)/rho;

  // Cheat sheet: 17.1
  VectorXd hx(3); // h(x')
  hx << rho, phi, rho_dot;

  VectorXd y = z - hx;

  //Normalize angle to within -pi and pi if needed
  while (y(1) > PI || y(1) < -PI)
  {
    if (y(1) > PI)
    {
      y(1) -= 2 * PI;
    }
    else if (y(1) < -PI)
    {
      y(1) += 2 * PI;
    }
  }

  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  //estimate:
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
