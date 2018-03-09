#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  // Calculate intermediate terms
  VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  // Recover state parameters
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];

  // Calculate intermediate terms
  float c1 = sqrt(px*px + py*py);
  float c2 = atan2(py,px);
  float c3 = (px*vx + py*vy)/c1;

  // Calculate y using z'=h(x)
  VectorXd z_pred = VectorXd(3);
  z_pred << c1, c2, c3;
	VectorXd y = z - z_pred;

  // Normalize PHI in y
  // PHI needs to be between PI and -PI, so
  // add or subtract 2PI until this is the
  // case.
  while (y[1] > M_PI || y[1] < -M_PI) {
    if (y[1] > M_PI) {
      y[1] = y[1] - 2*M_PI;
    } else if (y[1] < -M_PI) {
      y[1] = y[1] + 2*M_PI;
    }
  }
  
//  std::cout << "PHI:" << c2 << ":" << z[1] << ":" << y[1] << std::endl;
//  if (c2 > M_PI || c2 < -M_PI) {
//    std::cout << "  OUT-OF-RANGE" << std::endl;
//  }
  
  // Calculate intermediate terms - at this point, H_ should be set to Hj
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;

}
