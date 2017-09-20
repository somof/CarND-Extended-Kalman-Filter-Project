#include "kalman_filter.h"
#include "float.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
    // VectorXd y = z - H_ * x_;
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;

    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();

    // MatrixXd K =  P_ * Ht * Si;
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

    //new state
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

    // Forum
    // https://discussions.udacity.com/t/why-use-different-update-functions-update-and-updateekf-in-kalman-filter/302125

    // TODO
    // 座標変換
    // z_pred -> 入れ替え


  
    // Get predicted location in polar coords.
    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);
  
    // float eps = 0.000001;  // Make sure we don't divide by 0.
    // if (fabs(px) < eps && fabs(py) < eps) {
    //     px = eps;
    //     py = eps;
    // } else if (fabs(px) < eps) {
    //     px = eps;
    // }
  
    float rho = sqrtf(powf(px, 2) + powf(py, 2));
    float phi = atan2f(py, px);
    float rho_dot = (px * vx + py * vy) / (rho + FLT_MIN);
  
    VectorXd hx(3);
    hx << rho, phi, rho_dot;



	// VectorXd z_pred = H_ * x_;
	// VectorXd y = z - z_pred;

	VectorXd y = z - hx;

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
