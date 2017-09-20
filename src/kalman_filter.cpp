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
    // Why use different update functions(update and updateEKF) in Kalman Filter?
    // https://discussions.udacity.com/t/why-use-different-update-functions-update-and-updateekf-in-kalman-filter/302125
    //
    // Radar estimates going out of whack
    // https://discussions.udacity.com/t/radar-estimates-going-out-of-whack/347808

    // Get predicted location in polar coords.
    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);
  
    float rho = sqrtf(px * px + py * py);
    float phi = atan2f(py, px);

    // float rho_dot = (px * vx + py * vy) / (rho + FLT_MIN);
    float rho_dot;
    if (fabs(rho) < 0.0001) {
        rho_dot = 0;
    } else {
        rho_dot = (px * vx + py * vy) / rho;
    }
    float diff = z(1) - phi;
    // Forum
    // https://discussions.udacity.com/t/ekf-gets-off-track/276122
    if (6.2 < diff) {
        phi += 2 * M_PI;
    }


    VectorXd hx(3);
    hx << rho, phi, rho_dot;
	VectorXd y = z - hx;
	// VectorXd z_pred = H_ * x_;
	// VectorXd y = z - z_pred;


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
