#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
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
  R_laser_ <<
      0.0225, 0,
      0     , 0.0225;

  //measurement covariance matrix - radar
  R_radar_ <<
      0.09, 0,      0,
      0,    0.0009, 0,
      0,    0,      0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  // create a 4D state vector, we don't know yet the values of the x state
  ekf_.x_ = VectorXd(4);
  // input at ProcessMeasurement()

  // state covariance matrix
  ekf_.P_ = MatrixXd(4, 4);
  // ekf_.P_ <<
  //  1000,    0,    0,    0,
  //     0, 1000,    0,    0,
  //     0,    0, 1000,    0,
  //     0,    0,    0, 1000;
  // kf_.P_ <<
  //     1, 0, 0, 0,
  //     0, 1, 0, 0,
  //     0, 0, 1000, 0,
  //     0, 0, 0, 1000;

  // state transition matrix
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ <<
      1, 0, 1, 0,
      0, 1, 0, 1,
      0, 0, 1, 0,
      0, 0, 0, 1;

  // process covariance matrix
  ekf_.Q_ = MatrixXd(4, 4);
  // ekf_.Q_ <<
  //     dt_4/4*noise_ax,               0, dt_3/2*noise_ax,               0,
  //     0              , dt_4/4*noise_ay,               0, dt_3/2*noise_ay,
  //     dt_3/2*noise_ax,               0,   dt_2*noise_ax,               0,
  //     0              , dt_3/2*noise_ay,               0,   dt_2*noise_ay;

  // measurement matrix
  ekf_.H_ = MatrixXd(2, 4);
  //
  // if Laser
  //     ekf_.H_ = MatrixXd(2, 4);
  // if Radar
  //     ekf_.H_ = MatrixXd(3, 4);

  // measurement covariance matrix
  ekf_.R_ = MatrixXd(2, 2);
  //
  // if Laser
  //     ekf_.R_ = MatrixXd(2, 2);
  // if Radar
  //     ekf_.R_ = MatrixXd(3, 3);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;

    // state covariance matrix
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ <<
        1, 0,    0,    0,
        0, 1,    0,    0,
        0, 0, 1000,    0,
        0, 0,    0, 1000;
    // ekf_.P_ <<
    //     1000,    0,    0,    0,
    //        0, 1000,    0,    0,
    //        0,    0, 1000,    0,
    //        0,    0,    0, 1000;

    //measurement matrix
    // ekf_.H_ = MatrixXd(2, 4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
        // measurement matrix
        ekf_.H_ = MatrixXd(3, 4);

        // TODO

        // measurement covariance matrix
        ekf_.R_ = MatrixXd(3, 3);
        ekf_.R_ = R_radar_;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
        // measurement matrix
        ekf_.H_ = MatrixXd(2, 4);
        // ekf_.H_ = H_laser_;
        ekf_.H_ <<
            1, 0, 0, 0,
            0, 1, 0, 0;
        // measurement covariance matrix
        ekf_.R_ = MatrixXd(2, 2);
        ekf_.R_ = R_laser_;
    }

    // test code
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */


  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  float noise_ax = 9;
  float noise_ay = 9;

  // state transition matrix
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // process covariance matrix
  ekf_.Q_ <<
      dt_4/4*noise_ax,               0, dt_3/2*noise_ax,               0,
      0              , dt_4/4*noise_ay,               0, dt_3/2*noise_ay,
      dt_3/2*noise_ax,               0,   dt_2*noise_ax,               0,
      0              , dt_3/2*noise_ay,               0,   dt_2*noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates

      // TODO
      // measurement matrix
      //     ekf_.H_ = MatrixXd(3, 4);
      // measurement covariance matrix
      //     ekf_.R_ = MatrixXd(3, 3);

  } else {
    // Laser updates
      // measurement matrix
      //     ekf_.H_ = MatrixXd(2, 4);
      // measurement covariance matrix
      //     ekf_.R_ = MatrixXd(2, 2);

      ekf_.Update(measurement_pack.raw_measurements_); // Lesson5-13 code
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
