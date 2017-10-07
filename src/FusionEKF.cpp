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
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  H_laser_ << 1, 0 ,0, 0, 
             0, 1, 0, 0;

  ekf_.F_ = MatrixXd(4, 4); //4x4 matrix (state Transition)
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  ekf_.P_ = MatrixXd(4,4); // 4x4 matrix
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

  //set the acceleration noise components
  float noise_ax = 9;
  float noise_ay = 9;
  
  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
}

/**
*Destructor
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
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      cout << "Init RADAR";
      float ro = measurement_pack.raw_measurements_(0);
      float theta = measurement_pack.raw_measurements_(1);
      float ro_dot = measurement_pack.raw_measurements_(2);
      ekf_.x_(0) = ro*cos(theta);
      ekf_.x_(1) = ro*sin(theta);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      cout << "Init LASER";
      ekf_.x_(0) = measurement_pack.raw_measurements_(0); //x
      ekf_.x_(1) = measurement_pack.raw_measurements_(1); //y
    }
    
    ekf_.F_ << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
   previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
   cout << "Init complete";
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

//  if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
{

  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; //dt in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  cout << "dt : " << dt << endl;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  float noise_ax = 9.0;
  float noise_ay = 9.0;

  //Modify the F matrix so that the time in integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  //set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
		0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
		dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
		0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  cout<< "Predict Input" << endl;
  //cout<<"F_:" << ekf_.F_ << endl;
  //cout<<"Q_:" << ekf_.Q_ << endl;
  //cout<<"P_:" << ekf_.P_ << endl;
  //cout << "x input: " << ekf_.x_ << endl;

  //ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

    ekf_.Predict();
    cout << "Radar updates" <<endl;
    VectorXd x_state = ekf_.x_;
    cout << "measurement z: " << measurement_pack.raw_measurements_ << endl;
    Hj_ = tools.CalculateJacobian(x_state);

    cout << "Hj : " << Hj_ <<endl;

    ekf_.H_ = Hj_;
    cout << "Hj_: " << ekf_.H_ << endl;
    ekf_.R_ = R_radar_;

    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  }
  else {

    ekf_.Predict();
    cout << "Laser updates" << endl;
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;

    cout << "measurement z: " << measurement_pack.raw_measurements_ << endl;
    cout << "H_ : " << ekf_.H_ << endl;

    VectorXd z_state = VectorXd(2);
    z_state << 0.0, 0.0;
    z_state(0) = measurement_pack.raw_measurements_(0);
    z_state(1) = measurement_pack.raw_measurements_(1);

    ekf_.Update(z_state);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
}
