#include <iostream>
#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

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
  cout << "Prediction: " << endl;
  cout << "x_ in: " << x_ << endl;
  cout << "P_ in: " << P_ << endl;
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ *Ft + Q_;
  cout << "x_out: " << x_ << endl;
  cout << "P_out: " << P_ << endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  
  cout << "laser update in x_: " << x_ << endl;

  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_*P_*Ht+R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_*Ht*Si;


  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

 //new state
  x_ = x_ + (K * y);
  P_ = (I- K * H_) * P_;

  // KF Prediction step
  //x = F * x + u;
  //MatrixXd Ft = F.transpose();
  //P = F * P * Ft + Q;
  cout << "Laser Update " << endl;
  cout << "H_: " << H_ << endl; 
  cout << "y: " << y << endl;
  cout << "z: " << z << endl;
  cout << "S: " << S << endl;
  cout << "K: " << K << endl;
  cout << "x_: " << x_ << endl;
  cout << "P_: " << P_ << endl;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float x = x_(0);
  float y1 = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  // Check division by zero
  if (fabs(x*x+ y1*y1) < 0.0001) {
    std::cout << "UpdateEKF(): CalculateJacobian() - Error - Division by Zero" << std::endl;
    return;
  }


  float rho = sqrt(x*x+y1*y1);
  float theta = atan2(y1,x);
  float ro_dot = (x*vx+y1*vy)/rho;
  VectorXd z_pred = VectorXd(3);
  z_pred << rho, theta, ro_dot;

  VectorXd y = z - z_pred;

  y(1) = atan2(sin(y(1)),cos(y(1)));
  const double pi = 3.14159265358979323846;

  if (y(1) < -pi) {
    y(1) = y(1) + 2*pi;

  }
  else if (y(1) > pi) {
    y(1) = y(1) - 2*pi;
  }


  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K  = P_ * Ht * Si;

  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  // new state
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;

  // EKF Prediction Step

  cout << "Radar Update EKF: " << endl;
  cout << "z : " << z << endl;
  cout << "z_pred: " << z_pred << endl;
  cout << "y: " << y << endl;
  cout << "S: " << S << endl;
  cout << "K: " << K << endl;
  cout << "x_: " << x_ << endl;
  cout << " P_: " << P_ << endl;
}
