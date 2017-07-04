#include "kalman_filter.h"

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
  //std::cout << "In Predict"  << std::endl;
  //std::cout << "x_" << x_ << std::endl;
  //std::cout << "F_" << F_ << std::endl;
  x_ =  F_ * x_;
  MatrixXd Ft = F_.transpose();
  //std::cout << "Ft" << Ft << std::endl;
  //std::cout << "P_" << P_ << std::endl;
  P_ = F_ * P_* Ft + Q_;
  //std::cout << "after Predict"  << std::endl;
  //std::cout << "x_" << x_ << std::endl;
  //std::cout << "P_" << P_ << std::endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  std::cout << "x_" << x_ << std::endl;
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  //std::cout << "In Update "  << std::endl;
  //std::cout << "z" << z << std::endl;
  //std::cout << "Before KF x_" << x_ << std::endl;
  KF(y);
  //std::cout << "after KF x_" << x_ << std::endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  //Recalculate x object state to rho, theta, rho_dot coordinates

  double rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  double theta = atan2(x_(1),x_(0));
  double rho_dot = 0;
  if(fabs(rho) >= 0.0001)
  	rho_dot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;

  VectorXd h = VectorXd(3); 

  h << rho, theta, rho_dot;

  VectorXd y = z - h;

  //Below adjustment is crucial, atans returns value between -pi and pi. 
  //When calculated the measurements of y=z-h, I encountered y(1) value > 6. 
  //We need this adjustments to make things working. 
  // to move the y(1) value between (-pi, pi)
  // I noticed the data fluctation when bikes moves from second quardant to 3rd quardant,
  // I got this huge Vertical (Y) direction change and speed change
  
  if(y(1) > 3.14159 ) {
	y(1) = y(1) - 2*3.14159;
  }
  if(y(1) < -3.14159) {
	y(1) = y(1) + 2*3.14159;
  }
  //std::cout << "In UpdateEKF "  << std::endl;
  //std::cout << "z" << z << std::endl;
  //std::cout << "calculated h" << h << std::endl;
  //std::cout << "Before KF x_" << x_ << std::endl;
  KF(y);
  //std::cout << "after KF x_" << x_ << std::endl;
}


//Univeral update Kalman Filter steps.  
void KalmanFilter::KF(const VectorXd &y){
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;
  //std::cout << "In KF"  << std::endl;
  //std::cout << "y: "  << y << std::endl;
  //std::cout << "K: "  << K << std::endl;

  //New State
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
