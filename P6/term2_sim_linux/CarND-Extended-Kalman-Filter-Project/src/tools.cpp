#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  
  VectorXd rmse(4);
  rmse << 0,0,0,0;


  if((estimations.size() == 0) || (ground_truth.size() == 0)){
  	cout << "size wrong" << endl;
	return rmse;
  }
  if(estimations.size()  != ground_truth.size()) {
	cout << "size wrong" << endl;
	return rmse;
  }

  for(int i=0; i< estimations.size(); i++){
	//check size

	VectorXd residual = estimations[i] - ground_truth[i];

	//cofficient multiplication
	residual = residual.array() * residual.array();
	rmse = rmse + residual;
	}

   //cacluate the mean
   rmse = rmse/estimations.size();

   //calculate the squared root
   rmse = rmse.array().sqrt();

   //return
   return rmse; 
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);

  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);


  //pre-compute a set of terms 
  float c1 = px*px + py*py;

  //check division by 0
  if(fabs(c1) < 0.0001){
	cout << "CalculateJacobian() - Error, Divided by 0"<< endl;
        c1 = 0.0001;
  }

  float c2 =  sqrt(c1);
  float c3 = c1*c2;

  //compute the Jocobian Maxtrix
  Hj << (px/c2), (py/c2), 0, 0, 
        -(py/c1), (px/c1), 0, 0, 
        py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  //cout << "In CalculateJacobian(): Hj "<<  Hj << endl;
  return Hj;
}
