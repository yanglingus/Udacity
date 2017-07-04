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
  H_laser_ << 1, 0, 0 , 0,
              0, 1, 0, 0;

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  Hj_ << 1, 1, 0, 0,
	 1, 1, 0, 0,
         1, 1, 1, 1;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

   
   // state convariance maxtrix P
   ekf_.P_ = MatrixXd(4,4);
   ekf_.P_ << 1, 0,0,0,
	      0,1,0,0,
              0,0,1000,0,
              0, 0, 0, 1000;


   //the initial transition matrix 
   ekf_.F_ = MatrixXd(4,4);
   ekf_.F_ << 1, 0, 1, 0,
              0, 1, 0, 1,
              0, 0, 1, 0,
              0, 0, 0, 1;

   noise_ax = 9.0;
   noise_ay = 9.0;
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
    // cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
 	float rho_measured = measurement_pack.raw_measurements_(0); //range
	float phi_measured = measurement_pack.raw_measurements_(1); //bearing
 	float rhodot_measured = measurement_pack.raw_measurements_(2); //speed of the rho


	//coordinates conversion from polar to catesian
	float x = rho_measured * cos(phi_measured);
	float y = rho_measured * sin(phi_measured);
	float vx = rhodot_measured * cos(phi_measured);
	float vy = rhodot_measured * sin(phi_measured);
    
        ekf_.x_ << x, y, vx, vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      //No velocities from the first measure of LIDAR
      ekf_.x_ << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1), 5.19994,0;
    }

    if( (fabs(ekf_.x_(0)) < 0.0001) and (fabs(ekf_.x_(1)) < 0.0001)) {
        ekf_.x_(0) = 0.0001;
        ekf_.x_(1) = 0.0001;
     }

    ekf_.P_ = MatrixXd(4,4);
    ekf_.P_ << 1, 0,0,0,
	      0,1,0,0,
              0,0,1000,0,
              0, 0, 0, 1000;



    // done initializing, no need to predict or update
    is_initialized_ = true;

    cout << "EKF init: "<< ekf_.x_ << endl;
  
    //set the timestamp
    previous_timestamp_ = measurement_pack.timestamp_;
    // cout << "previous_timestamp: "<< previous_timestamp_ << endl;

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
   
  //Calculate the timestep between measurements in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0; 
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1, 0, dt, 0,
              0, 1, 0, dt,
              0, 0, 1, 0,
              0, 0, 0, 1;
  float dt_2 = dt * dt;
  float dt_3 =  dt_2 * dt;
  float dt_4 = dt_3 * dt;
  float dt_5 = dt_4 / 4;
  float dt_6 = dt_3 / 2;



  //set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4,4);
  ekf_.Q_ << dt_5 * noise_ax, 0, dt_6 * noise_ax, 0, 
  		0, dt_5 * noise_ay, 0, dt_6 * noise_ay,  
  		dt_6 * noise_ax, 0, dt_2 * noise_ax, 0, 
  		0, dt_6 * noise_ay, 0, dt_2 * noise_ay;
   

  //cout << "EKF F  : "<< ekf_.F_ << endl;
  //cout << "EKF Q : "<< ekf_.Q_ << endl;
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
    // Use Jacobian instead of H
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
