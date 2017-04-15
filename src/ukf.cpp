#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  //std_a_ = 30;
  std_a_ = 0.3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  //std_yawdd_ = 30;
  std_yawdd_ = 0.5;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  is_initialized_ = false;
  previous_timestamp_ = 0;
  n_x_ = 5;
  n_aug_ = 7;
  n_z_ = 3;
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  //define spreading parameter
  lambda_ = 3 - n_aug_;  
  NIS_laser_ = 0;
  NIS_radar_ = 0;
}

UKF::~UKF() {}

bool UKF::Init(const MeasurementPackage &measurement_pack) {
  /**
  Initialize the filter with the first usable measurement
  */

    float px = 0;
    float py = 0;
    float vel_abs = 0.1;
    float yaw_angle = 0.1;
    float yaw_rate = 0.1;

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    /**
    Convert radar from polar to cartesian coordinates and initialize state.
    */

    float ro = measurement_pack.raw_measurements_(0);
    float phi = measurement_pack.raw_measurements_(1);
    float ro_dot = measurement_pack.raw_measurements_(2);
    px = ro * cos(phi);
    py = ro * sin(phi);
    float vx = ro_dot * cos(phi);
    float vy = ro_dot * sin(phi);
    vel_abs = sqrt(vx*vx + vy*vy);
    if (vx != 0) {
      yaw_angle = vy / vx;
    }
    yaw_rate = 0;
    //state covariance matrix P initialization
    //have relatively high and yet reasonable confidence in yaw_rate, and yaw_acc
    //to avoid divergence issue
    // P_ << 1, 0, 0, 0, 0,
    //  0, 1, 0, 0, 0,
    //  0, 0, 1, 0, 0,
    //  0, 0, 0, 1, 0,
    //  0, 0, 0, 0, 1;
  	P_ << 2, 0, 0, 0, 0,
  		0, 4, 0, 0, 0,
  		0, 0, 1, 0, 0,
  		0, 0, 0, 0.5, 0,
  		0, 0, 0, 0, 0.5;

  }
  else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    /**
    Initialize state.
    */
    px = measurement_pack.raw_measurements_[0];
    py = measurement_pack.raw_measurements_[1];
    //state covariance matrix P initialization
    //have relatively high and yet reasonable confidence in yaw_rate, and yaw_acc
    //to avoid divergence issue
    // P_ << 1, 0, 0, 0, 0,
    //  0, 1, 0, 0, 0,
    //  0, 0, 1, 0, 0,
    //  0, 0, 0, 1, 0,
    //  0, 0, 0, 0, 1;
  	P_ << 1, 0, 0, 0, 0,
  		0, 1, 0, 0, 0,
  		0, 0, 2, 0, 0,
  		0, 0, 0, 3, 0,
  		0, 0, 0, 0, 1;
     }
  previous_timestamp_ = measurement_pack.timestamp_;

  x_ << px, py, vel_abs, yaw_angle, yaw_rate;

  // done initializing, no need to predict or update
  if (px == 0 && py == 0) {
    //will start true kalman state initialization till records whose px/py is not zero arrives
    return false;
  }
  else {
    return true;
  }

}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

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
		if (Init(meas_package)) {
		  is_initialized_ = true;
		  // if (Tools::trace_tag == true) {
			 //  Tools::traceStream << "UKF::Init(const MeasurementPackage &measurement_pack)" << endl;
			 //  Tools::traceStream << "x_" << endl << x_ << endl;
			 //  Tools::traceStream << "P_" << endl << P_ << endl;
			 //  Tools::traceStream << endl;
		  // }
		}    
		return;
  }  

  /*****************************************************************************
  *  Prediction
  ****************************************************************************/
  //compute the elapsed time between the current and previous measurements
  double delta_t = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0; //dt - expressed in seconds
  previous_timestamp_ = meas_package.timestamp_;

  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  while (delta_t > 0.1) {
	  //Here we try to descrease the delta time between prediction steps, 
	  //(1 second in the case of dataset 2 is too large), so that using 
    //small yaw angle gap (using 0.05 in the case.) for the prediction step
	  Prediction(Tools::yaw_angle_gap);
	  delta_t -= Tools::yaw_angle_gap;
  }

  Prediction(delta_t); 


  /*****************************************************************************
    *  Update
    ****************************************************************************/  
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(meas_package);
  }
  else {
    UpdateLidar(meas_package);
  } 


  NormalizeAngle(x_(3));  
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

    //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  AugmentedSigmaPoints(Xsig_aug);
  // std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;  

  //create matrix with predicted sigma points as columns
  SigmaPointPrediction(Xsig_aug,delta_t);
  // std::cout << "Xsig_pred_ = " << std::endl << Xsig_pred_ << std::endl;  


  //compute x_ and P_ for the prediction step
  PredictMeanAndCovariance(Xsig_pred_);  
  // std::cout << "x_ = " << std::endl << x_ << std::endl;    
  // std::cout << "P_ = " << std::endl << P_ << std::endl;  
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  const VectorXd &z = meas_package.raw_measurements_;
  MatrixXd R(2, 2);
  R << std_laspx_*std_laspx_, 0,
    0, std_laspy_ * std_laspy_;

  MatrixXd H(2, 5);
  H << 1, 0, 0, 0, 0,
    0, 1, 0, 0, 0;
  VectorXd z_pred = H * x_;
  VectorXd z_diff = z - z_pred;
  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P_ * Ht + R;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * z_diff);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H) * P_;
  //update NIS
  NIS_laser_ = ComputeNIS(z_pred, S, z);


}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  VectorXd z_pred = VectorXd(n_z_);
  MatrixXd S = MatrixXd(n_z_, n_z_);
  MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);
  PredictRadarMeasurement(z_pred, S, Zsig, Xsig_pred_);
  UpdateRadarState(meas_package, Xsig_pred_, Zsig, z_pred, S); 

}

void UKF::AugmentedSigmaPoints(MatrixXd &Xsig_aug) {

  //create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;
  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5, 5) = P_;
  P_aug(5, 5) = std_a_*std_a_;
  P_aug(6, 6) = std_yawdd_*std_yawdd_;
  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);

  }

}

void UKF::SigmaPointPrediction(const MatrixXd &Xsig_aug, double delta_t) {

  //predict sigma points
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    //extract values for better readability
    double p_x = Xsig_aug(0, i);
    double p_y = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yawd = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
      px_p = p_x + v / yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
      py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd*delta_t));
    }
    else {
      px_p = p_x + v*delta_t*cos(yaw);
      py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;
  	NormalizeAngle(yaw_p);

    //write predicted sigma points into right column
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;



  }

}

void UKF::PredictMeanAndCovariance(const MatrixXd &Xsig_pred) {


  //create vector for weights
  VectorXd weights = VectorXd(2 * n_aug_ + 1);

  //create vector for predicted state
  VectorXd x = VectorXd(n_x_);

  //create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);


  // set weights
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights(0) = weight_0;
  for (int i = 1; i<2 * n_aug_ + 1; i++) {  //2n+1 weights
    double weight = 0.5 / (n_aug_ + lambda_);
    weights(i) = weight;
  }

  //predicted state mean
  x.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x = x + weights(i) * Xsig_pred.col(i);
  }

  //predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;

    //angle normalization
    NormalizeAngle(x_diff(3));

    P = P + weights(i) * x_diff * x_diff.transpose();

  }

  //write result
  x_ = x;
  P_ = P;
  NormalizeAngle(x_(3));

}

void UKF::NormalizeAngle(double &angle) {
  while (angle> M_PI) {
    angle-=2.*M_PI;
  }
  while (angle<-M_PI) {
    angle+=2.*M_PI;
  }

}


void UKF::PredictRadarMeasurement(VectorXd &z_pred, MatrixXd &S, MatrixXd &Zsig, const MatrixXd &Xsig_pred) {

  //set vector for weights
  VectorXd weights = VectorXd(2 * n_aug_ + 1);
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights(0) = weight_0;
  for (int i = 1; i < 2 * n_aug_ + 1; i++) {
    double weight = 0.5 / (n_aug_ + lambda_);
    weights(i) = weight;
  }

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

                        // extract values for better readibility
    double p_x = Xsig_pred(0, i);
    double p_y = Xsig_pred(1, i);
    double v = Xsig_pred(2, i);
    double yaw = Xsig_pred(3, i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0, i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1, i) = atan2(p_y, p_x);                                 //phi
    Zsig(2, i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  //mean predicted measurement

  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    z_pred = z_pred + weights(i) * Zsig.col(i);
  }

  //measurement covariance matrix S

  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
                        //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization

    NormalizeAngle(z_diff(1));

    S = S + weights(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z_, n_z_);
  R << std_radr_*std_radr_, 0, 0,
  0, std_radphi_*std_radphi_, 0,
  0, 0, std_radrd_*std_radrd_;
  S = S + R;

}

void UKF::UpdateRadarState(const MeasurementPackage &meas_package, const MatrixXd &Xsig_pred, const MatrixXd &Zsig,
  const VectorXd &z_pred, const MatrixXd &S) {

  //set vector for weights
  VectorXd weights = VectorXd(2 * n_aug_ + 1);
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights(0) = weight_0;
  for (int i = 1; i<2 * n_aug_ + 1; i++) {  //2n+1 weights
    double weight = 0.5 / (n_aug_ + lambda_);
    weights(i) = weight;
  }

  //create example vector for incoming radar measurement
  const VectorXd &z = meas_package.raw_measurements_;


  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);


  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

                        //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    NormalizeAngle(z_diff(1));

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x_;
    //angle normalization
    NormalizeAngle(x_diff(3));

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization

  NormalizeAngle(z_diff(1));

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  //update NIS
  NIS_radar_ = ComputeNIS(z_pred, S, z);

}

double UKF::ComputeNIS(const VectorXd &z_pred, const MatrixXd &S, const VectorXd &z) {
  double nis = 0;
  VectorXd y = z - z_pred;
  MatrixXd Si = S.inverse();
  int mea_size = z.size();

  Eigen::Map<MatrixXd> yt_matrix(y.data(), 1,mea_size);
  Eigen::Map<MatrixXd> y_matrix(y.data(), mea_size,1);

  MatrixXd temp = yt_matrix * Si * y_matrix;
  nis = temp(0,0);
  return nis;

}
