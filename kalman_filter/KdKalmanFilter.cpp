/** 
 * Multi-dimensional Kalman Filter
 */

#include <iostream>
#include <vector>
#include "Dense"

using std::cout;
using std::endl;
using std::vector;
using Eigen::VectorXd;
using Eigen::MatrixXd;

// Kalman Filter variables
VectorXd x;	// object state
MatrixXd P;	// object covariance matrix
VectorXd u;	// external motion
MatrixXd F; // state transition matrix
MatrixXd H;	// measurement matrix
MatrixXd R;	// measurement covariance matrix
MatrixXd I; // Identity matrix
MatrixXd Q;	// process covariance matrix

vector<VectorXd> measurements;
void filter(VectorXd &x, MatrixXd &P);


int main() {

  // KF with 1D motion
  x = VectorXd(2);
  x << 0, 0;

  P = MatrixXd(2, 2);
  P << 1000, 0, 0, 1000;

  u = VectorXd(2);
  u << 0, 0;

  F = MatrixXd(2, 2);
  F << 1, 1, 0, 1;

  H = MatrixXd(1, 2);
  H << 1, 0;

  R = MatrixXd(1, 1);
  R << 1;

  I = MatrixXd::Identity(2, 2);

  Q = MatrixXd(2, 2);
  Q << 0, 0, 0, 0;

  // create a list of measurements
  VectorXd single_meas(1);
  single_meas << 1;
  measurements.push_back(single_meas);
  single_meas << 2;
  measurements.push_back(single_meas);
  single_meas << 3;
  measurements.push_back(single_meas);

  // call Kalman filter algorithm
  kalman_filter(x, P);

  return 0;
}


void kalman_filter(VectorXd &x, MatrixXd &P) {

  for (unsigned int n = 0; n < measurements.size(); ++n) {

    VectorXd z = measurements[n];
    
    // KF masurement update step
    MatrixXd y = z - H * x;  // error calculation given new measurement z
    MatrixXd S = H * P * H.transpose() + R;
    MatrixXd K = P * H.transpose() * S.inverse();  // Kalman gain
    
    // new state
    x = x + K * y;
    P = (I - (K * H)) * P;  // covariance
		
    // KF prediction step
    // Question: why do we not use the process noise in the state prediction function, even though the state transition equation has one?, i.e. why u = 0?
    // Answer: mean is zero and its covariance matrix is usually noted by Q * N(0,Q)Q∗N(0,Q). 
    // The first equation only predicts the mean state. As the mean value of the noise is zero, it does not directly affect the predicted state. 
    // However, we can see that the noise covariance QQ is added here to the state covariance prediction so that the state uncertainty always increases through the process noise
    // State vector only tracks position and velocity, so we are modelling acceleration as a random noise
    x = F * x + u;  // u: process/ motion noise, i.e. uncertainty in the object's position when predicting location. Prediction assumes constant velocity, so (de)acceleration creates noise. Process noise depends on both the elapsed time and the uncertainty of acceleration
    P = F * P * F.transpose() + Q;  // state covariance update  // Q is a function of delta t, among other variables. As more time passes, we become more uncertain about our position and velocity

    cout << "x=" << endl <<  x << endl;
    cout << "P=" << endl <<  P << endl;
  }
}
