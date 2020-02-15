#include "Dense"
#include "ukf.h"

using Eigen::MatrixXd;

int main() {

  // Create a UKF instance
  UKF ukf;

  MatrixXd Xsig_pred = MatrixXd(15, 5);
  ukf.SigmaPointPrediction(&Xsig_pred);

  return 0;
}
