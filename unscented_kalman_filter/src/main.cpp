#include <iostream>
#include "Eigen/Dense"
#include "ukf.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;


int main() {

  UKF ukf;  // creating a UKF instance


  MatrixXd Xsig = MatrixXd(5, 11);
  ukf.GenerateSigmaPoints(&Xsig);
  std::cout << "\n Xsig = " << std::endl << Xsig << std::endl;

  MatrixXd Xsig_aug = MatrixXd(7, 15);
  ukf.AugmentedSigmaPoints(&Xsig_aug);
  std::cout << "\n Xsig_aug = " << std::endl << Xsig_aug << std::endl;

  MatrixXd Xsig_pred = MatrixXd(15, 5);
  ukf.SigmaPointPrediction(&Xsig_pred);
  std::cout << "\n Xsig_pred = " << std::endl << Xsig_pred << std::endl;


  VectorXd x_pred = VectorXd(5);
  MatrixXd P_pred = MatrixXd(5, 5);
  ukf.PredictMeanAndCovariance(&x_pred, &P_pred);
  std::cout << "\n x_pred (predicted state) = " << std::endl << x_pred << std::endl;
  std::cout << "\n P_pred (predicted covariance matrix)= " << std::endl << P_pred << std::endl;

  VectorXd z_out = VectorXd(3);
  MatrixXd S_out = MatrixXd(3, 3);
  ukf.PredictRadarMeasurement(&z_out, &S_out);
  std::cout << "\n z_out = " << std::endl << z_out << std::endl;
  std::cout << "\n S_out = " << std::endl << S_out << std::endl;


  VectorXd x_out = VectorXd(5);
  MatrixXd P_out = MatrixXd(5, 5);
  ukf.UpdateState(&x_out, &P_out);
  std::cout << "\n x_out (updated state) = " << std::endl << x_out << std::endl;
  std::cout << "\n P_out (updated state covariance) = " << std::endl << P_out << std::endl;


  return 0;
}
