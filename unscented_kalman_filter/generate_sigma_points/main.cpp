#include <iostream>
#include "Dense"
#include "ukf.h"

using Eigen::MatrixXd;

int main() {

  UKF ukf;  // creating a UKF instance

  MatrixXd Xsig = MatrixXd(5, 11);
  ukf.GenerateSigmaPoints(&Xsig);

  std::cout << "Xsig = " << std::endl << Xsig << std::endl;

  return 0;
}
