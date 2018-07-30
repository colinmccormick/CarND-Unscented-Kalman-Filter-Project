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

  // declare rmse
  VectorXd rmse(4);
  rmse.fill(0.0);

  // check for inconsistencies
  if ( (estimations.size() != ground_truth.size()) || estimations.size() == 0 ) {
  	std::cout << "Error on size of estimations or ground truth for RMSE." << std::endl;
    return rmse;
  }

  // calculate residuals, square, and accumulate
  for (int i=0; i < estimations.size(); i++) {
  	VectorXd residuals = estimations[i] - ground_truth[i];
  	residuals = residuals.array() * residuals.array();
  	rmse += residuals;
  }

  // calculate rmse
  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;

}