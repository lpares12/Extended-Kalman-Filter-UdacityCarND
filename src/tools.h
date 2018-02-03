#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  /**
   * Constructor.
   */
  Tools();

  /**
   * Destructor.
   */
  virtual ~Tools();

  /**
   * A helper method to calculate RMSE.
   */
  VectorXd calculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
   * A helper method to calculate Jacobians.
   */
  MatrixXd calculateJacobian(const VectorXd& x_state);

  /**
   * A helper method to convert Cartesian vector to polar.
   * @param polar_vector a vector in polar format (rho,phi,rho_d)
   * @return polar_vector in cartesian format (px,py,vx,vy)
   */
  static VectorXd convertToCartesian(const VectorXd& polar_vector);

  /**
   * A helper method to convert polar vector to cartesian.
   * @param cartesian_vector a vector containing the cartesian coordinates (px,py,vx,vy)
   * @return cartesian_vector in polar format (rho, phi, rho_d)
   */
  static VectorXd convertToPolar(const VectorXd& cartesian_vector);

};

#endif /* TOOLS_H_ */
