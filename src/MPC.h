#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();
  double limit;

  virtual ~MPC();

  Eigen::MatrixXd Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, double limit);
  Eigen::MatrixXd Model(Eigen::VectorXd state, Eigen::VectorXd actuators, int Num, double dt_);
  Eigen::MatrixXd Transform(vector<double> ptsx, vector<double> ptsy, Eigen::VectorXd state);


};



#endif /* MPC_H */
