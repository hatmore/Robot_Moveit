#pragma once

#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <cmath>

namespace mpc_position_controller
{

struct MPCConfig
{
  int num_joints = 7;
  int horizon = 10;
  double dt = 0.002;  // 500 Hz

  Eigen::VectorXd q_weights;   // position tracking (n)
  Eigen::VectorXd v_weights;   // velocity tracking (n)
  Eigen::VectorXd r_weights;   // control effort (n)

  Eigen::VectorXd q_min;       // joint position lower limits (n)
  Eigen::VectorXd q_max;       // joint position upper limits (n)
  Eigen::VectorXd v_max;       // velocity limits (n)
  Eigen::VectorXd a_max;       // acceleration limits (n)

  double constraint_margin = 0.05;       // safety margin for position limits [rad]
  double terminal_cost_multiplier = 3.0; // scale terminal stage cost
};

struct MPCState
{
  Eigen::VectorXd q;      // current positions (n)
  Eigen::VectorXd q_dot;  // current velocities (n)
};

struct MPCReference
{
  std::vector<Eigen::VectorXd> q_ref;      // N x (n)
  std::vector<Eigen::VectorXd> q_dot_ref;  // N x (n)
};

class MPCSolver
{
public:
  explicit MPCSolver(const MPCConfig& config);

  void updateConfig(const MPCConfig& config);

  /**
   * Solve one MPC step.
   * @param state   Current joint state
   * @param ref     Reference trajectory (N steps)
   * @param u_opt   [out] Optimal acceleration command (n)
   * @param q_cmd   [out] Position command to send to hardware (n)
   * @return true on success
   */
  bool solve(const MPCState& state, const MPCReference& ref,
             Eigen::VectorXd& u_opt, Eigen::VectorXd& q_cmd);

  const std::vector<Eigen::VectorXd>& getPredictedPositions() const
  {
    return predicted_positions_;
  }

private:
  void buildPredictionMatrices();
  void buildCostMatrices();
  void solveUnconstrained(const Eigen::VectorXd& x0,
                          const Eigen::VectorXd& x_ref_stacked,
                          Eigen::VectorXd& U);
  void enforceConstraints(const MPCState& state, Eigen::VectorXd& u0);

  MPCConfig config_;
  int n_;  // num_joints
  int N_;  // horizon

  // Discrete dynamics: x_{k+1} = A_d * x_k + B_d * u_k
  Eigen::MatrixXd A_d_;  // (2n x 2n)
  Eigen::MatrixXd B_d_;  // (2n x n)

  // Condensed prediction matrices: X = S_x * x_0 + S_u * U
  Eigen::MatrixXd S_x_;  // (2nN x 2n)
  Eigen::MatrixXd S_u_;  // (2nN x nN)

  // Pre-computed QP: min 0.5 U^T H U + g^T U
  Eigen::MatrixXd H_;                     // (nN x nN) Hessian
  Eigen::LLT<Eigen::MatrixXd> H_llt_;    // Cholesky factorization
  Eigen::MatrixXd SuTQ_;                  // S_u^T * Q_big  (nN x 2nN)

  // Pre-allocated workspace
  Eigen::VectorXd x0_vec_;           // (2n)
  Eigen::VectorXd x_ref_stacked_;    // (2nN)
  Eigen::VectorXd U_opt_;            // (nN)
  Eigen::VectorXd g_;                // (nN) gradient

  std::vector<Eigen::VectorXd> predicted_positions_;
};

}  // namespace mpc_position_controller
