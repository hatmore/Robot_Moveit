#include "mpc_position_controller/mpc_solver.hpp"

namespace mpc_position_controller
{

MPCSolver::MPCSolver(const MPCConfig& config)
  : config_(config)
  , n_(config.num_joints)
  , N_(config.horizon)
{
  // Pre-allocate workspace
  x0_vec_.resize(2 * n_);
  x_ref_stacked_.resize(2 * n_ * N_);
  U_opt_.resize(n_ * N_);
  g_.resize(n_ * N_);

  buildPredictionMatrices();
  buildCostMatrices();
}

void MPCSolver::updateConfig(const MPCConfig& config)
{
  config_ = config;
  n_ = config.num_joints;
  N_ = config.horizon;

  x0_vec_.resize(2 * n_);
  x_ref_stacked_.resize(2 * n_ * N_);
  U_opt_.resize(n_ * N_);
  g_.resize(n_ * N_);

  buildPredictionMatrices();
  buildCostMatrices();
}

void MPCSolver::buildPredictionMatrices()
{
  const double dt = config_.dt;
  const int n = n_;
  const int N = N_;

  // A_d = [I   dt*I]
  //       [0     I ]
  A_d_ = Eigen::MatrixXd::Identity(2 * n, 2 * n);
  A_d_.topRightCorner(n, n) = dt * Eigen::MatrixXd::Identity(n, n);

  // B_d = [0.5*dt^2 * I]
  //       [   dt * I    ]
  B_d_ = Eigen::MatrixXd::Zero(2 * n, n);
  B_d_.topRows(n) = 0.5 * dt * dt * Eigen::MatrixXd::Identity(n, n);
  B_d_.bottomRows(n) = dt * Eigen::MatrixXd::Identity(n, n);

  // S_x: [A_d; A_d^2; ...; A_d^N]  size (2nN x 2n)
  S_x_.resize(2 * n * N, 2 * n);
  Eigen::MatrixXd A_pow = A_d_;
  for (int k = 0; k < N; ++k) {
    S_x_.block(2 * n * k, 0, 2 * n, 2 * n) = A_pow;
    if (k < N - 1) {
      A_pow = A_pow * A_d_;
    }
  }

  // S_u: lower-triangular block Toeplitz  size (2nN x nN)
  // S_u[i,j] = A_d^{i-j} * B_d  for i >= j, else 0
  S_u_ = Eigen::MatrixXd::Zero(2 * n * N, n * N);

  // Precompute A_d^k * B_d for k = 0..N-1
  std::vector<Eigen::MatrixXd> AdkB(N);
  AdkB[0] = B_d_;
  for (int k = 1; k < N; ++k) {
    AdkB[k] = A_d_ * AdkB[k - 1];
  }

  for (int row = 0; row < N; ++row) {
    for (int col = 0; col <= row; ++col) {
      S_u_.block(2 * n * row, n * col, 2 * n, n) = AdkB[row - col];
    }
  }
}

void MPCSolver::buildCostMatrices()
{
  const int n = n_;
  const int N = N_;

  // Q_bar = diag(q_weights, v_weights)  (2n)
  Eigen::VectorXd q_bar_diag(2 * n);
  q_bar_diag.head(n) = config_.q_weights;
  q_bar_diag.tail(n) = config_.v_weights;

  // Q_big diagonal (2nN): repeat Q_bar for each stage, terminal stage scaled
  Eigen::VectorXd Q_diag(2 * n * N);
  for (int k = 0; k < N; ++k) {
    double scale = (k == N - 1) ? config_.terminal_cost_multiplier : 1.0;
    Q_diag.segment(2 * n * k, 2 * n) = q_bar_diag * scale;
  }

  // R_big diagonal (nN)
  Eigen::VectorXd R_diag(n * N);
  for (int k = 0; k < N; ++k) {
    R_diag.segment(n * k, n) = config_.r_weights;
  }

  // H = S_u^T * Q_big * S_u + R_big
  // Q_big is diagonal, so Q_big * S_u = element-wise row scaling
  Eigen::MatrixXd QS_u = Q_diag.asDiagonal() * S_u_;  // (2nN x nN)
  H_ = S_u_.transpose() * QS_u;                        // (nN x nN)
  H_.diagonal() += R_diag;

  // Pre-compute S_u^T * Q_big for gradient: g = SuTQ_ * (S_x * x0 - X_ref)
  SuTQ_ = S_u_.transpose() * Q_diag.asDiagonal();      // (nN x 2nN)

  // Cholesky factorization (H is SPD)
  H_llt_.compute(H_);
}

void MPCSolver::solveUnconstrained(
    const Eigen::VectorXd& x0,
    const Eigen::VectorXd& x_ref_stacked,
    Eigen::VectorXd& U)
{
  // g = SuTQ_ * (S_x_ * x0 - x_ref_stacked)
  g_.noalias() = SuTQ_ * (S_x_ * x0 - x_ref_stacked);
  // U* = H^{-1} * (-g)
  U = H_llt_.solve(-g_);
}

void MPCSolver::enforceConstraints(const MPCState& state, Eigen::VectorXd& u0)
{
  const double dt = config_.dt;
  const double margin = config_.constraint_margin;

  for (int j = 0; j < n_; ++j) {
    // 1. Clamp acceleration to limits
    u0(j) = std::clamp(u0(j), -config_.a_max(j), config_.a_max(j));

    // 2. Check velocity after applying u0
    double v_next = state.q_dot(j) + u0(j) * dt;
    if (std::abs(v_next) > config_.v_max(j)) {
      double v_clamped = std::clamp(v_next, -config_.v_max(j), config_.v_max(j));
      u0(j) = (v_clamped - state.q_dot(j)) / dt;
    }

    // 3. Check position after applying u0
    double q_next = state.q(j) + state.q_dot(j) * dt + 0.5 * u0(j) * dt * dt;
    double q_lo = config_.q_min(j) + margin;
    double q_hi = config_.q_max(j) - margin;
    if (q_next < q_lo || q_next > q_hi) {
      double q_clamped = std::clamp(q_next, q_lo, q_hi);
      u0(j) = 2.0 * (q_clamped - state.q(j) - state.q_dot(j) * dt) / (dt * dt);
      // Re-clamp acceleration after position correction
      u0(j) = std::clamp(u0(j), -config_.a_max(j), config_.a_max(j));
    }
  }
}

bool MPCSolver::solve(
    const MPCState& state,
    const MPCReference& ref,
    Eigen::VectorXd& u_opt,
    Eigen::VectorXd& q_cmd)
{
  if (static_cast<int>(ref.q_ref.size()) < N_ ||
      static_cast<int>(ref.q_dot_ref.size()) < N_) {
    return false;
  }

  // Build x0 = [q; q_dot]
  x0_vec_.head(n_) = state.q;
  x0_vec_.tail(n_) = state.q_dot;

  // Build stacked reference: X_ref = [x_ref_1; x_ref_2; ...; x_ref_N]
  for (int k = 0; k < N_; ++k) {
    x_ref_stacked_.segment(2 * n_ * k, n_) = ref.q_ref[k];
    x_ref_stacked_.segment(2 * n_ * k + n_, n_) = ref.q_dot_ref[k];
  }

  // Solve unconstrained QP
  solveUnconstrained(x0_vec_, x_ref_stacked_, U_opt_);

  // Extract first control input
  u_opt = U_opt_.head(n_);

  // Enforce constraints on u_0
  enforceConstraints(state, u_opt);

  // Compute position command: q_cmd = q + v*dt + 0.5*a*dt^2
  const double dt = config_.dt;
  q_cmd = state.q + state.q_dot * dt + 0.5 * u_opt * (dt * dt);

  // Store predicted positions for diagnostics
  predicted_positions_.resize(N_);
  Eigen::VectorXd x_k = x0_vec_;
  Eigen::VectorXd u_k = u_opt;
  for (int k = 0; k < N_; ++k) {
    x_k = A_d_ * x_k + B_d_ * (k == 0 ? u_k : U_opt_.segment(n_ * k, n_));
    predicted_positions_[k] = x_k.head(n_);
  }

  return true;
}

}  // namespace mpc_position_controller
