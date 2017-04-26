//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   25.04.2017
//------------------------------------------------------------------------------

#include "kalman_filter.h"

using namespace Eigen;

//------------------------------------------------------------------------------
// Linear prediction
//------------------------------------------------------------------------------
void LinearKalmanPredictor::Predict(KalmanState &state, uint64_t dt) {
  (void) dt;
  state.x = F_ * state.x;
  state.P = F_ * state.P * F_.transpose() + Q_;
}

//------------------------------------------------------------------------------
// Initialize weights
//------------------------------------------------------------------------------
void UnscentedKalmanPredictor::InitializeWeights() {
  int    n_aug  = n_dims_ + nu_.size();
  double weight = 0.5/(n_aug+lambda_);
  weights_      = VectorXd(2*n_aug+1);
  weights_(0)   = lambda_/(lambda_+n_aug);
  for(int i = 1; i < 2*n_aug+1; ++i)
    weights_(i) = weight;
}

//------------------------------------------------------------------------------
// Calculate sigma points
//------------------------------------------------------------------------------
void UnscentedKalmanPredictor::CalculateSigmaPoints(
  Eigen::MatrixXd &sigma_points,
  KalmanState     &state) {

  int  n     = state.x.size() + nu_.size();
  auto x_aug = VectorXd(n);
  x_aug.head(state.x.size()) = state.x;
  x_aug.tail(nu_.size())     = nu_;

  auto P_aug = MatrixXd(n, n);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(state.x.size(), state.x.size()) = state.P;
  P_aug.bottomRightCorner(nu_.size(), nu_.size())     = Q_;

  MatrixXd L = P_aug.llt().matrixL();

  sigma_points.col(0) = x_aug;
  for(int i = 0; i< n; ++i) {
    sigma_points.col(i+1)   = x_aug + sqrt(lambda_+n) * L.col(i);
    sigma_points.col(i+1+n) = x_aug - sqrt(lambda_+n) * L.col(i);
  }
}

//------------------------------------------------------------------------------
// Unscented prediction
//------------------------------------------------------------------------------
void UnscentedKalmanPredictor::Predict(KalmanState &state, uint64_t dt) {
  //----------------------------------------------------------------------------
  // Calculate and transform the sigma points
  //----------------------------------------------------------------------------
  int n = state.x.size() + nu_.size();
  auto sigma_aug = MatrixXd(n, 2*n+1);
  CalculateSigmaPoints(sigma_aug, state);

  for(int i = 0; i < sigma_aug.cols(); ++i)
    state.sigma_points.col(i) = Transform(sigma_aug.col(i), dt);

  //----------------------------------------------------------------------------
  // Predict the state mean
  //----------------------------------------------------------------------------
  state.x.fill(0.0);
  for(int i = 0; i < 2*n+1; ++i)
    state.x += weights_(i) * state.sigma_points.col(i);
  Normalize(state.x);

  //----------------------------------------------------------------------------
  // Predict the state covariance
  //----------------------------------------------------------------------------
  state.P.fill(0.0);
  for(int i = 0; i < 2*n+1; ++i) {
    VectorXd x_diff = state.sigma_points.col(i) - state.x;
    Normalize(x_diff);
    state.P += weights_(i) * x_diff * x_diff.transpose();
  }

  state.nis = 0;
}

//------------------------------------------------------------------------------
// Linear update
//------------------------------------------------------------------------------
void LinearKalmanUpdater::Update(KalmanState    &state,
                                 const VectorXd &z) {
  VectorXd y  = z - H_ * state.x;
  MatrixXd S  = H_ * state.P * H_.transpose() + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K  = state.P * H_.transpose() * Si;

  state.x += K * y;
  size_t s = state.x.size();
  MatrixXd I = MatrixXd::Identity(s, s);
  state.P = (I - K * H_) * state.P;
  state.nis = y.transpose() * Si * y;
}

//------------------------------------------------------------------------------
// Extended update
//------------------------------------------------------------------------------
void ExtendedKalmanUpdater::Update(KalmanState    &state,
                                   const VectorXd &z) {
  (void) z;
  MatrixXd S  = Hj_ * state.P * Hj_.transpose() + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K  = state.P * Hj_.transpose() * Si;

  state.x = state.x + (K * y_);
  size_t s = state.x.size();
  MatrixXd I = MatrixXd::Identity(s, s);
  state.P = (I - K * Hj_) * state.P;
  state.nis = y_.transpose() * Si * y_;
}

//------------------------------------------------------------------------------
// Initialize weights
//------------------------------------------------------------------------------
void UnscentedKalmanUpdater::InitializeWeights() {
  double weight = 0.5/(n_aug_+lambda_);
  weights_      = VectorXd(2*n_aug_+1);
  weights_(0)   = lambda_/(lambda_+n_aug_);
  for(unsigned i = 1; i < 2*n_aug_+1; ++i)
    weights_(i) = weight;
}

//------------------------------------------------------------------------------
// Unscented update
//------------------------------------------------------------------------------
void UnscentedKalmanUpdater::Update(KalmanState    &state,
                                    const VectorXd &z) {
  //----------------------------------------------------------------------------
  // Project the sigma points onto the measurement space
  //----------------------------------------------------------------------------
  int n_points = state.sigma_points.cols();

  auto z_sigma = MatrixXd(z.size(), n_points);
  for(int i = 0; i < n_points; ++i)
    z_sigma.col(i) = Transform(state.sigma_points.col(i));

  //----------------------------------------------------------------------------
  // Compute the projected mean
  //----------------------------------------------------------------------------
  int  n_z    = z.size();
  auto z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for(int i = 0; i < n_points; ++i)
      z_pred += weights_(i) * z_sigma.col(i);

  //----------------------------------------------------------------------------
  // Compute the projected covariance
  //----------------------------------------------------------------------------
  auto S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for(int i = 0; i < n_points; i++) {
    VectorXd z_diff = z_sigma.col(i) - z_pred;
    NormalizeZ(z_diff);
    S += weights_(i) * z_diff * z_diff.transpose();
  }
  S += R_;

  //----------------------------------------------------------------------------
  // Compute ctoss-corelation between state space and measurement space
  //----------------------------------------------------------------------------
  int n_x = state.x.size();
  auto Tc = MatrixXd(n_x, n_z);
  Tc.fill(0.0);
  for(int i = 0; i < n_points; ++i) {
    VectorXd z_diff = z_sigma.col(i) - z_pred;
    NormalizeZ(z_diff);

    VectorXd x_diff = state.sigma_points.col(i) - state.x;
    NormalizeX(x_diff);
    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  //----------------------------------------------------------------------------
  // Update the measurement
  //----------------------------------------------------------------------------
  auto Si = S.inverse();
  auto K = Tc * Si;
  VectorXd y = z - z_pred;
  NormalizeZ(y);

  state.x += K * y;
  state.P -= K*S*K.transpose();

  //----------------------------------------------------------------------------
  // Compute NIS
  //----------------------------------------------------------------------------
  state.nis = y.transpose() * Si * y;
}
