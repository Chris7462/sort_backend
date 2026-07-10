#include "internal/kalman_filter.hpp"


namespace sort
{

KalmanFilter::KalmanFilter(int dim_x, int dim_z)
: dim_x_(dim_x), dim_z_(dim_z)
{
  // Initialize state and covariance
  x_ = VectorXf::Zero(dim_x_);
  P_ = MatrixXf::Identity(dim_x_, dim_x_);

  // Initialize system matrices
  F = MatrixXf::Identity(dim_x_, dim_x_);
  H = MatrixXf::Zero(dim_z_, dim_x_);
  Q = MatrixXf::Identity(dim_x_, dim_x_);
  R = MatrixXf::Identity(dim_z_, dim_z_);
}

void KalmanFilter::predict()
{
  // State prediction: x = F * x
  x_ = F * x_;

  // Covariance prediction: P = F * P * F' + Q
  P_ = F * P_ * F.transpose() + Q;
}

void KalmanFilter::update(const VectorXf & z)
{
  // Innovation: y = z - H * x
  VectorXf y = z - H * x_;

  // Innovation covariance: S = H * P * H' + R
  MatrixXf S = H * P_ * H.transpose() + R;

  // Kalman gain: K = P * H' * S^-1
  //MatrixXf K = P_ * H.transpose() * S.inverse();
  // Solve K * S = P * H' (i.e. K = (P*H') * S^-1) via LDLT decomposition
  // instead of forming S.inverse() directly. S is symmetric positive
  // semi-definite by construction, so LDLT is both applicable and more
  // numerically robust than an explicit matrix inverse, especially as S
  // becomes ill-conditioned.
  MatrixXf PHt = P_ * H.transpose();
  MatrixXf K = S.ldlt().solve(PHt.transpose()).transpose();

  // State update: x = x + K * y
  x_ = x_ + K * y;

  // Covariance update (Joseph form): P = (I - KH) * P * (I - KH)' + K * R * K'
  MatrixXf I_KH = MatrixXf::Identity(dim_x_, dim_x_) - K * H;
  P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
}

const KalmanFilter::VectorXf & KalmanFilter::getState() const
{
  return x_;
}

void KalmanFilter::setState(const VectorXf & x)
{
  x_ = x;
}

void KalmanFilter::setCovariance(const MatrixXf & P)
{
  P_ = P;
}

const KalmanFilter::MatrixXf & KalmanFilter::getCovariance() const
{
  return P_;
}

} // namespace sort
