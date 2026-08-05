#ifndef USE_FAST_CALIB2_SVD_SOLVER_HPP
#define USE_FAST_CALIB2_SVD_SOLVER_HPP

#include <Eigen/Dense>

namespace use_fast_calib2 {

// Compute full SVD of a matrix (dynamic-size)
inline void computeFullSVD(const Eigen::MatrixXd &A,
                           Eigen::MatrixXd &U,
                           Eigen::VectorXd &S,
                           Eigen::MatrixXd &Vt)
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  U = svd.matrixU();
  S = svd.singularValues();
  Vt = svd.matrixV().transpose();
}

// Compute Moore-Penrose pseudo-inverse with tolerance
inline Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd &A, double tol = 1e-9)
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd &S = svd.singularValues();
  Eigen::VectorXd S_inv = S;
  for (int i = 0; i < S.size(); ++i) {
    if (S(i) > tol)
      S_inv(i) = 1.0 / S(i);
    else
      S_inv(i) = 0.0;
  }
  return svd.matrixV() * S_inv.asDiagonal() * svd.matrixU().transpose();
}

} // namespace use_fast_calib2

#endif // USE_FAST_CALIB2_SVD_SOLVER_HPP
