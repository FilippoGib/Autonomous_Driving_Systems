#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <eigen3/Eigen/Dense>

class KalmanFilter
{
public:
  KalmanFilter();
  ~KalmanFilter();

  // init the filter
  void init(double dt);

  void predict();
  void update(const Eigen::VectorXd& z);

  // setters
  void setState(double x, double y);

  // getters
  double getXCovariance() { return P_.coeff(0, 0); }
  double getYCovariance() { return P_.coeff(1, 1); }
  double getX() { return x_[0]; }
  double getY() { return x_[1]; }
  Eigen::MatrixXd getS(){return S_;};

private:
  // dt in seconds
  double dt_;

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transistion matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix
  Eigen::MatrixXd H_;

  // measurement covariance matrix
  Eigen::MatrixXd R_;

  Eigen::MatrixXd S_; // I need to expose the innvoation covariance to compute the Mahalanobis distance
};

#endif // KALMAN_FILTER_H_
