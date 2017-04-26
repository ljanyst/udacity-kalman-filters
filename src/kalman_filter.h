//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   25.04.2017
//------------------------------------------------------------------------------

#pragma once

#include <cstdint>
#include <Eigen/Dense>

//------------------------------------------------------------------------------
//! State of the Kalman filter
//------------------------------------------------------------------------------
struct KalmanState {
  KalmanState(int n) {
    x = Eigen::VectorXd(n);
    P = Eigen::MatrixXd(n, n);
    x.fill(0.0);
    P.fill(0.0);
  }
  Eigen::VectorXd x;
  Eigen::MatrixXd P;
  Eigen::MatrixXd sigma_points;
  double          nis = 0;
};

//------------------------------------------------------------------------------
//! Base class for the prediction strategy
//------------------------------------------------------------------------------
class KalmanPredictor {
  public:
    virtual ~KalmanPredictor() {}
    virtual void Predict(KalmanState &state, uint64_t dt) = 0;
};

//------------------------------------------------------------------------------
//! Base class for the update strategy
//------------------------------------------------------------------------------
class KalmanUpdater {
  public:
    virtual ~KalmanUpdater() {}
    virtual void Update(KalmanState           &state,
                        const Eigen::VectorXd &z) = 0;
};

//------------------------------------------------------------------------------
//! Linear prediction
//------------------------------------------------------------------------------
class LinearKalmanPredictor: public KalmanPredictor {
  public:
    virtual ~LinearKalmanPredictor() {}
    virtual void Predict(KalmanState &state, uint64_t dt) override;
  protected:
    Eigen::MatrixXd F_;
    Eigen::MatrixXd Q_;
};

//------------------------------------------------------------------------------
//! Unscented prediction
//------------------------------------------------------------------------------
class UnscentedKalmanPredictor: public KalmanPredictor {
  public:
    virtual ~UnscentedKalmanPredictor() {}
    virtual void Predict(KalmanState &state, uint64_t dt) override;
  protected:
    virtual Eigen::VectorXd Transform(const Eigen::VectorXd &x,
                                      uint64_t         dt) = 0;
    virtual void Normalize(Eigen::VectorXd &x) { (void) x; }
    void InitializeWeights();
    Eigen::MatrixXd Q_;
    Eigen::VectorXd nu_;
    Eigen::VectorXd weights_;
    unsigned        n_dims_;
    double          lambda_;

  private:
    void CalculateSigmaPoints(Eigen::MatrixXd &sigma_points,
                              KalmanState     &state);
};

//------------------------------------------------------------------------------
//! Linear update
//------------------------------------------------------------------------------
class LinearKalmanUpdater: public KalmanUpdater {
  public:
    virtual ~LinearKalmanUpdater() {}
    virtual void Update(KalmanState           &state,
                        const Eigen::VectorXd &z) override;
  protected:
    Eigen::MatrixXd H_;
    Eigen::MatrixXd R_;
};

//------------------------------------------------------------------------------
//! Non-linear update
//------------------------------------------------------------------------------
class ExtendedKalmanUpdater: public KalmanUpdater {
  public:
    virtual ~ExtendedKalmanUpdater() {}
    virtual void Update(KalmanState           &state,
                        const Eigen::VectorXd &z) override;
  protected:
    Eigen::MatrixXd Hj_;
    Eigen::MatrixXd R_;
    Eigen::VectorXd y_;
};

//------------------------------------------------------------------------------
//! Unscented update
//------------------------------------------------------------------------------
class UnscentedKalmanUpdater: public KalmanUpdater {
  public:
    virtual ~UnscentedKalmanUpdater() {}
    virtual void Update(KalmanState           &state,
                        const Eigen::VectorXd &z) override;
  protected:
    virtual Eigen::VectorXd Transform(const Eigen::VectorXd &x) = 0;
    virtual void NormalizeX(Eigen::VectorXd &x) { (void) x; }
    virtual void NormalizeZ(Eigen::VectorXd &z) { (void) z; }

    void InitializeWeights();
    Eigen::MatrixXd R_;
    Eigen::VectorXd weights_;
    unsigned        n_aug_;
    double          lambda_;

  private:
    void CalculateSigmaPoints(Eigen::MatrixXd &sigma_points,
                              KalmanState     &state);
};
