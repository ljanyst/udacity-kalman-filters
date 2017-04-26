//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   25.04.2017
//------------------------------------------------------------------------------

#include <cmath>
#include <Eigen/Dense>

#include "lidar_radar_ukf.h"

using namespace std;
using namespace Eigen;

namespace {
  //----------------------------------------------------------------------------
  // Unscented predictor for the model
  //----------------------------------------------------------------------------
  class Predictor: public UnscentedKalmanPredictor {
    public:
      //------------------------------------------------------------------------
      // Constructor
      //------------------------------------------------------------------------
      Predictor() {
        Q_ = MatrixXd(2, 2);
        Q_ << 0.7,    0,
                0, 0.15;
        nu_ = VectorXd(2);
        nu_ << 0, 0;
        n_dims_ = 5;
        lambda_ = 3. - (n_dims_+nu_.size());
        InitializeWeights();
      }

      //------------------------------------------------------------------------
      // Transformation function
      //------------------------------------------------------------------------
      virtual VectorXd Transform(const VectorXd &x,
                                 uint64_t        dt) override {

        //----------------------------------------------------------------------
        // Extract variables for better readability
        //----------------------------------------------------------------------
        double px       = x(0);
        double py       = x(1);
        double v        = x(2);
        double yaw      = x(3);
        double yawd     = x(4);
        double nu_a     = x(5);
        double nu_yawdd = x(6);

        double dt1      = dt/1000000.0;
        double dt2      = dt1*dt1;

        //----------------------------------------------------------------------
        // Compute the new state
        //----------------------------------------------------------------------
        double px_p, py_p;
        if(fabs(yawd) > 0.001) {
          px_p = px + v/yawd * (sin(yaw+yawd*dt1) - sin(yaw));
          py_p = py - v/yawd * (cos(yaw+yawd*dt1) - cos(yaw));
        }
        else {
          px_p = px + v*dt1*cos(yaw);
          py_p = py + v*dt1*sin(yaw);
        }

        double v_p    = v;
        double yaw_p  = yaw + yawd*dt1;
        double yawd_p = yawd;

        //----------------------------------------------------------------------
        // Add noise
        //----------------------------------------------------------------------
        px_p   += 0.5*nu_a*dt2*cos(yaw);
        py_p   += 0.5*nu_a*dt2*sin(yaw);
        v_p    += nu_a*dt1;
        yaw_p  += 0.5*nu_yawdd*dt2;
        yawd_p += nu_yawdd*dt1;

        //----------------------------------------------------------------------
        // Write the predictions
        //----------------------------------------------------------------------
        auto result = VectorXd(5);
        result(0) = px_p;
        result(1) = py_p;
        result(2) = v_p;
        result(3) = yaw_p;
        result(4) = yawd_p;
        return result;
      }

      //------------------------------------------------------------------------
      // Normalize X
      //------------------------------------------------------------------------
      virtual void Normalize(VectorXd &x) override {
        for(; x(3) < -M_PI; x(3) += 2*M_PI);
        for(; x(3) >  M_PI; x(3) -= 2*M_PI);
      }
  };

  //----------------------------------------------------------------------------
  // Lidar Updater
  //----------------------------------------------------------------------------
  class LidarUpdater: public LinearKalmanUpdater {
    public:
      LidarUpdater() {
        H_ = MatrixXd(2, 5);
        H_ << 1, 0, 0, 0, 0,
              0, 1, 0, 0, 0;

        R_ = MatrixXd(2, 2);
        R_ << 0.0225,      0,
              0,      0.0225;
      }
  };

  //----------------------------------------------------------------------------
  // Radar Updater
  //----------------------------------------------------------------------------
  class RadarUpdater: public UnscentedKalmanUpdater {
    public:
      //------------------------------------------------------------------------
      // Constructor
      //------------------------------------------------------------------------
      RadarUpdater() {
        R_ = MatrixXd(3, 3);
        R_ << 0.09,      0,    0,
                 0, 0.0009,    0,
                 0,      0, 0.09;

        n_aug_  = 7;
        lambda_ = 3. - n_aug_;
        InitializeWeights();
      }

      //------------------------------------------------------------------------
      // Transform
      //------------------------------------------------------------------------
      virtual VectorXd Transform(const VectorXd &x) override {
        double    px     = x(0);
        double    py     = x(1);
        double    v      = x(2);
        double    yaw    = x(3);
        double    vx     = v*cos(yaw);
        double    vy     = v*sin(yaw);

        double    sq     = px*px + py*py;
        double    sqroot = sqrt(sq);

        auto result = VectorXd(3);
        result(0) = sqroot;                     // rho
        result(1) = atan2(py, px);              // phi
        result(2) = (px*vx + py*vy)/sqroot;     // rho dot
        return result;
      }

      //------------------------------------------------------------------------
      // Normalize X
      //------------------------------------------------------------------------
      virtual void NormalizeX(Eigen::VectorXd &x) override {
        for(; x(3) < -M_PI; x(3) += 2*M_PI);
        for(; x(3) >  M_PI; x(3) -= 2*M_PI);
      }

      //------------------------------------------------------------------------
      // Normalize Z
      //------------------------------------------------------------------------
      virtual void NormalizeZ(Eigen::VectorXd &z) override {
        for(; z(1) < -M_PI; z(1) += 2*M_PI);
        for(; z(1) >  M_PI; z(1) -= 2*M_PI);
      }
  };
}

//------------------------------------------------------------------------------
// Constructor
//------------------------------------------------------------------------------
LidarRadarUKF::LidarRadarUKF(): state_(5), predictor_(new Predictor()) {
  state_.P << 1, 0,  0,  0,   0,
              0, 1,  0,  0,   0,
              0, 0, 20,  0,   0,
              0, 0,  0, 15,   0,
              0, 0,  0,  0, 0.1;

  state_.sigma_points = MatrixXd(5, 15);
  updaters_.emplace_back(new LidarUpdater());
  updaters_.emplace_back(new RadarUpdater());
}

//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
LidarRadarUKF::~LidarRadarUKF() {
}

//------------------------------------------------------------------------------
// Process a measurement
//------------------------------------------------------------------------------
const KalmanState &LidarRadarUKF::ProcessMeasurement(
  const Measurement &measurement) {

  //----------------------------------------------------------------------------
  // Initialization
  //----------------------------------------------------------------------------
  if(!is_initialized_) {
    if(measurement.sensor_type == Measurement::LASER)
      state_.x << measurement.data[0], measurement.data[1], 0, 0, 0;
    else if(measurement.sensor_type == Measurement::RADAR) {
      double rho     = measurement.data[0];
      double phi     = measurement.data[1];
      double rho_dot = measurement.data[2];
      state_.x << rho*cos(phi), rho*sin(phi), 0, 0, rho_dot;
    }
    is_initialized_     = true;
    previous_timestamp_ = measurement.timestamp;
    return state_;
  }

  //----------------------------------------------------------------------------
  // Prediction and measurement processing
  //----------------------------------------------------------------------------
  double dt = measurement.timestamp - previous_timestamp_;
  previous_timestamp_ = measurement.timestamp;
  predictor_->Predict(state_, dt);
  updaters_[measurement.sensor_type]->Update(state_, measurement.data);
  return state_;
}
