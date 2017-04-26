//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   23.04.2017
//------------------------------------------------------------------------------

#include <cmath>
#include <Eigen/Dense>

#include "lidar_radar_ekf.h"

using namespace std;
using namespace Eigen;

namespace {
  //----------------------------------------------------------------------------
  // Linear prodictor for our model
  //----------------------------------------------------------------------------
  class Predictor: public LinearKalmanPredictor {
    public:
      //------------------------------------------------------------------------
      // Constructor
      //------------------------------------------------------------------------
      Predictor() {
        F_ = MatrixXd(4, 4);
        F_ << 1, 0, 1, 0,
              0, 1, 0, 1,
              0, 0, 1, 0,
              0, 0, 0, 1;
        Q_ = MatrixXd(4, 4);
      }

      //------------------------------------------------------------------------
      // Predict
      //------------------------------------------------------------------------
      virtual void Predict(KalmanState &state, uint64_t dt) override {
        double dt1 = dt/1000000.0;
        double dt2 = dt1*dt1;
        double dt3 = dt2*dt1;
        double dt4 = dt3*dt1;

        F_(0, 2) = dt1;
        F_(1, 3) = dt1;

        Q_ << dt4/4*noise_ax_,               0, dt3/2*noise_ax_, 0,
                            0, dt4/4*noise_ay_,               0, dt3/2*noise_ay_,
              dt3/2*noise_ax_,               0,   dt2*noise_ax_, 0,
                            0, dt3/2*noise_ay_,               0, dt2*noise_ay_;
        return LinearKalmanPredictor::Predict(state, dt);
      }
    private:
      double noise_ax_ = 9;
      double noise_ay_ = 9;
  };

  //----------------------------------------------------------------------------
  // Lidar Updater
  //----------------------------------------------------------------------------
  class LidarUpdater: public LinearKalmanUpdater {
    public:
      LidarUpdater() {
        H_ = MatrixXd(2, 4);
        H_ << 1, 0, 0, 0,
              0, 1, 0, 0;

        R_ = MatrixXd(2, 2);
        R_ << 0.0225,      0,
              0,      0.0225;
      }
  };

  //----------------------------------------------------------------------------
  // Radar Updater
  //----------------------------------------------------------------------------
  class RadarUpdater: public ExtendedKalmanUpdater {
    public:
      //------------------------------------------------------------------------
      // Constructor
      //------------------------------------------------------------------------
      RadarUpdater() {
        Hj_ = MatrixXd(3, 4);
        R_  = MatrixXd(3, 3);
        R_ << 0.09,      0,    0,
                 0, 0.0009,    0,
                 0,      0, 0.09;
      }

      //------------------------------------------------------------------------
      // Update
      //------------------------------------------------------------------------
      virtual void Update(KalmanState           &state,
                          const Eigen::VectorXd &z) override {

        VectorXd &x      = state.x;
        double    px     = x(0);
        double    py     = x(1);
        double    vx     = x(2);
        double    vy     = x(3);
        double    sq     = px*px + py*py;
        double    sqroot = sqrt(sq);
        double    sq15   = sq*sqroot;

        if(fabs(sq) < 0.0001)
          return;

        //----------------------------------------------------------------------
        // Compute projection of the estimated state onto the measurement space
        // and the error
        //----------------------------------------------------------------------
        auto zp = VectorXd(3);
        zp << sqroot,
              atan2(py, px),
              (px*vx + py*vy)/sqroot;
        y_ = z - zp;
        for(; y_(1) < -M_PI; y_(1) += 2*M_PI);
        for(; y_(1) > M_PI; y_(1) -= 2*M_PI);

        //----------------------------------------------------------------------
        // Compute the Jacobian
        //----------------------------------------------------------------------
        Hj_ <<             (px/sqroot),             (py/sqroot),         0,         0,
                              -(py/sq),                 (px/sq),         0,         0,
               py*(vx*py - vy*px)/sq15, px*(px*vy - py*vx)/sq15, px/sqroot, py/sqroot;

        return ExtendedKalmanUpdater::Update(state, z);
      }

  };
}

//------------------------------------------------------------------------------
// Constructor
//------------------------------------------------------------------------------
LidarRadarEKF::LidarRadarEKF(): state_(4), predictor_(new Predictor()) {
  state_.P << 1, 0,   0,   0,
              0, 1,   0,   0,
              0, 0, 200,   0,
              0, 0,   0, 200;

  updaters_.emplace_back(new LidarUpdater());
  updaters_.emplace_back(new RadarUpdater());
}

//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
LidarRadarEKF::~LidarRadarEKF() {
}

//------------------------------------------------------------------------------
// Process a measurement
//------------------------------------------------------------------------------
const KalmanState &LidarRadarEKF::ProcessMeasurement(
  const Measurement &measurement) {

  //----------------------------------------------------------------------------
  // Initialization
  //----------------------------------------------------------------------------
  if(!is_initialized_) {
    if(measurement.sensor_type == Measurement::LASER)
      state_.x << measurement.data[0], measurement.data[1], 0, 0;
    else if(measurement.sensor_type == Measurement::RADAR) {
      double rho =  measurement.data[0];
      double phi =  measurement.data[1];
      state_.x << rho*cos(phi), rho*sin(phi), 0, 0;
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
