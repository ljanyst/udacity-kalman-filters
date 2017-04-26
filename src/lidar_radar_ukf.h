//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   23.05.2017
//------------------------------------------------------------------------------

#pragma once

#include <vector>
#include <memory>

#include "kalman_filter.h"
#include "measurement.h"

//------------------------------------------------------------------------------
//! Lidar and radar fusion using an Unscented Kalman Filter
//------------------------------------------------------------------------------
class LidarRadarUKF {
  public:
    //--------------------------------------------------------------------------
    //! Constructor
    //--------------------------------------------------------------------------
    LidarRadarUKF();

    //--------------------------------------------------------------------------
    //! Destructor
    //--------------------------------------------------------------------------
    virtual ~LidarRadarUKF();

    //--------------------------------------------------------------------------
    //! Process a measurement
    //--------------------------------------------------------------------------
    const KalmanState &ProcessMeasurement(const Measurement &measurement);

  private:
    bool                                        is_initialized_;
    long long                                   previous_timestamp_;
    KalmanState                                 state_;
    std::unique_ptr<KalmanPredictor>            predictor_;
    std::vector<std::unique_ptr<KalmanUpdater>> updaters_;
};
