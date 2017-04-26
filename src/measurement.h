//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   23.04.2017
//------------------------------------------------------------------------------

#pragma once

#include <cstdint>
#include <Eigen/Dense>

struct Measurement {
  uint64_t timestamp;
  enum SensorType {
    LASER = 0,
    RADAR
  } sensor_type;

  Eigen::VectorXd data;
};
