//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   23.04.2017
//------------------------------------------------------------------------------

#pragma once

#include <ostream>
#include <vector>
#include <string>
#include <Eigen/Dense>

#include "measurement.h"

namespace Utils {
  //----------------------------------------------------------------------------
  //! Output measurement
  //----------------------------------------------------------------------------
  void Output(std::ostream          &out,
              const Measurement     &m,
              const Eigen::VectorXd &estimate,
              double                 nis,
              const Eigen::VectorXd &ground_truth);

  //----------------------------------------------------------------------------
  //! Read the dataset
  //----------------------------------------------------------------------------
  void ReadDataset(std::vector<Measurement>     &measurements,
                   std::vector<Eigen::VectorXd> &ground_truth,
                   const std::string            &filename);

  //----------------------------------------------------------------------------
  //! Calculate RMSE
  //----------------------------------------------------------------------------
  Eigen::ArrayXd CalculateRMSE(
    const std::vector<Eigen::VectorXd> &estimations,
    const std::vector<Eigen::VectorXd> &ground_truth);
}
