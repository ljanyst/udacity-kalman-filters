//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   23.04.2017
//------------------------------------------------------------------------------

#include <stdexcept>
#include <fstream>
#include "utils.h"

using namespace Eigen;
using namespace std;

namespace Utils {
  //----------------------------------------------------------------------------
  // Output measurement
  //----------------------------------------------------------------------------
  void Output(std::ostream      &out,
              const Measurement &m,
              const VectorXd    &estimate,
              double             nis,
              const VectorXd    &ground_truth) {

    //--------------------------------------------------------------------------
    // Output the measurement
    //--------------------------------------------------------------------------
    if(m.sensor_type == Measurement::LASER) {
      out << "L"         << " ";
      out << m.timestamp << " ";
      out << m.data(0)   << " ";
      out << m.data(1)   << " ";
    }
    else if(m.sensor_type == Measurement::RADAR) {
      double rho = m.data(0);
      double phi = m.data(1);

      out << "R"            << " ";
      out << m.timestamp    << " ";
      out << rho * cos(phi) << " ";
      out << rho * sin(phi) << " ";
    }

    //--------------------------------------------------------------------------
    // Output the result
    //--------------------------------------------------------------------------
    out << estimate(0) << " ";
    out << estimate(1) << " ";
    out << estimate(2) << " ";
    out << estimate(3) << " ";
    out << nis         << " ";

    //--------------------------------------------------------------------------
    // Output the ground truth
    //--------------------------------------------------------------------------
    out << ground_truth(0) << " ";
    out << ground_truth(1) << " ";
    out << ground_truth(2) << " ";
    out << ground_truth(3) << "\n";
  }

  //----------------------------------------------------------------------------
  // Read the dataset
  //----------------------------------------------------------------------------
  void ReadDataset(std::vector<Measurement>     &measurements,
                   std::vector<Eigen::VectorXd> &ground_truth,
                   const std::string            &filename) {

    //--------------------------------------------------------------------------
    // Open the file
    //--------------------------------------------------------------------------
    ifstream in_file(filename.c_str());
    if(!in_file.is_open())
      throw runtime_error("Cannot open input file: " + filename);

    //--------------------------------------------------------------------------
    // Process the input
    //--------------------------------------------------------------------------
    string line;
    while(getline(in_file, line)) {
      string        sensor_type;
      Measurement   meas;
      istringstream iss(line);
      long long     timestamp;

      iss >> sensor_type;

      //------------------------------------------------------------------------
      // Read a laser measurement
      //------------------------------------------------------------------------
      if(sensor_type.compare("L") == 0) {
        meas.sensor_type = Measurement::LASER;
        meas.data        = VectorXd(2);

        double x, y;
        iss >> x >> y;
        meas.data << x, y;
      }

      //------------------------------------------------------------------------
      // Read a radar measurement
      //------------------------------------------------------------------------
      else if(sensor_type.compare("R") == 0) {
        meas.sensor_type = Measurement::RADAR;
        meas.data = VectorXd(3);

        double rho, phi, rho_dot;
        iss >> rho >> phi >> rho_dot;
        meas.data << rho, phi, rho_dot;
      }

      iss >> timestamp;
      meas.timestamp = timestamp;
      measurements.push_back(meas);

      //------------------------------------------------------------------------
      // Read ground truth
      //------------------------------------------------------------------------
      double x, y, vx, vy;
      iss >> x >> y >> vx >> vy;
      auto gt = VectorXd(4);
      gt << x, y, vx, vy;
      ground_truth.push_back(gt);
    }
    in_file.close();
  }

  //----------------------------------------------------------------------------
  // Calculate RMSE
  //----------------------------------------------------------------------------
  ArrayXd CalculateRMSE(const std::vector<VectorXd> &estimations,
                        const std::vector<VectorXd> &ground_truth) {

    if(estimations.size() != ground_truth.size() || estimations.empty())
      return ArrayXd(1);

    auto result = ArrayXd(estimations[0].size());
    for(int i = 0; i < result.size(); ++i)
      result[i] = 0;

    for(size_t i = 0; i < estimations.size(); ++i) {
      ArrayXd e = estimations[i] - ground_truth[i];
      result += e*e;
    }
    result /= estimations.size();
    return result.sqrt();
  }
}
