//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   25.05.2017
//------------------------------------------------------------------------------

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

#include "lidar_radar_ukf.h"
#include "utils.h"
#include "measurement.h"

using namespace std;
using namespace Eigen;

//------------------------------------------------------------------------------
// Start the show
//------------------------------------------------------------------------------
int main(int argc, char **argv) {
  //----------------------------------------------------------------------------
  // Process arguments
  //----------------------------------------------------------------------------
  string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " path/to/input.txt output.txt";

  if(argc != 3) {
    cerr << usage_instructions << endl;
    return 1;
  }

  ofstream out_file(argv[2]);

  if(!out_file.is_open()) {
    cerr << "Cannot open output file: " << argv[2] << endl;
    return 1;
  }

  //----------------------------------------------------------------------------
  // Read the dataset
  //----------------------------------------------------------------------------
  vector<Measurement> measurements;
  vector<VectorXd>    ground_truth;
  try {
    Utils::ReadDataset(measurements, ground_truth, argv[1]);
  }
  catch(std::runtime_error &e) {
    cerr << "Unable to read data: " << e.what() << std::endl;
    return 1;
  }

  //----------------------------------------------------------------------------
  // Process the measurements
  //----------------------------------------------------------------------------
  LidarRadarUKF fusion;
  vector<VectorXd> estimations;

  for(size_t i = 0; i < measurements.size(); ++i) {
    const auto &m  = measurements[i];
    const auto &gt = ground_truth[i];
    auto result = fusion.ProcessMeasurement(m);

    //--------------------------------------------------------------------------
    // Convert the estimation result to cartesian coordiantes
    //--------------------------------------------------------------------------
    VectorXd cartesian = VectorXd(4);
    auto &x = result.x;
    double px = x(0);
    double py = x(1);
    double vx = x(2) * cos(x(3));
    double vy = x(2) * sin(x(3));
    cartesian << px, py, vx, vy;
    estimations.push_back(cartesian);

    Utils::Output(out_file, m, cartesian, result.nis, gt);
  }

  //----------------------------------------------------------------------------
  // Compute the accuracy (RMSE)
  //----------------------------------------------------------------------------
  cout << "Accuracy - RMSE:" << endl;
  cout << Utils::CalculateRMSE(estimations, ground_truth) << endl;

  //----------------------------------------------------------------------------
  // Cleanup
  //----------------------------------------------------------------------------
  out_file.close();

  return 0;
}
