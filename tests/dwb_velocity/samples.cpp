// Exercise the unmodified Humble DWB velocity iterator, not a Python copy.
#include <cmath>
#include <iostream>
#include <stdexcept>
#include "dwb_plugins/one_d_velocity_iterator.hpp"

int main(int argc, char **argv)
{
  if (argc != 5) {return 2;}
  const double speed = std::stod(argv[1]), accel = std::stod(argv[2]);
  const double horizon = std::stod(argv[3]);
  const int samples = std::stoi(argv[4]);
  for (double current : {-0.12, 0.0, 0.08, 0.15, 0.21, 0.30, 0.45}) {
    dwb_plugins::OneDVelocityIterator it(current, 0.0, speed, accel, -accel, horizon, samples);
    int zero = 0, walking = 0;
    for (; !it.isFinished(); ++it) {
      const double value = it.getVelocity();
      if (std::abs(value) < 1e-6) {++zero;}
      else if (std::abs(value-speed) < 1e-6) {++walking;}
      else {throw std::runtime_error("DWB offered sustained sub-walking forward speed");}
    }
    if (zero != 1 || walking != 1) {
      throw std::runtime_error("DWB cannot offer both rotation-in-place X=0 and walking from this odometry");
    }
  }
  std::cout << "PASS upstream DWB X samples: 0 and " << speed
            << " reachable from rest, body sway, walking and over-limit odometry\n";
}
