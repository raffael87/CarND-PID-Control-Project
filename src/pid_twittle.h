#ifndef PID_TWITTLE_H
#define PID_TWITTLE_H

#include "PID.h"

#include <iostream>
#include <numeric>
#include <vector>

class PidTwittle {
public:
  PidTwittle(PID &pid);

  void UpdateCteError(double cte);
  void Twittle();
  bool IsLapDriven() const;
  bool IsVehicleCrashed(double cte, double speed);

private:
  void NextParameterIndex();
  void Log() const;

  PID &pid_;

  std::vector<double> parameters_;
  std::vector<double> d_parameters_;

  double best_error_;
  double total_ct_error_;

  unsigned int steps_;
  unsigned int current_parameter_;
  unsigned int counter_;
  unsigned int laps_;

  bool is_vehicle_crashed_;

  enum class Operation { first, inc, dec };

  Operation current_operation_;
};

#endif // PID_TWITTLE_H
