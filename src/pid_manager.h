#ifndef PID_MANAGER_H
#define PID_MANAGER_H

#include "PID.h"

#include <iostream>
#include <numeric>
#include <vector>

class PidsManager {
public:
  PidsManager(PID &pid);

  void UpdateCteError(double cte);
  void Twittle();
  bool IsLapDriven() const;
  bool isOffTrack(double cte, double speed);

private:
  void NextParameterIndex();
  void Log();

  PID &pid_;

  std::vector<double> parameters_;
  std::vector<double> d_parameters_;

  double best_error_;
  double total_ct_error_;

  unsigned int steps_;
  unsigned int current_parameter_;
  unsigned int counter_;
  unsigned int laps_;

  bool offTrack_;

  enum class Operation { first, inc, dec };

  Operation current_operation_;
};

#endif // PID_MANAGER_H
