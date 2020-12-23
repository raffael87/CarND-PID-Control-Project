#include "pid_twittle.h"

namespace Config {
constexpr unsigned int kStepsOneRound{5500U};
constexpr unsigned int kStepsToIgnoreAtBeginning{100U};
constexpr unsigned int kCrashSteps{100U};
constexpr unsigned int kCrashMinVelocity{4U};
constexpr unsigned int kCrashMinSteps{kCrashSteps * kCrashMinVelocity};
constexpr unsigned int kCrashPenalization{1000U};
constexpr double kDParamsTolerance{0.1};
} // namespace Config

using namespace Config;

PidTwittle::PidTwittle(PID &pid) : pid_(pid) {
  parameters_ = {pid.Kp, pid.Ki, pid.Kd};
  d_parameters_ = {pid.Kp * 0.1, pid.Ki * 0.1, pid.Kd * 0.1};
  best_error_ = std::numeric_limits<double>::max();
  total_ct_error_ = 0;
  current_operation_ = Operation::first;
  current_parameter_ = 0;
  counter_ = 0;
  laps_ = 0;
  is_vehicle_crashed_ = false;
}

void PidTwittle::NextParameterIndex() {
  current_parameter_ = (current_parameter_ + 1U) % parameters_.size();
}

void PidTwittle::UpdateCteError(double cte) {
  if (steps_ > kStepsToIgnoreAtBeginning) {
    total_ct_error_ += pow(cte, 2);
  }

  ++steps_;
}

bool PidTwittle::IsVehicleCrashed(const double cte, const double speed) {
  is_vehicle_crashed_ = false;
  if (kCrashMinSteps < steps_) {
    if ((6U < cte) || kCrashMinVelocity > speed)
      is_vehicle_crashed_ = true;
  }
  return is_vehicle_crashed_;
}

void PidTwittle::Twittle() {
  // Calculate sum of dprams and if bigger then perform twittle
  double sum_d_params =
      accumulate(d_parameters_.begin(), d_parameters_.end(), 0.0);

  std::cout << "###### Lap finished ######" << std::endl;
  std::cout << "Sum params: " << sum_d_params << std::endl;
  std::cout << "Current Operation: " << static_cast<int>(current_operation_)
            << std::endl;

  double error = total_ct_error_ / (steps_ - kStepsToIgnoreAtBeginning);

  std::cout << "Current Error: " << error << std::endl;

  if (sum_d_params > kDParamsTolerance) {
    if (is_vehicle_crashed_) {
      error += kCrashPenalization;
    }
    switch (current_operation_) {
    case Operation::first:
      current_operation_ = Operation::inc;
      best_error_ = error;
      parameters_[current_parameter_] += d_parameters_[current_parameter_];
      // std::cout << "--- first inc" << std::endl;
      break;
    case Operation::inc:
      if (error > best_error_) {
        // std::cout << "--- inc better" << std::endl;
        best_error_ = error;
        d_parameters_[current_parameter_] *= 1.1;
        NextParameterIndex();
        parameters_[current_parameter_] += d_parameters_[current_parameter_];
      } else {
        // std::cout << "--- inc worse" << std::endl;
        current_operation_ = Operation::dec;
        parameters_[current_parameter_] -=
            2U * d_parameters_[current_parameter_];
      }
      break;
    case Operation::dec:
      current_operation_ = Operation::inc;
      if (error < best_error_) {
        best_error_ = error;
        d_parameters_[current_parameter_] *= 1.1;
        // std::cout << "--- dec better" << std::endl;
      } else {
        // std::cout << "--- dec worse" << std::endl;
        parameters_[current_parameter_] += d_parameters_[current_parameter_];
        d_parameters_[current_parameter_] *= 0.9;
      }
      NextParameterIndex();
      parameters_[current_parameter_] += d_parameters_[current_parameter_];
      break;
    }

    // after one iteration we have to update the values in the pid
    pid_.Init(parameters_[0], parameters_[1], parameters_[2]);
    ++counter_;
  }

  ++laps_;

  Log();

  // reset error, steps etc
  total_ct_error_ = 0.0;
  steps_ = 0U;
  is_vehicle_crashed_ = false;
}

void PidTwittle::Log() const {
  std::cout << "Laps: " << laps_ << std::endl;
  std::cout << "Parameter changes: " << counter_ << std::endl;
  std::cout << "Best Error: " << best_error_ << std::endl;
  std::cout << "New Parameters: " << parameters_[0] << " " << parameters_[1]
            << " " << parameters_[2] << std::endl;
  std::cout << "New D Parameters: " << d_parameters_[0] << " "
            << d_parameters_[1] << " " << d_parameters_[2] << std::endl;
  std::cout << "Steps: " << steps_ << std::endl;
  std::cout << "Crashed: " << is_vehicle_crashed_ << std::endl;
  std::cout << "Next Operation: " << static_cast<int>(current_operation_)
            << std::endl;
}

bool PidTwittle::IsLapDriven() const { return steps_ >= kStepsOneRound; }
