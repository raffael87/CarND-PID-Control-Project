#include "pid_manager.h"

namespace Config {
constexpr unsigned int kStepsOneRound{5500U};
constexpr unsigned int kStepsToIgnoreAtBeginning{100U};
constexpr double kDParamsTolerance{0.1};
} // namespace Config

PidsManager::PidsManager(PID &pid) : pid_(pid) {
  parameters_ = {pid.Kp, pid.Ki, pid.Kd};
  d_parameters_ = {pid.Kp * 0.1, pid.Ki * 0.1, pid.Kd * 0.1};
  best_error_ = std::numeric_limits<double>::max();
  total_ct_error_ = 0;
  current_operation_ = Operation::first;
  current_parameter_ = 0;
  counter_ = 0;
  laps_ = 0;
  offTrack_ = false;

  // pid_steering_.Init(parameters_[0], parameters_[1], parameters_[2]);
}

void PidsManager::NextParameterIndex() {
  current_parameter_ = (current_parameter_ + 1U) % parameters_.size();
}

void PidsManager::UpdateCteError(double cte) {
  if ((steps_ % (Config::kStepsOneRound + Config::kStepsToIgnoreAtBeginning)) >
      Config::kStepsToIgnoreAtBeginning) {
    total_ct_error_ += pow(cte, 2);
  }
  steps_++;
}

bool PidsManager::isOffTrack(double cte, double speed) {
  // the car is off track if after some initial steps we get very high cte
  // values or small vehicle speed
  if (steps_ > 4 * Config::kStepsToIgnoreAtBeginning) {
    offTrack_ = speed < 4 || cte > 6;
    return offTrack_;
  } else {
    return false;
  }
}

void PidsManager::Twittle() {
  // Calculate sum of dprams and if bigger then perform twittle
  double sum_d_params =
      accumulate(d_parameters_.begin(), d_parameters_.end(), 0.0);

  std::cout << "Sum params: " << sum_d_params << std::endl;
  std::cout << "Operation: " << static_cast<int>(current_operation_)
            << std::endl;

  double error = total_ct_error_ / (steps_ - Config::kStepsToIgnoreAtBeginning);

  std::cout << "Current Error: " << error << std::endl;

  if (sum_d_params > Config::kDParamsTolerance) {
    if (offTrack_) {
      error += 1000U;
    }
    switch (current_operation_) {
    case Operation::first:
      best_error_ = error;
      parameters_[current_parameter_] += d_parameters_[current_parameter_];
      current_operation_ = Operation::inc;
      std::cout << "--- first inc" << std::endl;
      break;
    case Operation::inc:
      if (error > best_error_) {
        std::cout << "--- inc better" << std::endl;
        best_error_ = error;
        d_parameters_[current_parameter_] *= 1.1;
        NextParameterIndex();
        parameters_[current_parameter_] += d_parameters_[current_parameter_];
      } else {
        std::cout << "--- inc worse" << std::endl;
        parameters_[current_parameter_] -=
            2U * d_parameters_[current_parameter_];
        current_operation_ = Operation::dec;
      }
      break;
    case Operation::dec:
      current_operation_ = Operation::inc;
      if (error < best_error_) {
        best_error_ = error;
        d_parameters_[current_parameter_] *= 1.1;
        std::cout << "--- dec better" << std::endl;
      } else {
        std::cout << "--- dec worse" << std::endl;
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
  offTrack_ = false;
}

void PidsManager::Log() {
  std::cout << "###### Lap finished ######" << std::endl;
  std::cout << "Laps: " << laps_ << std::endl;
  std::cout << "Parameter changes: " << counter_ << std::endl;
  std::cout << "Best Error: " << best_error_ << std::endl;
  std::cout << "New Parameters: " << parameters_[0] << " " << parameters_[1]
            << " " << parameters_[2] << std::endl;
  std::cout << "New D Parameters: " << d_parameters_[0] << " "
            << d_parameters_[1] << " " << d_parameters_[2] << std::endl;
  std::cout << "Steps: " << steps_ << std::endl;
  std::cout << "Crashed: " << offTrack_ << std::endl;
  std::cout << "Next Operation: " << static_cast<int>(current_operation_)
            << std::endl;
}

bool PidsManager::IsLapDriven() const {
  return steps_ >= Config::kStepsOneRound;
}
