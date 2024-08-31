// Copyright (C) 2024 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file stepper_driver.h
/// @brief Class to control stepper motors via stepper motor drivers.

#ifndef STEPPER_DRIVER_H_
#define STEPPER_DRIVER_H_

#include <Arduino.h>

namespace mt {

/// @brief The Stepper Driver class.
class StepperDriver {
 public:

  /// @brief Enum of power states based on the ENA/EN pin.
  enum class PowerState {
    kEnabled = 0,
    kDisabled,
  };

  /// @brief Enum of angular speed unit.
  enum class SpeedUnits {
    kMicrostepsPerSecond = 0,
    kDegreesPerSecond,
    kRadiansPerSecond,
    kRevolutionsPerMinute,
  };

  /// @brief Enum of angular acceleration units.
  enum class AccelerationUnits {
    kMicrostepsPerSecondPerSecond = 0,
    kDegreesPerSecondPerSecond,
    kRadiansPerSecondPerSecond,
    kRevolutionsPerMinutePerMinute,
  };

  /// @brief Enum of angular units.
  enum class AngleUnits {
    kMicrosteps = 0,
    kDegrees,
    kRadians,
    kRevolutions,
  };

  /// @brief Enum of calculation options.
  enum class CalculationOption {
    kCalculateOnly = 0,
    kSetupMotion,
  };

  /// @brief Enum of the types of motion/motion control.
  enum class MotionType {
    kStopAndReset = 0,
    kPause,
    kResume,
    kAbsolute,
    kRelative,
  };

  /// @brief Enum of motion status.
  enum class MotionStatus {
    kIdle = 0,
    kPaused,
    kAccelerate,
    kConstantSpeed,
    kDecelerate,
  };

  /// @brief Enum of motor motion directions based on the DIR/CW pin.
  enum class MotionDirection {
    kNegative = -1,
    kNeutral = 0,
    kPositive = 1,
  };

  /// @brief Construct a Stepper Driver object. 
  /// @param pul_pin PUL/STP/CLK (pulse/step/clock) pin.
  /// @param dir_pin DIR/CW (direction) pin.
  /// @param ena_pin ENA/EN (enable) pin.
  /// @param microstep_mode Microstep mode (1 = full step, 2 = half step (1/2), 4 = quarter step (1/4), etc.).  
  /// @param full_step_angle_degrees Motor full step angle in degrees.
  /// @param gear_ratio Gear ratio for motors coupled with a gearbox in the drive system.
  StepperDriver(uint8_t pul_pin, uint8_t dir_pin, uint8_t ena_pin, uint8_t microstep_mode = 1,
                float full_step_angle_degrees = 1.8, double gear_ratio = 1);

  /// @brief Destroy the Stepper Driver object.
  ~StepperDriver();

  /// @brief Set the target speed at which to move the motor.
  /// @param speed The target speed.
  /// @param speed_units The units of the specified speed.
  void SetSpeed(double speed, SpeedUnits speed_units = SpeedUnits::kRevolutionsPerMinute);

  /// @brief Set the target acceleration/deceleration for the motor to accelerate to the target speed and decelerate to a stop.
  /// @param acceleration The target acceleration/deceleration.
  /// @param acceleration_units The units of the specified acceleration.
  void SetAcceleration(double acceleration, 
                       AccelerationUnits acceleration_units = AccelerationUnits::kRadiansPerSecondPerSecond);

  /// @brief Calculate the relative number of microsteps to move to a target angle with respect to the; current angular position (relative), OR; zero/home angular position (absolute).
  /// @param angle The target angle (positive or negative).
  /// @param angle_units The units of the specified angle.
  /// @param motion_type The type of motion.
  /// @param calculation_option The calculation option.
  /// @return The relative number of microsteps.
  uint64_t CalculateRelativeMicrostepsToMoveByAngle(float angle, AngleUnits angle_units = AngleUnits::kDegrees,
                           MotionType motion_type = MotionType::kRelative, 
                           CalculationOption calculation_option = CalculationOption::kCalculateOnly);

  /// @brief Move to a target angle with respect to the; current angular position (relative), OR; zero/home angular position (absolute).
  /// @param angle The target angle (positive or negative).
  /// @param angle_units The units of the specified angle.
  /// @param motion_type The type of motion.
  /// @return The status of the motion operation.
  MotionStatus MoveByAngle(float angle, AngleUnits angle_units = AngleUnits::kDegrees,
                           MotionType motion_type = MotionType::kRelative); ///< This must be called periodically.

  /// @brief Move the motor indefinitely (jogging).
  void MoveByJogging(MotionDirection direction); ///< This must be called periodically.

  /// @brief Get the current angular position.
  /// @param angle_units The units required for the angle.
  /// @return The current angular position.
  double GetAngularPosition(AngleUnits angle_units) const;

  /// @brief Set the minimum time (us) for a low or high-level pulse of the PUL pin.
  /// @param pul_delay_us The minimum PUL low or high-level delay (us).
  void set_pul_delay_us(float pul_delay_us);

  /// @brief Set the minimum time (us) to delay after changing direction via the DIR pin.
  /// @param dir_delay_us The minimum DIR change delay (us).
  void set_dir_delay_us(float dir_delay_us);

  /// @brief Set the minimum time (us) to delay after changing the power state via the ENA pin.
  /// @param ena_delay_us The minimum ENA change delay (us).
  void set_ena_delay_us(float ena_delay_us);

  /// @brief Set the ENA/EN (enable) pin to control the power state (enable or disable) the motor.
  /// @param power_state The power state.
  void set_power_state(PowerState power_state);

  /// @brief Get the state of the ENA/EN (enable) pin to determine the power state of the motor.
  /// @return The power state.
  PowerState power_state() const;

 private:

  /// @brief The value of pi for math calculations.
  static const double kPi = 3.14159265358979323846;

  /// @brief Pulse the PUL/STP/CLK pin to move the motor by the minimum step based on the micro-stepping mode.
  void MoveByMicrostep(); ///< This must be called periodically.

  /// @brief Move the motor by the minimum step based on the micro-stepping mode, at speed based on the microstep period (us).
  /// @param operating_microstep_period_us The microstep period (us).
  void MoveByMicrostepAtMicrostepPeriod(double operating_microstep_period_us); ///< This must be called periodically.

  /// @brief Accelerate/decelerate the motor based on the speed period (us), by increasing/decreasing the speed based on the microstep period (us).
  /// @param motion_status The status of the motion operation.
  /// @param calculation_option The calculation option.
  void AccelerateOrDecelerateAtSpeedPeriod(CalculationOption calculation_option); ///< This must be called periodically.

  /// @{
  /// @brief Output pins.
  uint8_t pul_pin_; ///< PUL/STP/CLK (pulse/step/clock) pin.
  uint8_t dir_pin_; ///< DIR/CW (direction) pin.
  uint8_t ena_pin_; ///< ENA/EN (enable) pin.
  /// @}

  /// @{
  /// @brief Motor drive system properties.
  float full_step_angle_degrees_; ///< Motor full step angle in degrees.
  double gear_ratio_; ///< Gear ratio for motors coupled with a gearbox in the drive system.
  double microstep_angle_degrees_; ///< microstep angle = full step angle in degrees / (gear ratio x microstep mode)
  /// @}

  /// @{
  /// @brief Stepper driver properties.
  uint8_t microstep_mode_; ///< Microstep mode.
  float pul_delay_us_ = 2.5F; ///< Minimum time (us) to delay after a low or high-level pulse of the PUL pin.
  float dir_delay_us_ = 5.0F; ///< Minimum time (us) to delay after changing direction via the DIR pin.
  float ena_delay_us_ = 5.0F; ///< Minimum time (us) to delay after changing the power state via the ENA pin.
  /// @}

  /// @{
  /// @brief Motor states and targets.
  PowerState power_state_ = PowerState::kEnabled; ///< Power state based on the ENA/EN pin.
  double microstep_period_us_ = 100000.0; ///< Target speed based on the microstep period (us) between microsteps.
  double speed_in_flux_microsteps_per_us_; /// The speed (microsteps/us) that is changing due acceleration/deceleration.
  double microstep_period_in_flux_us_; // The microstep period (us) that is changing due to acceleration/deceleration.
  uint64_t reference_microstep_time_us_; ///< Reference time (us) for the microstep period.
  double speed_period_us_ = 0.0; ///< Target acceleration/deceleration based on the speed period (us) between increase/decrease of the microstep period.
  uint64_t reference_speed_time_us_; ///< Reference time (us) for the speed period.
  uint64_t angular_position_microsteps_ = 0; ///< The current angular position (microsteps).
  uint64_t relative_microsteps_to_move_ = 0; ///< Target number of microsteps to move the motor relative to the current angular position.
  int8_t angular_position_updater_microsteps_; ///< Value (microsteps) to increment/decrement the current angular position depending on motor motion direction based on the DIR/CW pin.
  uint64_t microsteps_after_acceleration_ = 0; ///< Expected number of microsteps remaining after acceleration has completed (microsteps).
  uint64_t microsteps_after_constant_speed_ = 0; ///< Expected number of microsteps remaining after constant speed motion has completed (microsteps).
  MotionStatus motion_status_ = MotionStatus::kIdle; ///< The status of the move (by angle) operation.
  MotionDirection jog_direction_ = MotionDirection::kNeutral; ///< The direction of the move (by jogging) operation.
  /// @}
};

} // namespace mt

#endif // STEPPER_DRIVER_H_