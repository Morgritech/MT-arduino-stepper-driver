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

  /// @brief Enum of acceleration algorithms.
  enum class AccelerationAlgorithm {
    kMorgridge24 = 1,
    kAustin05 = 2,
    kEiderman04 = 3,
  };

  /// @brief Enum of GPIO pin states.
  enum class PinState {
    kLow = 0,
    kHigh,
  };

  /// @brief Enum of power states based on the ENA/EN pin.
  enum class PowerState {
    kDisabled = 0,
    kEnabled,
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
  StepperDriver(uint16_t pul_pin, uint16_t dir_pin, uint16_t ena_pin, uint16_t microstep_mode = 1,
                float full_step_angle_degrees = 1.8, float gear_ratio = 1);

  /// @brief Destroy the Stepper Driver object.
  ~StepperDriver();

  /// @brief Set the target speed at which to move the motor.
  /// @param speed The target speed.
  /// @param speed_units The units of the specified speed.
  void SetSpeed(float speed, SpeedUnits speed_units = SpeedUnits::kRevolutionsPerMinute);

  /// @brief Set the target acceleration/deceleration for the motor to accelerate to the target speed and decelerate to a stop.
  /// @param acceleration The target acceleration/deceleration.
  /// @param acceleration_units The units of the specified acceleration.
  void SetAcceleration(float acceleration, 
                       AccelerationUnits acceleration_units = AccelerationUnits::kRadiansPerSecondPerSecond);

  /// @brief Calculate the relative number of microsteps to move to a target angle with respect to the; current angular position (relative), OR; zero/home angular position (absolute).
  /// @param angle The target angle (positive or negative).
  /// @param angle_units The units of the specified angle.
  /// @param motion_type The type of motion.
  /// @param calculation_option The calculation option.
  /// @return The relative number of microsteps.
  uint32_t CalculateRelativeMicrostepsToMoveByAngle(float angle, AngleUnits angle_units = AngleUnits::kDegrees,
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
  float GetAngularPosition(AngleUnits angle_units) const;

  /// @brief Set the acceleration algorithm to be used for acceleration/deceleration.
  /// @param acceleration_algorithm The algorithm.
  void set_acceleration_algorithm(AccelerationAlgorithm acceleration_algorithm);

  /// @brief Set the ENA pin state when the motor is enabled (powered).
  /// @param ena_pin_enabled_state The pin state.
  void set_ena_pin_enabled_state(PinState ena_pin_enabled_state);

  /// @brief Set the DIR pin state for positive motion direction.
  /// @param dir_pin_positive_direction_state The pin state.
  void set_dir_pin_positive_direction_state(PinState dir_pin_positive_direction_state);

  /// @brief Set the minimum time (us) for a low or high-level pulse of the PUL pin.
  /// @param pul_delay_us The minimum PUL low or high-level delay (us).
  void set_pul_delay_us(float pul_delay_us);

  /// @brief Set the minimum time (us) to delay after changing direction via the DIR pin.
  /// @param dir_delay_us The minimum DIR change delay (us).
  void set_dir_delay_us(float dir_delay_us);

  /// @brief Set the minimum time (us) to delay after changing the power state via the ENA pin.
  /// @param ena_delay_us The minimum ENA change delay (us).
  void set_ena_delay_us(float ena_delay_us);

  /// @brief Set the ENA/EN (enable) pin to control the power state (enable or disable) of the motor.
  /// @param power_state The power state.
  void set_power_state(PowerState power_state);

  /// @brief Get the state of the ENA/EN (enable) pin to determine the power state of the motor.
  /// @return The power state.
  PowerState power_state() const;

 private:

  /// @brief The value of pi for math calculations.
  static const double kPi_ = 3.14159265358979323846;

  /// @brief Pulse the PUL/STP/CLK pin to move the motor by the minimum step based on the micro-stepping mode.
  void MoveByMicrostep();

  /// @brief Move the motor by the minimum step based on the micro-stepping mode, at speed based on the set microstep period (us).
  void MoveByMicrostepAtMicrostepPeriod();

  /// @brief Calculate the ramp speed based on the microstep period in flux (us), during an acceleration/deceleration.
  void CalculateMicrostepPeriodInFlux();

  /// @brief Move the motor by the minimum step based on the micro-stepping mode, at the speed based on the microstep period (us), that is changing due to acceleration/deceleration.
  void MoveByMicrostepAtMicrostepPeriodInFlux();

  /// @brief Reset parameters used for acceleration/deceleration.
  void ResetAccelerationParameters();

  /// @brief Helper for printing out debugging information.
  void DebugHelperForMoveByAngle();

  /// @{
  /// @brief Pre-calculated unit conversion constants.
  double k180DividedByProductOfPiAndMicrostepAngleDegrees_; ///< 180 / (pi x microstep angle in degrees)
  float k6DividedByMicrostepAngleDegrees_; ///< 6 / microstep angle in degrees
  float k360DividedByMicrostepAngleDegrees_; ///< 360 / microstep angle in degrees
  double kPiTimesMicrostepAngleDegreesDivided180_; ///< (pi x microstep angle in degrees) / 180
  float kMicrostepAngleDegreesDivided360_; ///< microstep angle in degrees / 360

  /// @brief Default acceleration algorithm.
  AccelerationAlgorithm acceleration_algorithm_ = AccelerationAlgorithm::kMorgridge24;

  /// @{
  /// @brief Output pins.
  uint16_t pul_pin_; ///< PUL/STP/CLK (pulse/step/clock) pin.
  uint16_t dir_pin_; ///< DIR/CW (direction) pin.
  uint16_t ena_pin_; ///< ENA/EN (enable) pin.
  /// @}

  /// @{
  /// @brief Default pin states.
  PinState ena_pin_enabled_state_ = PinState::kLow; ///< The ENA pin state when the motor is enabled (powered).
  PinState ena_pin_disabled_state_ = PinState::kHigh; ///< The ENA pin state when motor is disabled (not powered).
  PinState dir_pin_positive_direction_state_ = PinState::kHigh; ///< The DIR pin state for positive motion direction.
  PinState dir_pin_negative_direction_state_ = PinState::kLow; ///< The DIR pin state for negative motion direction.
  /// @}

  /// @{
  /// @brief Motor drive system properties.
  float full_step_angle_degrees_; ///< Motor full step angle in degrees.
  float gear_ratio_; ///< Gear ratio for motors coupled with a gearbox in the drive system.
  float microstep_angle_degrees_; ///< microstep angle = full step angle in degrees / (gear ratio x microstep mode)
  /// @}

  /// @{
  /// @brief Stepper driver properties.
  uint16_t microstep_mode_; ///< Microstep mode.
  uint8_t pul_delay_us_ = 1; ///< Minimum time (us) to delay after a low or high-level pulse of the PUL pin.
  uint8_t dir_delay_us_ = 5; ///< Minimum time (us) to delay after changing direction via the DIR pin.
  uint8_t ena_delay_us_ = 5; ///< Minimum time (us) to delay after changing the power state via the ENA pin.
  /// @}

  /// @{
  /// @brief Motor states and targets.
  /// Power.
  PowerState power_state_ = PowerState::kEnabled; ///< Power state based on the ENA/EN pin.
  /// Position/distance.
  uint32_t angular_position_microsteps_ = 0; ///< The current angular position (microsteps).
  uint32_t relative_angle_to_move_microsteps_ = 0; ///< Target distance/angle (microsteps) to move the motor relative to the current angular position.
  uint32_t relative_angle_to_move_in_flux_microsteps_ = 0; ///< The distance/angle (microsteps) to move that is reducing due to the motor moving.
  int8_t angular_position_updater_microsteps_ = 1; ///< Value (microsteps) to increment/decrement the current angular position depending on motor motion direction based on the DIR/CW pin.
  uint32_t angle_after_acceleration_microsteps_ = 0; ///< Expected distance/angle (microsteps) remaining after acceleration has completed.
  uint32_t angle_after_constant_speed_microsteps_ = 0; ///< Expected distance/angle (microsteps) remaining after constant speed motion has completed.
  /// Speed.
  float speed_microsteps_per_s_ = 0.0; ///< Target speed (microsteps/s).
  //float speed_achievable_microsteps_per_s_ = 0.0; ///< Achievable speed based on the set acceleration and travel distance/angle (microsteps/s).
  uint32_t microstep_period_us_ = 100000.0; ///< Target speed based on the microstep period (us) between microsteps.
  float vi_microsteps_per_s_ = 0.0; ///< ith speed (microsteps/s), used to calculate Ti_us_. Morgridge '24.
  float Ti_us_ = 0.0; ///< ith microstep period (us), used to set the microstep_period_in_flux_us. Morgridge '24.
  float f_ = 1000000.0; ///< Timer frequency (count of timer ticks per sec) (Hz). Austin '05/Eiderman '04.
  double fsquared_ = f_ * f_; ///< Eiderman '04.
  float Cn_ = 0.0; ///< nth speed (us), used to set microstep_period_in_flux_us. Austin '05.
  float p_ = 0.0; ///< ith speed (us), used to set microstep_period_in_flux_us. Eiderman '04.
  float v0_ = 0.0; ///< Base speed (microsteps/s) used to calculate the initial value of p. Eiderman '04.
  uint32_t microstep_period_in_flux_us_ = 0.0; ///< The microstep period (us) that is changing due to acceleration/deceleration.
  uint64_t reference_microstep_time_us_ = 0; ///< Reference time (us) for the microstep period.
  /// Acceleration.
  float acceleration_microsteps_per_s_per_s_ = 0.0; ///< Target acceleration (microsteps/s^2).
  uint64_t reference_microstep_flux_time_us_ = 0; ///< Reference time (us) for the microstep period in flux.
  // Motion phase constants.
  int8_t K_ = 0; ///< Constant multiplier. (+1 for acceleration, 0 in-between, -1 for deceleration). Morgridge '24.
  float R_ = 0.0; ///< Constant multiplier. Eiderman '04.
  float m_ = 0.0; ///< Variable multiplier that depends on movement phase (m_ = -R_ for acceleration, 0 in-between, R_ for deceleration). Eiderman '04.
  /// Other.
  MotionStatus motion_status_ = MotionStatus::kIdle; ///< The status of the move (by angle) operation.
  MotionDirection jog_direction_ = MotionDirection::kNeutral; ///< The direction of the move (by jogging) operation.
  uint32_t i_ = 1; ///< Iteration counter. Morgridge '24/Eiderman '04.
  float q_ = 0.0; ///< Variable to calculate a more accurate value of p at the expense of processing overhead (i.e., slower). Eiderman '04.
  int32_t n_ = 0; ///< Iteration counter. Also depends on movement phase (n > 0 for acceleration, n < 0 for deceleration). Austin '05.
  /// Debug helpers.
  bool debug_enabled_ = false; ///< Flag to control whether debug outputs are printed.
  bool debug_helper_accel_initial_vars_printed_ = false; ///< Flag to aid in printing initial debug outputs during acceleration only once.
  bool debug_helper_cspeed_initial_vars_printed_ = false; ///< Flag to aid in printing initial debug outputs during constant speed only once.
  bool debug_helper_decel_initial_vars_printed_ = false; ///< Flag to aid in printing initial debug outputs during deceleration speed only once.
  /// @}
};

} // namespace mt

#endif // STEPPER_DRIVER_H_