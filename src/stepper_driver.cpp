// Copyright (C) 2024 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file stepper_driver.h
/// @brief Class to control stepper motors via stepper motor drivers.

#include "stepper_driver.h"

#include <Arduino.h>

namespace mt {

StepperDriver::StepperDriver(uint16_t pul_pin, uint16_t dir_pin, uint16_t ena_pin, uint16_t microstep_mode,
                             float full_step_angle_degrees, float gear_ratio) {
  pul_pin_ = pul_pin;
  dir_pin_ = dir_pin;
  ena_pin_ = ena_pin;

  full_step_angle_degrees_ = full_step_angle_degrees;
  microstep_mode_ = microstep_mode;
  gear_ratio_ = gear_ratio;
  microstep_angle_degrees_ = full_step_angle_degrees_ / (gear_ratio_ * microstep_mode_);

  k180DividedByProductOfPiAndMicrostepAngleDegrees_ = 180.0 / (kPi_ * microstep_angle_degrees_);
  k6DividedByMicrostepAngleDegrees_ = 6.0 / microstep_angle_degrees_;
  k360DividedByMicrostepAngleDegrees_ = 360.0 / microstep_angle_degrees_;
  kPiTimesMicrostepAngleDegreesDivided180_ = (kPi_ * microstep_angle_degrees_) / 180.0;
  kMicrostepAngleDegreesDivided360_ = microstep_angle_degrees_ / 360.0;
}

StepperDriver::~StepperDriver() {}

void StepperDriver::SetSpeed(float speed, SpeedUnits speed_units) {
  speed_microsteps_per_s_ = 0.0;

  switch (speed_units) {
    case SpeedUnits::kMicrostepsPerSecond: {
      speed_microsteps_per_s_ = speed;
      break;
    }
    case SpeedUnits::kDegreesPerSecond: {
      speed_microsteps_per_s_ = speed / microstep_angle_degrees_;
      break;
    }
    case SpeedUnits::kRadiansPerSecond: {
      speed_microsteps_per_s_ = k180DividedByProductOfPiAndMicrostepAngleDegrees_ * speed;
      break;
    }
    case SpeedUnits::kRevolutionsPerMinute: {
      speed_microsteps_per_s_ = k6DividedByMicrostepAngleDegrees_ * speed;
      break;
    }
  }

  if (speed_microsteps_per_s_ == 0) {
    microstep_period_us_ = 0; // (us).
  }
  else {
    float microstep_period_us = 1000000.0 / (speed_microsteps_per_s_); // (us).
    microstep_period_us_ = microstep_period_us;
  }

  if (motion_status_ != MotionStatus::kIdle && motion_status_ != MotionStatus::kPaused) {
    // Speed changed mid-motion.
    angle_after_acceleration_microsteps_ = 0; // Indicates speed profile should be recalculated.
    motion_status_ = MotionStatus::kAccelerate;
  }
}

void StepperDriver::SetAcceleration(float acceleration, AccelerationUnits acceleration_units) {
  acceleration_microsteps_per_s_per_s_ = 0.0;

  switch (acceleration_units) {
    case AccelerationUnits::kMicrostepsPerSecondPerSecond: {
      acceleration_microsteps_per_s_per_s_ = acceleration;
      break;
    }
    case AccelerationUnits::kDegreesPerSecondPerSecond: {
      acceleration_microsteps_per_s_per_s_ = acceleration / microstep_angle_degrees_;
      break;
    }
    case AccelerationUnits::kRadiansPerSecondPerSecond: {
      acceleration_microsteps_per_s_per_s_ = k180DividedByProductOfPiAndMicrostepAngleDegrees_ * acceleration;
      break;
    }
    case AccelerationUnits::kRevolutionsPerMinutePerMinute: {
      acceleration_microsteps_per_s_per_s_ = k6DividedByMicrostepAngleDegrees_ * acceleration;
      break;
    }
  }

  if (acceleration_algorithm_ == AccelerationAlgorithm::kEiderman04) R_ = acceleration_microsteps_per_s_per_s_ / fsquared_; // Eiderman '04, Equation 19.

  if (motion_status_ != MotionStatus::kIdle && motion_status_ != MotionStatus::kPaused) {
    // Acceleration changed mid-motion.
    angle_after_acceleration_microsteps_ = 0; // Indicates speed profile should be recalculated.
    motion_status_ = MotionStatus::kAccelerate;
  }
}

uint32_t StepperDriver::CalculateRelativeMicrostepsToMoveByAngle(float angle, AngleUnits angle_units,
                                                                 MotionType motion_type,
                                                                 CalculationOption calculation_option) {
  float angle_microsteps = 0.0;

  switch (angle_units) {
    case AngleUnits::kMicrosteps: {
      angle_microsteps = angle;
      break;
    }
    case AngleUnits::kDegrees: {
      angle_microsteps = angle / microstep_angle_degrees_;
      break;
    }
    case AngleUnits::kRadians: {
      angle_microsteps = k180DividedByProductOfPiAndMicrostepAngleDegrees_ * angle;
      break;
    }
    case AngleUnits::kRevolutions: {
      angle_microsteps = k360DividedByMicrostepAngleDegrees_ * angle;
      break;
    }
  }

  int32_t relative_angle_microsteps = 0; // Always zero for other Motion Types: Stop And Reset, Pause and Resume.

  switch (motion_type) {
    case MotionType::kAbsolute: {
      // Microsteps required to move to given angular position.
      relative_angle_microsteps = static_cast<int32_t>(angle_microsteps) - angular_position_microsteps_;
      break;
    }
    case MotionType::kRelative: {
      // Microsteps required to move by given angular amount.
      relative_angle_microsteps = static_cast<int32_t>(angle_microsteps);
      break;
    }
  }

  if (calculation_option == CalculationOption::kSetupMotion) {
    if (relative_angle_microsteps < 0) {
      // Negative motion direction.
      angular_position_updater_microsteps_ = -1;
      digitalWrite(dir_pin_, static_cast<uint8_t>(dir_pin_negative_direction_state_));
    }
    else if (relative_angle_microsteps > 0) {
      // Positive motion direction.
      angular_position_updater_microsteps_ = 1;
      digitalWrite(dir_pin_, static_cast<uint8_t>(dir_pin_positive_direction_state_));
    }

    delayMicroseconds(dir_delay_us_);
  }

	return abs(relative_angle_microsteps);
}

StepperDriver::MotionStatus StepperDriver::MoveByAngle(float angle, AngleUnits angle_units, MotionType motion_type) {
  if (power_state_ == PowerState::kDisabled || microstep_period_us_ == 0) {
    // Stop & reset if ENA pin is disabled or speed set to 0.
    motion_type = MotionType::kStopAndReset;
  }
 
  switch (motion_type) {
    case MotionType::kStopAndReset: {
      // Reset relative_angle_to_move_microsteps_, relative_angle_to_move_in_flux_microsteps_, angle_after_acceleration_microsteps_, n_, i_.
      ResetAccelerationParameters();
      motion_status_ = MotionStatus::kIdle;
      break;
    }
    case MotionType::kPause: {
      motion_status_ = MotionStatus::kPaused;
      break;
    }
    case MotionType::kResume: {
      [[fallthrough]];
    }
    case MotionType::kAbsolute: {
      [[fallthrough]];
    }
    case MotionType::kRelative: {
      if (motion_status_ == MotionStatus::kIdle || motion_status_ == MotionStatus::kPaused) {
        angle_after_acceleration_microsteps_ = 0; // Indicates speed profile should be recalculated.

        if (motion_status_ == MotionStatus::kIdle) {
          // Reset relative_angle_to_move_microsteps_, relative_angle_to_move_in_flux_microsteps_, angle_after_acceleration_microsteps_, n_, i_.
          ResetAccelerationParameters();
          relative_angle_to_move_microsteps_ = CalculateRelativeMicrostepsToMoveByAngle(angle, angle_units, motion_type,
                                                                                CalculationOption::kSetupMotion);
          relative_angle_to_move_in_flux_microsteps_ = relative_angle_to_move_microsteps_;                                                                                
        }

        motion_status_ = MotionStatus::kAccelerate;
      }

      break;
    }
  }

  switch (motion_status_) {
    case MotionStatus::kIdle: {
      break;
    }
    case MotionStatus::kPaused: {
      break;
    }
    case MotionStatus::kAccelerate: {
      if (acceleration_microsteps_per_s_per_s_ == 0) {
        // No acceleration/deceleration. Constant speed only.
        K_ = 0;
        m_ = 0;
        angle_after_constant_speed_microsteps_ = 0;
        motion_status_ = MotionStatus::kConstantSpeed;
      }
      else if (angle_after_acceleration_microsteps_ == 0) {
        // Recalculate speed profiles for acceleration/deceleration.
        K_ = 1;
        m_ = -R_;        
        // Calculate the minimum microsteps required to accelerate to; and decelerate from; the set speed.
        uint32_t min_microsteps_for_acceleration = static_cast<uint32_t>((speed_microsteps_per_s_ * speed_microsteps_per_s_)
                                                         / (2.0 * acceleration_microsteps_per_s_per_s_)); // (microsteps).
        if (relative_angle_to_move_microsteps_ <= (2 * min_microsteps_for_acceleration)) {
          // Setup triangular speed profile; motor will accelerate to achievable speed (<= set speed) for available microsteps, then decelerate to 0.
          angle_after_acceleration_microsteps_ = static_cast<uint32_t>(relative_angle_to_move_microsteps_ / 2.0);
          angle_after_constant_speed_microsteps_ = 0;
          //speed_achievable_microsteps_per_s_ = sqrt(2.0 * acceleration_microsteps_per_s_per_s_ * relative_angle_to_move_microsteps_);
        }
        else {
          // Setup trapezoidal speed profile; motor will accelerate to set speed, move at constant speed, then decelerate to 0.
          angle_after_acceleration_microsteps_ = relative_angle_to_move_microsteps_ - min_microsteps_for_acceleration;
          angle_after_constant_speed_microsteps_ = min_microsteps_for_acceleration;
          //speed_achievable_microsteps_per_s_ = speed_microsteps_per_s_;
        }
      }
      else {
        // Acceleration already in progress.
        if (relative_angle_to_move_in_flux_microsteps_ <= angle_after_acceleration_microsteps_) {
        //if (microstep_period_in_flux_us_ <= microstep_period_us_) {
          // Finished acceleration.
          if (angle_after_constant_speed_microsteps_ == 0) {
            // Triangular speed profile. Setup deceleration.
            K_ = -1;
            n_ = -angle_after_acceleration_microsteps_;
            m_ = R_;
            motion_status_ = MotionStatus::kDecelerate;
          }
          else {
            // Trapezoidal speed profile. Setup constant speed motion.
            K_ = 0;
            m_ = 0;
            motion_status_ = MotionStatus::kConstantSpeed;
          }
        }
        else {
          //Accelerate.
          MoveByMicrostepAtMicrostepPeriodInFlux();
        }
      }

      break;
    }
    case MotionStatus::kConstantSpeed: {
      if (relative_angle_to_move_in_flux_microsteps_ <= angle_after_constant_speed_microsteps_) {
        // Finished constant speed motion.
        if (angle_after_constant_speed_microsteps_ == 0) {
          // No acceleration/deceleration. Indicate motion complete.
          motion_status_ = MotionStatus::kIdle;
        }
        else {
          // Trapezoidal speed profile. Setup deceleration.
          K_ = -1;
          n_ = -angle_after_constant_speed_microsteps_;
          m_ = R_;
          motion_status_ = MotionStatus::kDecelerate;
        }
      }
      else {
        MoveByMicrostepAtMicrostepPeriodInFlux();
      }

      break;
    }
    case MotionStatus::kDecelerate: {
      if (relative_angle_to_move_in_flux_microsteps_ == 0) {
        // Finished decelerating. Indicate motion complete.
        motion_status_ = MotionStatus::kIdle;
      }
      else {
        // Decelerate.
        MoveByMicrostepAtMicrostepPeriodInFlux();       
      }

      break;
    }
  }

  if (debug_enabled_ == true) DebugHelperForMoveByAngle(); // Debugging.
  return motion_status_;
}

void StepperDriver::MoveByJogging(MotionDirection direction) {
  if (power_state_ == PowerState::kDisabled || microstep_period_us_ == 0) return;

  if (jog_direction_ != direction) {
    // Direction has changed.
    jog_direction_ = direction;

    if (jog_direction_ == MotionDirection::kNegative) {
      digitalWrite(dir_pin_, static_cast<uint8_t>(dir_pin_negative_direction_state_)); 
    }
    else if (jog_direction_ == MotionDirection::kPositive) {
      digitalWrite(dir_pin_, static_cast<uint8_t>(dir_pin_positive_direction_state_)); 
    }

    delayMicroseconds(dir_delay_us_); 
  }

  if (jog_direction_ == MotionDirection::kNeutral) return;
  
  MoveByMicrostepAtMicrostepPeriod();
}

float StepperDriver::GetAngularPosition(AngleUnits angle_units) const {
  float angular_position = 0.0;

  switch (angle_units) {
    case AngleUnits::kMicrosteps: {
      angular_position = angular_position_microsteps_;
      break;
    }
    case AngleUnits::kDegrees: {
      angular_position = angular_position_microsteps_ * microstep_angle_degrees_;
      break;
    }
    case AngleUnits::kRadians: {
      angular_position = angular_position_microsteps_ * kPiTimesMicrostepAngleDegreesDivided180_;
      break;
    }
    case AngleUnits::kRevolutions: {
      angular_position = angular_position_microsteps_ * kMicrostepAngleDegreesDivided360_;
      break;
    }
  }

  return angular_position;
}

void StepperDriver::set_acceleration_algorithm(AccelerationAlgorithm acceleration_algorithm) {
  acceleration_algorithm_ = acceleration_algorithm;
  if (acceleration_algorithm_ == AccelerationAlgorithm::kEiderman04) R_ = acceleration_microsteps_per_s_per_s_ / fsquared_; // Eiderman '04, Equation 19.
  // Reset relative_angle_to_move_microsteps_, relative_angle_to_move_in_flux_microsteps_, angle_after_acceleration_microsteps_, n_, i_.
  ResetAccelerationParameters();
}

void StepperDriver::set_ena_pin_enabled_state(PinState ena_pin_enabled_state) {
  ena_pin_enabled_state_ = ena_pin_enabled_state;
  if (ena_pin_enabled_state_ == PinState::kHigh) {
    ena_pin_disabled_state_ = PinState::kLow; 
  }
  else { 
    ena_pin_disabled_state_ = PinState::kHigh; 
  }
}

void StepperDriver::set_dir_pin_positive_direction_state(PinState dir_pin_positive_direction_state) {
  dir_pin_positive_direction_state_ = dir_pin_positive_direction_state;
  if (dir_pin_positive_direction_state_ == PinState::kHigh) {
    dir_pin_negative_direction_state_ = PinState::kLow;
  }
  else {
    dir_pin_negative_direction_state_ =PinState::kHigh;
  }
}

void StepperDriver::set_pul_delay_us(float pul_delay_us) {
  pul_delay_us_ = pul_delay_us;
}

void StepperDriver::set_dir_delay_us(float dir_delay_us) {
  dir_delay_us_ = dir_delay_us;
}

void StepperDriver::set_ena_delay_us(float ena_delay_us) {
  ena_delay_us_ = ena_delay_us;
}

void StepperDriver::set_power_state(PowerState power_state) {
  power_state_ = power_state;
  if (power_state_ == PowerState::kEnabled) {
    digitalWrite(ena_pin_, static_cast<uint8_t>(ena_pin_enabled_state_));    
  }
  else {
    digitalWrite(ena_pin_, static_cast<uint8_t>(ena_pin_disabled_state_));
  }

  delayMicroseconds(ena_delay_us_);
}

StepperDriver::PowerState StepperDriver::power_state() const { return power_state_; }

void StepperDriver::MoveByMicrostep() {
  digitalWrite(pul_pin_, LOW);
  delayMicroseconds(pul_delay_us_);
  digitalWrite(pul_pin_, HIGH);
  delayMicroseconds(pul_delay_us_);

  if (relative_angle_to_move_in_flux_microsteps_ != 0) {
    // Move by angle (not by jogging) is in operation.
    relative_angle_to_move_in_flux_microsteps_--;
  }

  angular_position_microsteps_ = angular_position_microsteps_ + angular_position_updater_microsteps_;
}

void StepperDriver::MoveByMicrostepAtMicrostepPeriod() {
  uint64_t current_time_us = micros();
  if ((current_time_us - reference_microstep_time_us_) >= microstep_period_us_) {
    MoveByMicrostep();
    reference_microstep_time_us_ = current_time_us;
  }
}

void StepperDriver::CalculateMicrostepPeriodInFlux() {
  switch (acceleration_algorithm_) {
    case AccelerationAlgorithm::kMorgridge24: {
      // K_ = 1 for acceleration, 0 in-between, -1 for deceleration.
      if (K_ == 0) {
        // Constant speed.
        if (microstep_period_in_flux_us_ != microstep_period_us_) { // Only execute once if needed, so as not to waste resources.
          vi_microsteps_per_s_ = speed_microsteps_per_s_;
          Ti_us_ = microstep_period_us_;
          microstep_period_in_flux_us_ = microstep_period_us_;
        }
      }
      else {
        // Acceleration/deceleration.
        if(i_ == 1) {
          // From stand-still. Calculate the speed/microstep period for i = 1.
          vi_microsteps_per_s_ = acceleration_microsteps_per_s_per_s_ * sqrt(2.0 / acceleration_microsteps_per_s_per_s_); // Morgridge '24, Equation 30.
        }
        else {
          // Already accelerating/decelerating.
          vi_microsteps_per_s_ = vi_microsteps_per_s_ + (K_ * (acceleration_microsteps_per_s_per_s_ / vi_microsteps_per_s_)); // Morgridge '24, Equation 31.
        }

        Ti_us_ = 1000000.0 / vi_microsteps_per_s_; // Morgridge '24, Equation 32.
        microstep_period_in_flux_us_ = Ti_us_;
      }

      i_++;
      break;
    }
    case AccelerationAlgorithm::kAustin05: {
      // Austin's algorithm doesn't have a concept of motion phase multiplier but this is used here to aid compatibility with the other algorithms.
      // K_ = 1 for acceleration, 0 in-between, -1 for deceleration.
      if (K_ == 0) {
        // Constant speed.
        if (microstep_period_in_flux_us_ != microstep_period_us_) { // Only execute once if needed, so as not to waste resources.
          Cn_ = microstep_period_us_;
          microstep_period_in_flux_us_ = microstep_period_us_;
        }    
      }
      else {
        if(n_ == 0) {
          // From stand-still. Calculate the speed/microstep period for n = 0.
          Cn_ = 0.676 * f_ * sqrt(2.0 / acceleration_microsteps_per_s_per_s_); // Austin '05, Equation 15.
        }
        else {
          // Already accelerating/decelerating. n > 0 for acceleration, n < 0 for deceleration.
          Cn_ = Cn_ - ((2.0 * Cn_) / ((4.0 * n_) + 1)); // Austin '05, Equation 13.
        }

        microstep_period_in_flux_us_ = Cn_;
      }

      n_++;
      break;
    }
    case AccelerationAlgorithm::kEiderman04: {
      // m_ = -R_ for acceleration, 0 in-between, R_ for deceleration.
      if (m_ == 0) {
        // Constant speed.
        if (microstep_period_in_flux_us_ != microstep_period_us_) { // Only execute once if needed, so as not to waste resources.
          p_ = microstep_period_us_;
          microstep_period_in_flux_us_ = microstep_period_us_;
        }    
      }
      else {
        if(i_ == 1) {
          // From stand-still. Calculate the speed/microstep period for i = 1.
          p_ = f_ / sqrt((v0_ * v0_) + (2.0 * acceleration_microsteps_per_s_per_s_)); // Eiderman '04, Equation 17.
        }
        else {
          // Already accelerating/decelerating.
          q_ = m_ * p_ * p_;
          p_ = p_ * (1 + q_); // Eiderman '04, Equation 20.
          //p_ = p_ * (1 + q_ + (q_ * q_)); // Eiderman '04, Equation 23.
          //p_ = p_ * (1 + q_ + (1.5 * q_ * q_)); // Eiderman '04, Equation 22.
        }

        microstep_period_in_flux_us_ = p_;
      }

      i_++;
      break;
    }
  }
}

void StepperDriver::MoveByMicrostepAtMicrostepPeriodInFlux() {
  uint64_t current_time_us = micros();
  if (current_time_us - reference_microstep_flux_time_us_ >= microstep_period_in_flux_us_) {
    MoveByMicrostep();
    CalculateMicrostepPeriodInFlux();
    reference_microstep_flux_time_us_ = current_time_us;
  }
}

void StepperDriver::ResetAccelerationParameters() {
  relative_angle_to_move_microsteps_ = 0;
  relative_angle_to_move_in_flux_microsteps_ = 0;
  angle_after_acceleration_microsteps_ = 0; // Indicates speed profile should be recalculated.
  n_ = 0;
  i_ = 1; 
}

void StepperDriver::DebugHelperForMoveByAngle() {
#if 0 // 0 to disable debug outputs, 1 to enable debug ouputs.
  if (motion_status_ == MotionStatus::kAccelerate) {
    if (debug_helper_accel_initial_vars_printed_ == false) {
      Serial.print(F("Set microstep period (us): ")); Serial.println(microstep_period_us_);
      Serial.print(F("Set acceleration (microsteps/us^2): ")); Serial.println(acceleration_microsteps_per_s_per_s_);

      Serial.print(F("Total relative angle (microsteps) to move: ")); Serial.println(relative_angle_to_move_microsteps_);

      if (acceleration_microsteps_per_s_per_s_ == 0) {
        Serial.print(F("___Constant speed only___"));
      }
      else if (angle_after_constant_speed_microsteps_ == 0) {
        Serial.println(F("___Triangular speed profile___ :"));
        Serial.print(F("Angle (microsteps) after acceleration: ")); Serial.println(angle_after_acceleration_microsteps_);
        Serial.println(F("Starting acceleration."));
      }
      else {
        Serial.println(F("___Trapezoidal speed profile___"));        
        Serial.print(F("Angle (microsteps) after acceleration: ")); Serial.println(angle_after_acceleration_microsteps_);
        Serial.print(F("Angle (microsteps) after constant speed: ")); Serial.println(angle_after_constant_speed_microsteps_);        
        Serial.println(F("Starting acceleration."));
      }

      debug_helper_accel_initial_vars_printed_ = true;
    }
  }
  else if (motion_status_ == MotionStatus::kConstantSpeed) {
    // Constant speed only OR trapezoidal speed profile.
    if (debug_helper_cspeed_initial_vars_printed_ == false) {
      Serial.println(F("Constant speed phase."));
      Serial.print(F("Microstep period (us) reached: ")); Serial.println(microstep_period_in_flux_us_);

      debug_helper_cspeed_initial_vars_printed_ = true;
    }
  }
  else if (motion_status_ == MotionStatus::kDecelerate) {
    // Triangular or trapezoidal speed profiles.
    if (debug_helper_cspeed_initial_vars_printed_ == false) {
      Serial.print(F("Starting deceleration."));

      if (acceleration_microsteps_per_s_per_s_ != 0 && angle_after_constant_speed_microsteps_ == 0) {
        // Triangular speed profile.
        Serial.print(F("Microstep period (us) reached: ")); Serial.println(microstep_period_in_flux_us_);
      }

      debug_helper_decel_initial_vars_printed_ = true;
    }
  }

    // Print acceleration/deceleration values.
    if (angle_after_acceleration_microsteps_ != 0 && K_ != 0) {
      switch (acceleration_algorithm_) {
        case AccelerationAlgorithm::kMorgridge24: {
          //Serial.print(F("v")); Serial.print(i_); Serial.print(F(" = ")); Serial.println(vi_microsteps_per_s_);
          //Serial.print(F("T")); Serial.print(i_); Serial.print(F(" = ")); Serial.println(Ti_us_);
          break;
        }
        case AccelerationAlgorithm::kAustin05: {
          //Serial.print(F("C")); Serial.print(n_); Serial.print(F(" = ")); Serial.println(Cn_);
          break;
        }
        case AccelerationAlgorithm::kEiderman04: {
          //Serial.print(F("p")); Serial.print(i_); Serial.print(F(" = ")); Serial.println(p_);
          break;
        }
      }

      //Serial.print(F("microstep_period_in_flux_us_")); Serial.print(n_); Serial.print(F(" = ")); Serial.println(microstep_period_in_flux_us_);
    }
#endif    
}

} // namespace mt