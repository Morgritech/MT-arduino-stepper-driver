// Copyright (C) 2024 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file move-by-jogging-with-serial.ino
/// @brief Example showing how to move a stepper motor by sending start/stop commands over serial, using the MT-arduino-stepper-driver library.

#include <stepper_driver.h>

/// @{
/// @brief GPIO pins.
const uint16_t kPulPin = 11; ///< For the stepper driver PUL/STP/CLK (pulse/step) interface.
const uint16_t kDirPin = 12; ///< For the stepper driver DIR/CW (direction) interface.
const uint16_t kEnaPin = 13; ///< For the stepper driver ENA/EN (enable) interface.
/// @}

/// @{
/// @brief Serial properties.
const int kBaudRate = 9600; ///< The serial communication speed.
/// Serial messages.
const char kToggleMotionMessage = 'm'; ///< Toggle (start/stop) the motor.
const char kToggleDirectionMessage = 'd'; ///< Change motor direction.
const char kIncreaseSpeedMessage = '+'; ///< Increase motor speed.
const char kDecreaseSpeedMessage = '-'; ///< Decrease motor speed.
/// @}

/// @{
/// @brief Stepper motor/drive system properties.
const float kFullStepAngle_degrees = 1.8F; ///< The full step angle in degrees. Obtained from the motor data sheet.
const float kGearRatio = 1.0F; ///< The gear ratio (1 if not using a gearing system or a geared stepper motor).
/// @}

/// @{
/// @brief Stepper driver properties.
const uint16_t kMicrostepMode = 8; ///< Microstep mode (1/8). Remember to change the setting on the stepper driver to match.
/// Minimum time (us) to delay after changing the state of a pin. Obtained from the stepper driver data sheet.
/// These values are for the StepperOnline DM542T stepper driver, but should work for most stepper drivers.
const float kPulDelay_us = 2.5F; ///< For the PUL pin.
const float kDirDelay_us = 5.0F; ///< For the Dir pin.
const float kEnaDelay_us = 5.0F; ///< For the Ena pin.
/// Speed.
float kSpeed_RPM = 20.0; ///< Rotation speed (RPM).
const float kSpeedUpdater_RPM = 1.0F; ///< Amount (RPM) by which to increase/decrease speed when a speed message is received over serial.
/// @}

/// @{
/// @brief Other properties.
const uint16_t kStartupTime_ms = 1000; ///< Minimum startup/boot time in milliseconds (ms); based on the stepper driver.
/// @}

/// @brief Stepper Driver instance for the stepper motor.
mt::StepperDriver stepper_driver(kPulPin, kDirPin, kEnaPin, kMicrostepMode, kFullStepAngle_degrees, kGearRatio);
//mt::StepperDriver stepper_driver(kPulPin, kDirPin, kEnaPin); // Default values are used for: microstep mode = 1, full step angle = 1.8, and gear ratio = 1.  

/// @brief The main application entry point for initialisation tasks.
void setup() {
  // Initialise the Serial Port.
  Serial.begin(kBaudRate);

  // Initialise the input pins.
  pinMode(kDirectionButtonPin, INPUT);
  pinMode(kMoveButtonPin, INPUT);

  // Initialise the output pins.
  pinMode(kPulPin, OUTPUT);
  pinMode(kDirPin, OUTPUT);
  pinMode(kEnaPin, OUTPUT);

  // Set stepper driver properties.
  // If these are not set, default values from the library will be used.
  stepper_driver.set_pul_delay_us(kPulDelay_us);
  stepper_driver.set_dir_delay_us(kDirDelay_us);
  stepper_driver.set_ena_delay_us(kEnaDelay_us);
  stepper_driver.SetSpeed(kSpeed_RPM, mt::StepperDriver::SpeedUnits::kRevolutionsPerMinute);

  // Activate the stepper driver.
  //stepper_driver.set_power_state(mt::StepperDriver::PowerState::kEnabled)// This is usually activated by default, hence this may not be required.

  // Delay for the startup time.
  delay(kStartupTime_ms);

  Serial.println(F("\n...Setup complete...\n"));
}

/// @brief The continuously running function for repetitive tasks.
void loop() {
  // Variable to hold the serial message received.
  static char serial_message;
  // Flag to keep track of when to move the motor based on a start/stop message received over serial.
  static bool move_motor = false;
  // Variable to keep track of the motion direction based on a direction message received over serial.
  static mt::StepperDriver::MotionDirection motion_direction = mt::StepperDriver::MotionDirection::kPositive;

  // Check for and process serial messages, one character at a time.
  if (Serial.available() > 0) {
    serial_message = Serial.read();
    //Serial.print(serial_message);

    switch(serial_message) {
      case kToggleMotionMessage: {
        if (move_motor == false) {
          move_motor = true;
          Serial.println(F("Start moving."));
        }
        else {
          move_motor = false;
          Serial.println(F("Stop moving."));
        }
        
        break;
      }
      case kToggleDirectionMessage: {
        if (motion_direction == mt::StepperDriver::MotionDirection::kPositive) {
          motion_direction = mt::StepperDriver::MotionDirection::kNegative;
          Serial.println(F("Negative direction."));
        }
        else {
          motion_direction = mt::StepperDriver::MotionDirection::kPositive;
          Serial.println(F("Positive direction."));
        }

        break;
      }
      case kIncreaseSpeedMessage: {
        kSpeed_RPM = kSpeed_RPM + kSpeedUpdater_RPM;
        stepper_driver.SetSpeed(kSpeed_RPM, mt::StepperDriver::SpeedUnits::kRevolutionsPerMinute);
        Serial.print(F("Speed (RPM) = "));
        Serial.println(kSpeed_RPM);
        break;
      }
      case kDecreaseSpeedMessage: {
        if (kSpeed_RPM >= 0.0) {
          kSpeed_RPM = kSpeed_RPM - kSpeedUpdater_RPM;
          stepper_driver.SetSpeed(kSpeed_RPM, mt::StepperDriver::SpeedUnits::kRevolutionsPerMinute);
        }
        Serial.print(F("Speed (RPM) = "));
        Serial.println(kSpeed_RPM);
        break;
      }
      default : {
        Serial.println(F("Invalid serial message."));
        break;
      }
    }
  }

  // Move the motor.
  if (move_motor == true) {
    stepper_driver.MoveByJogging(motion_direction); // This must be called periodically.
  }
}