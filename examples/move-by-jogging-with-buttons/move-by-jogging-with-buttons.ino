// Copyright (C) 2024 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file move-by-jogging-with-buttons.ino
/// @brief Example showing how to move a stepper motor by pressing/releasing a button to start/stop the motor, using the MT-arduino-stepper-driver library.
/// The MT-arduino-momentary-button library is also required in order to control the buttons.

#include <stepper_driver.h>
#include <momentary_button.h>

// GPIO pins.
constexpr uint8_t kDirectionButtonPin = 2; ///< Input pin for the direction button.
constexpr uint8_t kMoveButtonPin = 3; ///< Input pin for the move button.
constexpr uint8_t kPulPin = 11; ///< Output pin for the stepper driver PUL/STP/CLK (pulse/step) interface.
constexpr uint8_t kDirPin = 12; ///< Output pin for the stepper driver DIR/CW (direction) interface.
constexpr uint8_t kEnaPin = 13; ///< Output pin for the stepper driver ENA/EN (enable) interface.

// Serial properties.
constexpr int kBaudRate = 9600; ///< The serial communication speed.

// Button properties.
constexpr auto kDirectionButtonUnpressedPinState = mt::MomentaryButton::PinState::kLow; ///< Direction button unpressed pin state.
constexpr auto kMoveButtonUnpressedPinState = mt::MomentaryButton::PinState::kLow; ///< Move button unpressed pin state.
constexpr uint16_t kDirectionButtonDebouncePeriod_ms = 20; ///< Direction button debounce period (ms).
constexpr uint16_t kMoveButtonDebouncePeriod_ms = 20; ///< Move button debounce period (ms).
constexpr uint16_t kDirectionButtonShortPressPeriod_ms = 500; ///< Direction button short press period (ms).

// Stepper motor/drive system properties.
constexpr float kFullStepAngle_degrees = 1.8F; ///< The stepper motor full step angle in degrees. Obtained from the motor data sheet.
constexpr float kGearRatio = 1.0F; ///< The system/stepper motor gear ratio (1 if not using a gearing system or a geared stepper motor).

// Stepper driver properties.
constexpr uint16_t kMicrostepMode = 8; ///< Microstep mode (1/8). Remember to change the setting on the stepper driver to match.
// Minimum time (us) to delay after changing the state of a pin. Obtained from the stepper driver data sheet.
// These values are for the StepperOnline DM542T stepper driver, but should work for most stepper drivers.
constexpr float kPulDelay_us = 2.5F; ///< Minimum delay (us) for the stepper driver PUL pin.
constexpr float kDirDelay_us = 5.0F; ///< Minimum delay (us) for the stepper driver Dir pin.
constexpr float kEnaDelay_us = 5.0F; ///< Minimum delay (us) for the stepper driver Ena pin.
// Speed.
constexpr float kSpeed_RPM = 20.0F; ///< Rotation speed (RPM).

// Other properties.
constexpr uint16_t kStartupTime_ms = 1000; ///< Minimum startup/boot time in milliseconds (ms); based on the stepper driver.

/// @brief The Momentary Button instance for the direction button.
mt::MomentaryButton direction_button{kDirectionButtonPin, kDirectionButtonUnpressedPinState, kDirectionButtonDebouncePeriod_ms, kDirectionButtonShortPressPeriod_ms};
/// @brief The Momentary Button instance for the move button.
mt::MomentaryButton move_button{kMoveButtonPin, kMoveButtonUnpressedPinState, kMoveButtonDebouncePeriod_ms};

/// @brief Stepper Driver instance for the stepper motor.
mt::StepperDriver stepper_driver{kPulPin, kDirPin, kEnaPin, kMicrostepMode, kFullStepAngle_degrees, kGearRatio};
//mt::StepperDriver stepper_driver{kPulPin, kDirPin, kEnaPin}; // Default values: microstep mode = 1, full step angle = 1.8, gear ratio = 1.  

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

  // Set the expected behaviour of the stepper driver pins.
  // If these are not set, default values from the library will be used.
  stepper_driver.set_ena_pin_enabled_state(mt::StepperDriver::PinState::kLow); // The pin state for giving power to the motor.
  stepper_driver.set_dir_pin_positive_direction_state(mt::StepperDriver::PinState::kHigh); // The pin state for a "positive" motion direction.

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
  // Flag to keep track of when the move button is pressed.
  static bool move_motor = false;
  // Variable to keep track of the motion direction when the direction button is pressed.
  static auto motion_direction = mt::StepperDriver::MotionDirection::kPositive;

  // Detect state change on the move button pin.
  mt::MomentaryButton::ButtonState move_button_state = move_button.DetectStateChange(); // This must be called repeatedly.

  if (move_button_state == mt::MomentaryButton::ButtonState::kPressed) {
    move_motor = true;
    Serial.println(F("Start moving."));
  }

  if (move_button_state == mt::MomentaryButton::ButtonState::kReleased) {
    move_motor = false;
    Serial.println(F("Stop moving."));
  }

  // Detect press type on the direction button pin.
  mt::MomentaryButton::PressType direction_button_press_type = direction_button.DetectPressType(); // This must be called repeatedly.

  if (direction_button_press_type == mt::MomentaryButton::PressType::kShortPress) {
    if (motion_direction == mt::StepperDriver::MotionDirection::kPositive) {
      motion_direction = mt::StepperDriver::MotionDirection::kNegative;
      Serial.println(F("Negative direction."));
    }
    else {
      motion_direction = mt::StepperDriver::MotionDirection::kPositive;
      Serial.println(F("Positive direction."));
    }
  }

  // Move the motor.
  if (move_motor == true) {
    stepper_driver.MoveByJogging(motion_direction); // This must be called repeatedly.
  }
}