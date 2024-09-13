# MT-arduino-stepper-driver

[![Static check](https://github.com/Morgritech/MT-arduino-stepper-driver/actions/workflows/static-check.yaml/badge.svg)](https://github.com/Morgritech/MT-arduino-stepper-driver/actions/workflows/static-check.yaml) [![Build examples](https://github.com/Morgritech/MT-arduino-stepper-driver/actions/workflows/build-examples.yaml/badge.svg)](https://github.com/Morgritech/MT-arduino-stepper-driver/actions/workflows/build-examples.yaml)

Stepper motor driver library for the Arduino platform, to control stepper motors via stepper motor drivers that have a "step-direction-enable" interface.

This library implements non-blocking functions to move a stepper motor by jogging (start/stop on command), or by a set angle (absolute or relative). Constant speed and acceleration/deceleration are implemented. The library can handle multiple stepper drivers/motors, including geared stepper motors.

The following features are available:

- Constant speed motion. Speed can be set in various units, including; microsteps per second, degrees per second, radians per second, and, revolutions per minute.
- Move by jogging (start/stop on command) in a clockwise or counter-clockwise direction.
- Move by angle. Angles can be provided in various units, including; microsteps, degrees, radians, and, revolutions. Motion can be absolute (with respect to the start position) or relative (with respect to the current position). Also, motion can be paused/resumed.
- Built-in functionality to account for geared stepper motors or drive systems with a gearbox.
- The amount of microsteps to perform a particular motion can be retrieved, which can be used externally to develop synchronised motion between multiple motors.
- The angular position can be retrieved, which can be used externally to keep track of the deviation from a start/"soft home"/zero position.
- Acceleration. Acceleration can be set in various units, including; microsteps per second squared, degrees per second squared, radians per second squared, and, revolutions per minute squared.
- User selectable acceleration algorithms. Choose from 3 different algorithms; Austin D. 2005, Eiderman A. 2004 and an in-house algorithm developed by Morgridge J. 2024 during the creation of this library.
- The library allows changes in speed and acceleration during motion, however, this may negatively impact the motion especially on relatively slow microcontrollers.

A [report](extras/dsdr1001%20Stepper%20Motor%20Control%20Equations%20Issue%2001%2012-09-2024.pdf) showing the equations and algorithms used in the library can be found in the "extras" folder.

Overall, the library is relatively easy to use, and requires very little setup especially if mostly using the built-in defaults. See the "examples" folder for how to get started on using the library.

This library can be installed via the Arduino Library Manager.
