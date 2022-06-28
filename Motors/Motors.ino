/**
 * @mainpage Arduino Motors Project
 *
 * @section description_main Description
 * An Arduino sketch that interfaces with an Adafruit DRV8833 DC/Stepper Motor
 * Driver Breakout Board to drive two small DC motors.
 *
 * @section circuit_main Circuit
 * - An Adafruit DRV8833 DC/Stepper Motor Driver Breakout Board is connected
 *   as shown below.
 *   - SLP pin is connected to pin D2.
 *   - FLT pin is connected to pin D3.
 *   - AIN1 pin is connected to pin D5.
 *   - AIN2 pin is connected to pin D6.
 *   - BIN1 pin is connected to pin D9.
 *   - BIN2 pin is connected to pin D10.
 *   - AOUT1 pin is connected to motor A positive pin.
 *   - AOUT2 pin is connected to motor A negative pin.
 *   - BOUT1 pin is connected to motor B positive pin.
 *   - BOUT2 pin is connected to motor B negative pin.
 *   - VMOTOR+ pin is connected to a 6-9 volt power supply.
 * - A 10K potentiometer is connected to pin A0.
 *
 * @section notes_main Notes
 * - Use the Serial Monitor to view output while in DEBUG mode.
 *
 * Copyright (c) 2022 Woolsey Workshop.  All rights reserved.
 */

/**
 * @file Motors.ino
 *
 * @brief The primary Arduino sketch implementation file.
 *
 * @section description_motors_ino Description
 * The primary Arduino sketch implementation file.
 *
 * @section libraries_motors_ino Libraries
 * - Motor Class (local)
 *   - Provides a generic simple interface for controlling a DC motor.
 *
 * @section notes_motors_ino Notes
 * - Comments are Doxygen compatible.
 *
 * @section todo_motors_ino TODO
 * - None.
 *
 * @section author_motors_ino Author(s)
 * - Created by John Woolsey on 08-01-2021.
 * - Modified by John Woolsey on 06-28-2022.
 *
 * Copyright (c) 2022 Woolsey Workshop.  All rights reserved.
 */


// Includes
#include "Motor.h"


// Defines
#define DEBUG 1           ///< The mode of operation; 0 = normal, 1 = debug.
#define OP_DURATION 5000  ///< The operation duration in milliseconds.


// Pin Mapping
const uint8_t DRV8833_SLP = 2;    ///< The pin connected to the SLP (sleep) pin of the DRV8833 motor driver board.
const uint8_t DRV8833_FLT = 3;    ///< The pin connected to the FLT (fault) pin of the DRV8833 motor driver board.
const uint8_t DRV8833_AIN1 = 5;   ///< The pin connected to the AIN1 (motor A control 1) pin of the DRV8833 motor driver board.
const uint8_t DRV8833_AIN2 = 6;   ///< The pin connected to the AIN2 (motor A control 2) pin of the DRV8833 motor driver board.
const uint8_t DRV8833_BIN1 = 9;   ///< The pin connected to the BIN1 (motor B control 1) pin of the DRV8833 motor driver board.
const uint8_t DRV8833_BIN2 = 10;  ///< The pin connected to the BIN2 (motor B control 2) pin of the DRV8833 motor driver board.
const uint8_t Pot = A0;           ///< The pin connected to the 10 KΩ potentiometer.


// Global Variables
volatile bool motorDriverFaultDetected = false;  ///< The fault detected status of the motor driver.  The 'volatile' specifier is used since its value can be changed from within an interrupt service routine.
volatile bool motorDriverFaultCleared = false;   ///< The fault cleared status of the motor driver.  The 'volatile' specifier is used since its value can be changed from within an interrupt service routine.


// Global Instances
Motor motorA = Motor(DRV8833_AIN1, DRV8833_AIN2);                 ///< The instance of motor A.  Uses default arguments of command = Coast, speed = 0, name = "Unknown"
Motor motorB = Motor(DRV8833_BIN1, DRV8833_BIN2, Coast, 0, "B");  ///< The instance of motor B.  All arguments are specified.


/**
 * Standard Arduino setup function used for setup and configuration tasks.
 */
void setup() {
   // Serial Monitor
   if (DEBUG) {
      Serial.begin(9600);  // initialize serial bus
      while (!Serial);     // wait for serial connection
      Serial.println(F("Running in DEBUG mode.  Turn off for normal operation."));
   }

   // Pin configurations
   pinMode(DRV8833_SLP, OUTPUT);
   pinMode(DRV8833_FLT, INPUT_PULLUP);  // utilize microprocessor's internal pull-up resistor

   // Initialize interrupt service routine
   // Calls motorDriverFaultPinChanged() if change detected on DRV8833_FLT pin
   attachInterrupt(digitalPinToInterrupt(DRV8833_FLT), motorDriverFaultPinChanged, CHANGE);

   // Enable (turn on) motor driver
   digitalWrite(DRV8833_SLP, HIGH);
}


/**
 * Standard Arduino loop function used for repeating tasks.
 */
void loop() {
   checkForMotorDriverFault();
   basicOperations();  // perform basic motor control operations on motor A
   // rampingSpeed();  // ramp up and down the speed of motor A
   // potentiometerControl();  // control motor B speed with a potentiometer
}


/**
 * Demonstrates the basic operations available with the Motor class.
 */
void basicOperations() {
   // Basic operations with default attributes
   motorA.drive();                       // drive forward at full throttle
   if (DEBUG) printMotorStatus(motorA);
   delay(OP_DURATION);
   motorA.stop();                        // coast (soft) to a stop
   if (DEBUG) printMotorStatus(motorA);
   delay(OP_DURATION);

   // Basic operations with specific attributes
   motorA.drive(Forward, 75);            // drive forward at 75% throttle
   if (DEBUG) printMotorStatus(motorA);
   delay(OP_DURATION);
   motorA.stop(Coast);                   // coast (soft) to a stop
   if (DEBUG) printMotorStatus(motorA);
   delay(OP_DURATION);
   motorA.drive(Reverse, 50);            // drive in reverse at 50% throttle
   if (DEBUG) printMotorStatus(motorA);
   delay(OP_DURATION);
   motorA.stop(Brake);                   // brake (hard) to a stop
   if (DEBUG) printMotorStatus(motorA);
   delay(OP_DURATION);

   // Setting attributes
   motorA.setName("A");                  // change name of motor to A
   if (DEBUG) printMotorStatus(motorA);
   motorA.setCommand(Forward);           // drive forward at previously set speed
   if (DEBUG) printMotorStatus(motorA);
   delay(OP_DURATION);
   motorA.setSpeed(75);                  // set speed to 75% throttle with previously set command
   if (DEBUG) printMotorStatus(motorA);
   delay(OP_DURATION);
   motorA.setSpeed(0);                   // set speed to 0% throttle with previously set command
   if (DEBUG) printMotorStatus(motorA);
   delay(OP_DURATION);
   motorA.setCommand(Reverse);           // drive in reverse with previously set speed
   if (DEBUG) printMotorStatus(motorA);
   delay(OP_DURATION);
   motorA.setSpeed(75);                  // set speed to 75% throttle with previously set command
   if (DEBUG) printMotorStatus(motorA);
   delay(OP_DURATION);
   motorA.setCommand(Coast);             // coast (soft) to a stop
   if (DEBUG) printMotorStatus(motorA);
   delay(OP_DURATION);

   // Getting attributes
   // The command(), name(), and speed() methods retrieve the motor instance's
   // current command, name, and speed values respectively and are utilized
   // within the printMotorStatus() function used above.
}


/**
 * Checks and reports the fault status of the motor driver.
 */
void checkForMotorDriverFault() {
   if (motorDriverFaultDetected) {
      if (DEBUG) Serial.println(F("Motor driver fault detected."));
      motorDriverFaultDetected = false;  // reset detected flag
   }
   if (motorDriverFaultCleared) {
      if (DEBUG) Serial.println(F("Motor driver fault cleared."));
      motorDriverFaultCleared = false;  // reset cleared flag
   }
}


/**
 * Sets the motorDriverFaultDetected and motorDriverFaultCleared flags when a
 * change in the motor driver's FLT pin is detected.
 *
 * Invoked as an interrupt service routine (ISR), if enabled.
 */
void motorDriverFaultPinChanged() {
   if (digitalRead(DRV8833_FLT) == LOW) motorDriverFaultDetected = true;
   else motorDriverFaultCleared = true;
}


/**
 * Sets the speed of motor B relative to the value retrieved from the
 * potentiometer.
 */
void potentiometerControl() {
   static int previousReading = 0;
   int currentReading = analogRead(Pot);
   if (abs(currentReading - previousReading) > 10) {  // minimize unnecessary updates
      if (DEBUG) {
         Serial.print(F("Potentiometer reading: "));
         Serial.println(currentReading);
      }
      motorB.drive(Forward, map(currentReading, 0, 1023, 0, 100));  // maps ADC reading range to percentage range before driving
      if (DEBUG) printMotorStatus(motorB);
      previousReading = currentReading;
   }
}


/**
 * Prints the status of the motor to the Serial Monitor.
 *
 * @param motor  The motor instance.
 */
void printMotorStatus(Motor motor) {
   const char* const command_states[] = {"Forward", "Reverse", "Brake", "Coast"};  // constant array of constant strings
   Serial.print(F("Motor "));
   Serial.print(motor.name());
   Serial.print(F(": Command = "));
   Serial.print(command_states[motor.command()]);
   Serial.print(F(", Speed = "));
   Serial.println(motor.speed());
}


/**
 * Ramps up and down the speed of motor A.
 */
void rampingSpeed() {
   rampUp(motorA, Forward, OP_DURATION);    // ramp up forward speed for OP_DURATION ms
   rampDown(motorA, Forward, OP_DURATION);  // ramp down forward speed for OP_DURATION ms
}


/**
 * Ramps down the speed of a motor from 100% to 0% throttle over a specified
 * time period.
 *
 * @param motor     The motor instance.
 * @param command   The motor's drive command (Forward or Reverse).
 * @param duration  The length of time, in milliseconds, to ramp down the speed
 *                  of the motor.
 */
void rampDown(Motor motor, MotorCommand command, unsigned long duration) {
   if (command == Forward || command == Reverse) {
      motor.setCommand(command);
      for (int8_t speed = 100; speed >= 0; speed--) {  // signed integer used to avoid rollover issues
         motor.setSpeed(speed);
         delay(duration/100);
      }
   }
}


/**
 * Ramps up the speed of a motor from 0% to 100% throttle over a specified
 * time period.
 *
 * @param motor     The motor instance.
 * @param command   The motor's drive command (Forward or Reverse).
 * @param duration  The length of time, in milliseconds, to ramp up the speed of
 *                  the motor.
 */
void rampUp(Motor motor, MotorCommand command, unsigned long duration) {
   if (command == Forward || command == Reverse) {
      motor.setCommand(command);
      for (uint8_t speed = 0; speed <= 100; speed++) {
         motor.setSpeed(speed);
         delay(duration/100);
      }
   }
}
