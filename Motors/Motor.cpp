/**
 * @file Motor.cpp
 *
 * @brief The Motor class implementation file.
 *
 * @section description_Motor_cpp Description
 * The Motor class implementation file.
 *
 * @section todo_Motor_cpp TODO
 * - None.
 *
 * @section author_Motor_cpp Author
 * - Created by John Woolsey on 08-01-2021.
 * - Modified by John Woolsey on 06-09-2022.
 *
 * Copyright (c) 2022 Woolsey Workshop.  All rights reserved.
 */


// Includes
#include "Motor.h"


/**
 * The Motor instance constructor.
 *
 * Creates and configures the motor instance with the specified attributes and
 * drives the motor accordingly.
 *
 * @param posPin   The positive input control pin of the motor driver.
 * @param negPin   The negative input control pin of the motor driver.
 * @param command  The drive command of the motor (Forward, Reverse, Brake, or Coast).  The default is Coast.
 * @param speed    The speed of the motor as a percentage of the maximum speed.  The default is 0.
 * @param name     The name of the motor (20 characters max).  The default is Unknown.
 *
 * @return  Returns a configured instance of a motor.
 */
Motor::Motor(uint8_t posPin, uint8_t negPin, MotorCommand command, uint8_t speed, const char* name) {
   _posPin = posPin;
   _negPin = negPin;
   _command = command;
   _speed = speed;
   strcpy(_name, name);
   pinMode(posPin, OUTPUT);
   pinMode(negPin, OUTPUT);
   _driveMotor();
}


/**
 * Retrieves the current name of the motor.
 *
 * @return  Returns the motor's current name.
 */
const char* Motor::name() {
   return _name;
}


/**
 * Sets the name of the motor.
 *
 * @param name  The motor's name (20 characters max).
 */
void Motor::setName(const char* name) {
   strcpy(_name, name);
}


/**
 * Retrieves the current drive command of the motor.
 *
 * @return  Returns the motor's current drive command (Forward, Reverse, Brake, or Coast).
 */
MotorCommand Motor::command() {
   return _command;
}


/**
 * Sets and applies the drive command of the motor.
 *
 * @param command  The motor's drive command (Forward, Reverse, Brake, or Coast).
 */
void Motor::setCommand(MotorCommand command) {
   _command = command;
   _driveMotor();
}


/**
 * Retrieves the current speed of the motor.
 *
 * @return  Returns the motor's current speed as a percentage of the maximum speed.
 */
uint8_t Motor::speed() {
   return _speed;
}


/**
 * Sets and applies the speed of the motor.
 *
 * @param speed  The motor's speed as a percentage of the maximum speed.
 */
void Motor::setSpeed(uint8_t speed) {
   _speed = speed;
   _driveMotor();
}


/**
 * Sets and drives the motor with the specified attributes.
 *
 * @param command  The motor's drive command (Forward, Reverse, Brake, or Coast).  The default is Forward.
 * @param speed    The motor's speed as a percentage of the maximum speed.  The default is 100.
 */
void Motor::drive(MotorCommand command, uint8_t speed) {
   _command = command;
   _speed = speed;
   _driveMotor();
}


/**
 * Stops the motor with the specified drive command.
 *
 * If the Brake command is specified, the motor actively brakes to a stop.
 * If any other command is specified, the motor passively coasts to a stop.
 *
 * @param command  The motor's drive command (Brake or Coast).  The default is Coast.
 */
void Motor::stop(MotorCommand command) {
   _command = (command == Brake) ? Brake : Coast;
   _driveMotor();
}


/**
 * Drives the motor with the current command and speed attributes.
 *
 * Used internally to actively drive the input pins of the motor driver
 * according to the motor's currently set attributes.  This method is called by
 * many of the class' public methods.
 *
 * Ensures the current speed is set to an appropriate value within a valid
 * range.
 */
void Motor::_driveMotor() {
   if (_command == Brake || _command == Coast) _speed = 0;  // set speed to 0 if motor is stopping
   _speed = constrain(_speed, 0, 100);  // constrain speed to valid percentage range
   switch (_command) {
      case Forward:
         digitalWrite(_negPin, LOW);
         analogWrite(_posPin, map(_speed, 0, 100, 0, 255));  // use PWM to adjust speed
         break;
      case Reverse:
         digitalWrite(_posPin, LOW);
         analogWrite(_negPin, map(_speed, 0, 100, 0, 255));  // use PWM to adjust speed
         break;
      case Brake:
         digitalWrite(_posPin, HIGH);
         digitalWrite(_negPin, HIGH);
         break;
      default:  // Coast
         digitalWrite(_posPin, LOW);
         digitalWrite(_negPin, LOW);
         break;
   }
}
