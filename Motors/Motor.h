/**
 * @file Motor.h
 *
 * @brief The Motor class header file.
 *
 * @section description_Motor_h Description
 * The Motor class header file.
 *
 * @section notes_Motor_h Notes
 * - Source file comments are Doxygen compatible.
 *
 * @section todo_Motor_h TODO
 * - None.
 *
 * @section author_Motor_h Author
 * - Created by John Woolsey on 08-01-2021.
 * - Modified by John Woolsey on 06-09-2022.
 *
 * Copyright (c) 2022 Woolsey Workshop.  All rights reserved.
 */


#ifndef Motor_H
#define Motor_H


// Includes
#include "Arduino.h"


// Types
/** The motor command enumeration type. */
enum MotorCommand {
   Forward,  ///< Drive forward.
   Reverse,  ///< Drive in reverse.
   Brake,    ///< Apply active braking.
   Coast     ///< Apply passive coasting.
};

/**
 * The Motor class.
 *
 * Provides a generic simple interface for controlling a DC motor.
 */
class Motor {
   char _name[21];         ///< The name of the motor (20 characters max).
   MotorCommand _command;  ///< The drive command of the motor.
   uint8_t _speed;         ///< The speed of the motor as a percentage of the maximum speed.
   uint8_t _posPin;        ///< The positive input control pin of the motor driver.
   uint8_t _negPin;        ///< The negative input control pin of the motor driver.

   void _driveMotor();  ///< Drives the motor with the current command and speed attributes.

 public:
   Motor(uint8_t posPin, uint8_t negPin, MotorCommand command = Coast, uint8_t speed = 0, const char* name = "Unknown");  ///< The motor instance constructor.
   const char* name();  ///< Retrieves the current name of the motor.
   void setName(const char* name);  ///< Sets the name of the motor.
   MotorCommand command();  ///< Retrieves the current drive command of the motor.
   void setCommand(MotorCommand command);  ///< Sets the drive command of the motor.
   uint8_t speed();  ///< Retrieves the current speed of the motor.
   void setSpeed(uint8_t speed);  ///< Sets the speed of the motor.
   void drive(MotorCommand command = Forward, uint8_t speed = 100);  ///< Drives the motor with the specified attributes.
   void stop(MotorCommand command = Coast);  ///< Stops the motor with the specified command.
};

#endif  // Motor_H
