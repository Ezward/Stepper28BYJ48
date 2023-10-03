/*
   28BYJ-48 Stepper motor and ULN2003 stepper controller to
   drive a two wheel differential drive robot.
               
   Plug 28BYJ-48 into socket of ULN2003 board
   ULN2003 IN1 (blue wire) to microcontroller GPIO configured for output
   ULN2003 IN2 (pink wire) to microcontroller GPIO configured for output 
   ULN2003 IN3 (yellow wire) to microcontroller GPIO configured for output
   ULN2003 IN4 (orange wire) to microcontroller GPIO configured for output 
   ULN2003 (+) to microcontroller 5V Prefer to use external 5V Source 
   ULN2003 (-) to microcontroller GND (shared with external 5V Source if used)

   NOTES:
   - some 28BYJ-48 are rated for 12V, check your stepper
   - sketch uses 2 steppers, so 8 GPIO pins are required.  A breadboard can be used
     to create a 5v AND GND busses to power and ground can be shared.

   WHY:
   Why use a cheap stepper motor as the basis for a robot?  
   Well, they are cheap (as little as ~2 USD with controller board), 
   they are easy to implement and they are very precise.  
   For some applications, precision is a great benefit.  These cheap steppers
   have about 2048 steps per revolution in full step mode.  Also, they are
   pretty quiet compared to a cheap geared DC motor.  The disadvantage
   is that they are slow (very slow); only about on revolution every 4 seconds.  

******************************/
#ifndef DEBUG
  #define DEBUG
#endif

#include <math.h>
#include <debug.h>
#include <stepper28byj48.h>

//
// associate pins on ULN2003 to GPIO pins on Arduino.
// if you wire differently, change code here.
//
const int IN1_RIGHT = 4;
const int IN2_RIGHT = 5;
const int IN3_RIGHT = 6;
const int IN4_RIGHT = 7;
const int IN1_LEFT = 8;
const int IN2_LEFT = 9;
const int IN3_LEFT = 10;
const int IN4_LEFT = 11;

const double kWheelDiameter = 6.3;                       // diameter of wheel in cm
const double kWheelCircumference = PI * kWheelDiameter;  // distance cm of one revolution of output spindle
const double kWheelSpan = 16.75;                         // distance between wheels in cm
const double kWheelSpanCircumference = PI * kWheelSpan;  // full turn distance in cm

const double kTurnAroundRevolutions = (kWheelSpanCircumference / 2) / kWheelCircumference;
const double kRevolutionsPerMeter = 100 / kWheelCircumference;

//
// possible movements
//
typedef enum MovementState
{
  kInitial,
  kForward,
  kTurnRight,
  kTurnLeft,
  kTurnAround,
  kMovementError
};

//
// 2 stepper instances using stepper_28byj48 library
//
Stepper28BYJ48 rightWheel(kFullStep, IN1_RIGHT, IN2_RIGHT, IN3_RIGHT, IN4_RIGHT);
Stepper28BYJ48 leftWheel(kFullStep, IN1_LEFT, IN2_LEFT, IN3_LEFT, IN4_LEFT);

MovementState movementState = kInitial;
double forwardSpeed;
double turnSpeed;
boolean error = false;

void setup() 
{
  DSETUP();
  DPRINTLN("setup");

  rightWheel.setup();
  leftWheel.setup();

  forwardSpeed = min(rightWheel.getMaxSpeed(), leftWheel.getMaxSpeed());
  turnSpeed = forwardSpeed / 2;
}

void loop() 
{
  //
  // drive forward and backward
  //
  const int moving = 0 != (leftWheel.isMoving() + rightWheel.isMoving());
  if(!moving)
  {
    switch(movementState)
    {
      case kInitial:
      {
        DPRINTLN("Starting.");
        movementState = moveForward(kRevolutionsPerMeter);
        break;
      }
      case kForward:
      {
        DPRINTLN("done moving forward.");
        movementState = turnAround();
        break;
      }
      case kTurnAround:
      {
        DPRINTLN("done turning around.");
        movementState = moveForward(kRevolutionsPerMeter);
        break;
      }
      case kMovementError:
      {
        if(!error)
        {
          DPRINTLN("Movement error, movement halted");
          error = true;
        }
        break;  // stopped
      }
    }
  }
}

MovementState moveForward(const double revolutions)
{
  leftWheel.startMovement(kCounterClockwise, revolutions, forwardSpeed);
  rightWheel.startMovement(kClockwise, revolutions, forwardSpeed);
  DPRINT("moving foward...");
  return (leftWheel.isMoving() + rightWheel.isMoving()) ? kForward : kMovementError;
}

MovementState turnAround()
{ 
  leftWheel.startMovement(kClockwise, kTurnAroundRevolutions, turnSpeed);
  rightWheel.startMovement(kClockwise, kTurnAroundRevolutions, turnSpeed);  
  DPRINT("turning right around...");
  return (leftWheel.isMoving() + rightWheel.isMoving()) ? kTurnAround : kMovementError;
}

MovementState turnRight()
{
  leftWheel.startMovement(kClockwise, kTurnAroundRevolutions / 2, turnSpeed);
  rightWheel.startMovement(kClockwise, kTurnAroundRevolutions / 2, turnSpeed);  
  DPRINT("turning right...");
  return (leftWheel.isMoving() + rightWheel.isMoving()) ? kTurnRight : kMovementError;

}

MovementState turnLeft()
{
  leftWheel.startMovement(kClockwise, kTurnAroundRevolutions / 2, turnSpeed);
  rightWheel.startMovement(kClockwise, kTurnAroundRevolutions / 2, turnSpeed);  
  DPRINT("turning left...");
  return (leftWheel.isMoving() + rightWheel.isMoving()) ? kTurnLeft : kMovementError;
}


