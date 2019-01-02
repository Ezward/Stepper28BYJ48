
/*
# Control the 28BYJ48 stepper motor using the ULN2003 controller board.

## Usage:
- construct one or more Stepper28BYJ48 instances, passing the mode and GPIO
  pins to the contructor for each instance.
- call the setup() method for each stepper instance in the application's
  setup() method.  This will initialize the GPIO pins.
- to move
  - call the stepper instance's startMovement() method and pass it
    the direction, distance in revolutions and speed in revolutions per second.
    Note that there is a upper bound to the speed; call getMaxSpeed() to
    determine the maximum speed.
  - while the stepper is moving, call isMoving().  isMoving() must be called
    often enough to prevent the motor from stuttering or stopping.  Generally
    once per iteration of the application's loop() method will suffice.
    However, if the loop method itself contains time consumeing for or while
    loops, then you should call isMoving() for each stepper instance inside
    the inner loop.
  - call the stepper instance's stopMovement() method to stop the current
    movement early.

## Notes:
This library is written specifically to drive the 28BYJ-48 stepper motor
using the ULN2003 controller board with minimal code.  It supports 3 stepping
modes:
- **Wave stepping mode** engages 1 of the 4 coils at each step, creating a 4 step
  sequence or approximately 2048 steps per revolution of the output spindle
  (see blow for details).  Because only one coil at a time in energized,
  this mode **draws less power**, but it also **generates less torque**.
  If you need to extend battery life and can live with lower torque,
  this is a useful mode.
- **Full stepping mode** engages 2 of the 4 coils at each step, creating a 4 step
  sequence or approximately 2048 steps in one revolution of the output spindle
  (see below for details).  Because 2 coils are always energized, this mode
  **provides the most torque and holding power**, but will use about twice as
  much power as wave stepping.
- **Half stepping mode** alternately engages 1 coil, then 2 coils in an 8 step
  sequence or approximately 4096 steps in one revolution.  It provides
  **double the precision** of wave or full stepping.
  However, the low bound for torque and holding power is like wave stepping
  because at times only one coil is engaged; the upper bound for power
  usage is like full stepping because at times two coils are engaged.

The Stepper28BYJ48 library uses revolutions as the unit to specify the
'distance' in a stepper movement.  It uses revolutions per minute (RPM)
to specify speed.  This makes the library independent of external values, like
wheel circumference or external gear ratios.

The stepper itself moves in discrete steps.  The number of steps per revolution
depends upon the chosen stepping mode.  Because of the internal gearing between
the motor spindle and the output spindle, there is **not an integral number of
steps per revolution** of the output spindle.

There are 8 sets of magnets internally; this combined with the number of
steps in the stepping cycle provides either 32 steps per revolution of the
motor spindle (for wave and full stepping) or 64 steps per revolution of
the motor spindle (for half stepping).

There are 4 sets of gears connecting the motor spindle to the output spindle.
The gear ratio between the motor spindle and the output spindle is the
product of these internal gear ratios and is given by:

      (32.0 / 9) * (22.0 / 11) * (26.0 / 9) * (31.0 / 10) ~ 63.6839506173/1

To get the number of steps per revolution of the output spindle, we mulitply
the steps per revolution of the motor spindle by the gear ratio between the
motor spindle and the output spindle:

    step per revolution of motor spindle * 63.6839506173

For wave step mode and full step mode, which each have 4 steps per cycle, and
so 4 * 8 = 32 steps per revolution of the motor spindle, the steps per
revolution of the output spindle is given by:

      32 * 63.684 ~= 2037.89 wave or full steps per revolution

For half step mode, which has 8 steps per stepping cycle and so
8 * 8 = 64 steps per revolution of the motor spindle, the steps per
revolution of the output spindle is given by:

      64 * 63.684 ~= 4075.77 half steps per revolution

*/

#include <arduino.h>
#include <math.h>
#include <debug.h>
#include "stepper28byj48.h"

//
// 28BYJ48 has internal motor with 32 full steps, 64 half steps per revolution
//         and has gearing between the motor and output spindle which reduces
//         the output spindle rotation based.
//
const unsigned int kMotorCoils = 4;     // number of coils (determine full steps in one cycle)
const unsigned int kFullStepCycles = 8; // number of full step cycles in one motor rotation base on 8 internal magnets
const unsigned int kFullSteps = kMotorCoils * kFullStepCycles; // number of full steps in one rotation
const double kGearReduction = (32.0 / 9) * (22.0 / 11) * (26.0 / 9) * (31.0 / 10); // internal gearing on spindle is ~ 63.6839506173:1
const double kMicrosPerMinute = 60000000.0;

//
// --------------------------------------- public api ----------------------------------------------
//


/**
 * constructor with default stepsPerRevolution
 */
Stepper28BYJ48::Stepper28BYJ48(
  const StepperMode mode,   // IN : kFullStep or kHalfStep
  const unsigned char in1,  // IN : GPIO pin connected to IN1 on ULN2003 controller board
  const unsigned char in2,  // IN : GPIO pin connected to IN2 on ULN2003 controller board
  const unsigned char in3,  // IN : GPIO pin connected to IN3 on ULN2003 controller board
  const unsigned char in4)  // IN : GPIO pin connected to IN4 on ULN2003 controller board
  : Stepper28BYJ48(mode, kGearReduction * ((kHalfStep == mode) ? (2 * kFullSteps) : kFullSteps), in1, in2, in3, in4)
{
//  this->stepsPerRevolution = kGearReduction * ((kHalfStep == mode) ? (2 * kFullSteps) : kFullSteps); // 4096 half steps,2048 full steps in each full spindle revolution
}

/**
 * constructor with explicit stepsPerRevolution
 */
Stepper28BYJ48::Stepper28BYJ48(
  const StepperMode mode,   // IN : kFullStep or kHalfStep
  const double stepsPerRevolution, // IN : number of steps in one revolution
  const unsigned char in1,  // IN : GPIO pin connected to IN1 on ULN2003 controller board
  const unsigned char in2,  // IN : GPIO pin connected to IN2 on ULN2003 controller board
  const unsigned char in3,  // IN : GPIO pin connected to IN3 on ULN2003 controller board
  const unsigned char in4)  // IN : GPIO pin connected to IN4 on ULN2003 controller board
  : mode(mode), in1(in1), in2(in2), in3(in3), in4(in4), stepsPerRevolution(stepsPerRevolution)
{

  //
  // setup constants based on mode
  //
  this->mode = mode;
  this->modeSteps = (kHalfStep == mode) ? (2 * kMotorCoils) : kMotorCoils;  // 8 half steps or 4 full steps in one cycle
  this->minMicrosPerStep = (kHalfStep == mode) ? 1000 : 2000; // full steps take twice as long as half steps, because the motor moves twice as far
  this->maxSpeed = kMicrosPerMinute / (long)floor(this->stepsPerRevolution * this->minMicrosPerStep);
}

/**
 * Setup the stepper motor and GPIO pins.
 * Call this in the application's setup() method.
 */
void Stepper28BYJ48::setup()
{
  //
  // init microcontroller gpio pins that are connected to ULN2003 stepper controller board
  //
  pinMode(this->in1, OUTPUT);
  pinMode(this->in2, OUTPUT);
  pinMode(this->in3, OUTPUT);
  pinMode(this->in4, OUTPUT);

  this->stopStep(); // initialize all pins to low state

  DPRINT("modeSteps: "); DPRINTLN((unsigned int)(this->modeSteps));
  DPRINT("minMicrosPerStep: "); DPRINTLN(this->minMicrosPerStep);
  DPRINT("stepsPerRevolution: "); DPRINTLN( this->stepsPerRevolution);
  DPRINT("maxSpeed : "); DPRINTLN(this->maxSpeed);
}

/**
 * Get the number of steps in a single revolution
 * of the output spindle.
 */
double Stepper28BYJ48::getStepsPerRevolution() // RET: number of steps in one revolution
                                                      //      of the output spindle
{
  return this->stepsPerRevolution;
}

/**
 * Start a new movement.
 *
 * NOTE: The actual RPM is limited by the stepper mode.
 *       Call getMaxSpeed() to determine the maximum possible speed.
 */
boolean Stepper28BYJ48::startMovement(  // RET: true if movement started, false if not
  const boolean clockwise,              // IN : kClockwise or kCounterClockwise
  const double revolutions,             // IN : number of revolutions to move
  const double speed)                   // IN : speed to move in revolutions per minute
{
  if(revolutions > 0)
  {
    this->clockwise = clockwise;
    this->stepCountDown = revolutionsToSteps(revolutions);

    //
    // calculate the duration of the movement in microseconds
    //
    const double targetStepsPerMinute = speed * this->stepsPerRevolution;
    const unsigned long targetMicrosPerStep = (unsigned long)floor(kMicrosPerMinute / targetStepsPerMinute);
    this->movementDurationMicros = (targetMicrosPerStep < this->minMicrosPerStep)
        ? this->stepCountDown * this->minMicrosPerStep  // limit to max speed
        : (unsigned long)floor(this->stepCountDown * kMicrosPerMinute / targetStepsPerMinute); // total movement duration in micros

    //
    // remember the start time of the movement
    //
    this->movementStartMicros = micros();

    DPRINT("direction : "); DPRINTLN(clockwise ? "clockwise" : "counterclockwise");
    DPRINT("stepCountDown: "); DPRINTLN(this->stepCountDown);
    DPRINT("movementDurationMicros: "); DPRINTLN(this->movementDurationMicros);

  }
  else
  {
    stopMovement();
  }

  return this->isMoving();
}

/**
 * Stop the current movement early.
 * It is not necessary to call this if movement finishes normally (when isMoving() is false).
 * After calling this, isMoving() will return false.
 */
void Stepper28BYJ48::stopMovement()
{
  this->stepCountDown = 0;
  this->movementDurationMicros = 0;
  this->delayUntilMicros = 0;
}



/**
 * Poll the movement.
 * This should be called at lease once each time through the loop() function.
 *
 * NOTE : The stepper may stutter or stop and fail to reach it's final
 *        position in the alloted time if isMoving() is not called often enough.
 *        So if there are long running processes within the loop() method
 *        (so there may be another for() or while() loop in the loop() method)
 *        then you should call this inside the inner loop.
 */
boolean Stepper28BYJ48::isMoving() // RET: true if still moving, false if done moving
{
    const unsigned long now = micros();
    if(now < this->delayUntilMicros)
    {
      // we are waiting for current step's delay to finish
      return true;
    }

    if(this->stepCountDown > 0)
    {
        //
        // calculate the duration of the step.
        //
        // this is total delay for this step.
        // this may be incurred during polling the move() method.
        //
        const unsigned long movementCurrentMicros = now - this->movementStartMicros;
        const unsigned long movementRemainingMicros = (movementCurrentMicros < this->movementDurationMicros)
            ? this->movementDurationMicros - movementCurrentMicros  // still some time left
            : 0;                                                    // movement duration is met or exceeded

        const unsigned long targetDelayMicros = movementRemainingMicros / this->stepCountDown;
        this->delayUntilMicros = now + max(targetDelayMicros, this->minMicrosPerStep);

        //
        // this is the minimum delay for this step.
        // we will incur this before we return from this function.
        //
        // NOTE: we can wait 1 millisecond in half-step mode,
        //       but we must wait 2 milliseconds in full step mode
        //       or motor will not move.
        //
        const unsigned long minDelayUntilMicros = now + this->minMicrosPerStep;
        switch(this->mode)
        {
            case kHalfStep:
            {
              writeHalfStep(this->step);
              break;
            }
            case kFullStep:
            {
              writeFullStep(this->step);
              break;
            }
            case kWaveStep:
            {
              writeWaveStep(this->step);
              break;
            }
            default:
              // no-op
              break;
        }
        incrementStep();

        //
        // wait until the step duration has passed
        //
        // while(micros() < minDelayUntilMicros)
        // {
        //   // just wait
        // }
        this->stepCountDown -= 1;
        this->stepperAccumulator += (kClockwise == this->clockwise) ? 1 : -1;

        // DPRINT("stepCountDown: "); DPRINTLN(this->stepCountDown);
        // DPRINT("step: "); DPRINTLN(this->step);
    }

    //
    // we are still moving is we have steps remaining or delay remaining
    //
    return (this->stepCountDown > 0) || (micros() < this->delayUntilMicros);
}

/**
 * Get the current movement direction.
 */
boolean Stepper28BYJ48::getDirection() // RET: kClockwise or kCounterClockwise
{
  return this->clockwise;
}

/**
 * Get the steps remaining in the current movement.
 * This is zero when ithe movement is done.
 */
unsigned long Stepper28BYJ48::getStepCountDown()  // RET: number of steps left in current movement
{
  return this->stepCountDown;
}

/**
 * Get the number of revolutions left in the current move.
 * This counts down as the stepper moves.
 * This will be zero when done moving.
 */
double Stepper28BYJ48::getRevolutions()  // RET: number of revolutions remaining in current movement
{
  return stepsToRevolutions(this->stepCountDown);
}

/**
 * Set the step acculumator.
 *
 * The accumulator increments for each clockwise step
 * and decrements for each counter-clockwise step.
 * The accumulator counts across movements.
 *
 * This method is most commonly used to zero the accumulator,
 * but it could be used 'pick up where you left off'.
 */
void Stepper28BYJ48::setStepAccumulator(
  const long steps)  // IN : new value of accumulator
{
  this->stepperAccumulator = steps;
}

/**
 * Get the accumulated steps.
 *
 * The accumulator increments for each clockwise step
 * and decrements for each counter-clockwise step.
 * The accumulator counts across movements.
 */
long Stepper28BYJ48::getStepAccumulator()  // RET: number of accumulated steps
{
  return this->stepperAccumulator;
}

/**
 * Get the maximum speed in revolutions per minute for current mode
 */
double Stepper28BYJ48::getMaxSpeed()  // RET: maximum possible speed in revolutions per minute
{
  return this->maxSpeed;
}


//
// ------------------------------- private helpers ------------------------------------
//


/**
 * Calculate the number of steps to turn the shaft
 * the given number of revolutions.
 *
 * @param revolutions number of revolutions; may be fractional
 * @return number of steps to do given number of revolutions
 */
unsigned long Stepper28BYJ48::revolutionsToSteps(const double revolutions)
{
  return floor(revolutions * this->stepsPerRevolution);
}

/**
 * calculate the number of revolutions
 * give a number of steps.
 */
double Stepper28BYJ48::stepsToRevolutions(unsigned long steps)
{
  return (double)steps / this->stepsPerRevolution;
}


//
// writes the step to the ULN2003
// using 8 step (half step) mode of BYJ48 stepper motor
//
void Stepper28BYJ48::writeHalfStep(const char step)
{
    //
    // moving the stepper involves electrifying one or two of the 4 coils
    // such that the magnetic field generated by the electrified coils attracts
    // one or two fixed magnets.  Each step engages an adjacent set of magnets,
    // causing the shart to move (turn).
    //
    switch (step) {
        case 0: {
            digitalWrite(this->in1, HIGH);
            digitalWrite(this->in2, LOW);
            digitalWrite(this->in3, LOW);
            digitalWrite(this->in4, LOW);
            break;
        }
        case 1: {
            digitalWrite(this->in1, HIGH);
            digitalWrite(this->in2, HIGH);
            digitalWrite(this->in3, LOW);
            digitalWrite(this->in4, LOW);
            break;
        }
        case 2: {
            digitalWrite(this->in1, LOW);
            digitalWrite(this->in2, HIGH);
            digitalWrite(this->in3, LOW);
            digitalWrite(this->in4, LOW);
            break;
        }
        case 3: {
            digitalWrite(this->in1, LOW);
            digitalWrite(this->in2, HIGH);
            digitalWrite(this->in3, HIGH);
            digitalWrite(this->in4, LOW);
            break;
        }
        case 4: {
            digitalWrite(this->in1, LOW);
            digitalWrite(this->in2, LOW);
            digitalWrite(this->in3, HIGH);
            digitalWrite(this->in4, LOW);
            break;
        }
        case 5: {
            digitalWrite(this->in1, LOW);
            digitalWrite(this->in2, LOW);
            digitalWrite(this->in3, HIGH);
            digitalWrite(this->in4, HIGH);
            break;
        }
        case 6: {
            digitalWrite(this->in1, LOW);
            digitalWrite(this->in2, LOW);
            digitalWrite(this->in3, LOW);
            digitalWrite(this->in4, HIGH);
            break;
        }
        case 7: {
            digitalWrite(this->in1, HIGH);
            digitalWrite(this->in2, LOW);
            digitalWrite(this->in3, LOW);
            digitalWrite(this->in4, HIGH);
            break;
        }
        default: {
            DPRINTLN("STOPPING");
            digitalWrite(this->in1, LOW);
            digitalWrite(this->in2, LOW);
            digitalWrite(this->in3, LOW);
            digitalWrite(this->in4, LOW);
            break;
        }
    }
}

//
// writes the step to the ULN2003
// using 4 step (full step) mode of BYJ48 stepper motor
//
void Stepper28BYJ48::writeFullStep(const char step)
{
    writeHalfStep(step * 2 + 1); // use every-other step (1, 3, 5, 7) of half-step mode, always engaging two magnets
}

//
// this is here for illustration; really there is little reason to use this mode
// other than to pull less power (this mode pull about 1/2 power of full step mode,
// but get's less torque and holding power because of it).
//
void Stepper28BYJ48::writeWaveStep(const char step)
{
    writeHalfStep(step * 2); // use every-other step (0, 2, 4, 6) of half-step mode, always engaging one magnet
}


//
// stop writing to the ULN2003 (deactivate all coils)
//
void Stepper28BYJ48::stopStep()
{
  writeHalfStep(-1); // falls into default clause
}

//
// increment the step based on the direction
// and handle wrap around based on step mode
//
void Stepper28BYJ48::incrementStep()
{
    if (this->clockwise)
    {
      this->step = (this->step < (this->modeSteps - 1)) ? this->step + 1 : 0;
    }
    else
    {
      this->step = (this->step > 0) ? (this->step - 1) : this->modeSteps - 1;
    }
}
