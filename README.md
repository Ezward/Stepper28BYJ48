# Control the 28BYJ-48 stepper motor using the ULN2003 controller board.

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
using the ULN2003 controller board with minimal code.  Stepper motors work by enegizing coils in the motor to attract to internally placed magnets.  By energizing the coils in a specific pattern, the coils are attracted from magnet to magnet, thus turning the spindle.  The 28BYJ-48 stepper motor supports 3 stepping modes (3 coil energizing patterns):
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

