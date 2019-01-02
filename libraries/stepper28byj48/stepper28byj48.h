#ifndef STEPPER28BYJ48_H
#define STEPPER28BYJ48_H


//
// BYJ48 supports 8 half steps (smoother) or
// 4 full steps (higher torque)
//
typedef enum StepperMode
{
  kWaveStep = 1,  // 4 steps, lower power drain, lower torque
  kFullStep = 2,  // 4 steps, higher power drain, higher torque
  kHalfStep = 3   // 8 steps, higher power drain, lower torque, higher precision
};

//
// direction of rotation
//
const boolean kClockwise = true;
const boolean kCounterClockwise = !kClockwise;

class Stepper28BYJ48
{
  //
  // --------------------- internal state ---------------------
  //
  private:
    //
    // GPIO pins that correspond to IN1..4 of ULN2003 controller board
    //
    unsigned char in1, in2, in3, in4;

    //
    // mode determines how many steps per revolution
    //
    StepperMode mode;                 // kFullStep or kHalfStep
    char modeSteps;                   // number of steps in given mode
    double stepsPerRevolution;        // number of steps in one revolution of spindle
    unsigned long minMicrosPerStep;   // minimum number of microseconds per step
    double maxSpeed;                  // maximum possible speed in revolutions per minute

    //
    // direction, speed and revolutions are
    // by the user to make the stepper motor move.
    // These are converted into steps and waits
    // in order to move the stepper.
    //
    boolean clockwise = kClockwise; // true is clockwise, false is counter clockwise

    //
    // The library uses steps to actually drive the motor
    //
    int step = 0;                             // 0..3 in 4 step mode, 0..7 in 8 step mode
    unsigned int stepCountDown = 0;           // number of steps remaining in current movement
    long stepperAccumulator = 0;              // total number of accululated steps (adds clockwise, subtracts counterclockwise) since last stop.
    unsigned long movementDurationMicros = 0; // number of microseconds in current movement
    unsigned long movementStartMicros = 0;    // time at which movement start started (call to start())
    unsigned long delayUntilMicros = 0;       // number of microseconds to delay until next step can start

  //
  // --------------------- public api -----------------------------
  //
  public:
    /**
     * constructor with explicit stepsPerRevolution
     */
    Stepper28BYJ48(
      const StepperMode mode,   // IN : kFullStep or kHalfStep
      const double stepsPerRevolution, // IN : number of steps in one revolution
      const unsigned char in1,  // IN : GPIO pin connected to IN1 on ULN2003 controller board
      const unsigned char in2,  // IN : GPIO pin connected to IN2 on ULN2003 controller board
      const unsigned char in3,  // IN : GPIO pin connected to IN3 on ULN2003 controller board
      const unsigned char in4);  // IN : GPIO pin connected to IN4 on ULN2003 controller board

    /**
     * constructor with default stepsPerRevolution
     */
    Stepper28BYJ48(
      const StepperMode mode,   // IN : kWaveStep, kFullStep or kHalfStep
      const unsigned char in1,  // IN : GPIO pin connected to IN1 on ULN2003 controller board
      const unsigned char in2,  // IN : GPIO pin connected to IN2 on ULN2003 controller board
      const unsigned char in3,  // IN : GPIO pin connected to IN3 on ULN2003 controller board
      const unsigned char in4); // IN : GPIO pin connected to IN4 on ULN2003 controller board

    /**
     * Get the number of steps in a single revolution
     * of the output spindle.
     */
    double getStepsPerRevolution(); // RET: number of steps in one revolution
                                    //      of the output spindle

    /**
     * Setup the stepper motor and GPIO pins.
     * Call this in the application's setup() method.
     */
    void setup();

    /**
     * Start a new movement.
     *
     * NOTE: The actual RPM is limited by the stepper mode.
     *       Call getMaxSpeed() to determine the maximum possible speed
     */
    boolean startMovement(        // RET: true if movement started, false if not
      const boolean clockwise,    // IN : kClockwise or kCounterClockwise
      const double revolutions,   // IN : number of revolutions to move
      const double speed);        // IN : speed to move in revolutions per minute

    /**
     * Stop the current movement.
     * After calling this, isMoving() will return false.
     */
    void stopMovement();

    /**
     * Poll the movement.
     * This should be called at least once each time through the loop() function.
     *
     * NOTE : the stepper may stutter or stop and fail to reach it's final
     *        position in the alloted time if isMoving() is not called often enough.
     *        So if there are long running processes within the loop() method
     *        (so there may be another for() or while() loop in the loop() method)
     *        then you should call this inside the inner loop.
     */
    boolean isMoving();  // RET: true if still moving, false if done moving

    /**
     * Get the current movement direction.
     */
    boolean getDirection(); // RET: kClockwise or kCounterClockwise

    /**
     * Get the remaining steps in the current movement.
     * This is zero when the movement is done.
     */
    unsigned long getStepCountDown();  // RET: number of steps remaining in the current movement

   /**
     * Get the number of revolutions left in the current move.
     * This counts down as the stepper moves.
     * This will be zero when done moving.
     */
    double getRevolutions(); // RET: number of revolutions remaining in current movement

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
    void setStepAccumulator(
      const long steps);  // IN : new value of accumulator

    /**
     * Get the accumulator steps
     *
     * The accumulator increments for each clockwise step
     * and decrements for each counter-clockwise step.
     * The accumulator counts across movements.
     */
    long getStepAccumulator();  // RET: number of accumulated steps

    /**
     * Get the maximum speed in revolutions per minute for current mode
     */
    double getMaxSpeed(); // RET: maximum possible speed in revolutions per minute

  //
  // ----------------------- internal api ---------------------------
  //
  private:
    unsigned long revolutionsToSteps(const double revolutions);
    double stepsToRevolutions(unsigned long steps);
    void writeHalfStep(const char step);
    void writeFullStep(const char step);
    void writeWaveStep(const char step);
    void stopStep();
    void incrementStep();
};

#endif
