/*
   BYJ48 Stepper motor and ULN2003 stepper controller and
   LM393 optical encoder to provide limit detection
   so the steps per revolution of the stepper can be determined
   empiricly.
      
         
   Plug BYJ48 into socket of ULN2003 board
   ULN2003 IN1 to microcontroller GPIO configured for output (8 in sketch; change if necessary)
   ULN2003 IN2 to microcontroller GPIO configured for output (9 in sketch; change if necessary)
   ULN2003 IN3 to microcontroller GPIO configured for output (10 in sketch; change if necessary)
   ULN2003 IN4 to microcontroller GPIO configured for output (11 in sketch; change if necessary)
   ULN2003 (+) to microcontroller 5V Prefer to use external 5V Source
   ULN2003 (-) to microcontroller GND (shared with external 5V Source if used)

  
   LM393 vcc to either 5v or 3.3v on arduino
   LM393 gnd to arduino gnd
   LM393 DO (digital out) to microcontroller gpio (14 (A0) in sketch, change if necessary)
   LM393 AO (analog out) unconnected: does not work as the sensor does not output an analog value.
******************************/
#ifndef DEBUG
  #define DEBUG
#endif

#include <math.h>
#include <debug.h>
#include <stepper28byj48.h>

//
// associate pins on ULN2003 to GPIO pins on Arduino.
// if you wire differently than above, change code here.
//
// #define ESP32 true
#ifdef ESP32
// NodeMCU-32s
const int IN1 = 19;
const int IN2 = 18;
const int IN3 = 5;
const int IN4 = 17;
#else
// arduino uno
const int IN1 = 8;
const int IN2 = 9;
const int IN3 = 10;
const int IN4 = 11;
#endif

//
// stepper instance
//
Stepper28BYJ48 stepper(kHalfStep, IN1, IN2, IN3, IN4);
// Stepper28BYJ48 stepper(kFullStep, IN1, IN2, IN3, IN4);
// Stepper28BYJ48 stepper(kWaveStep, IN1, IN2, IN3, IN4);

boolean isZero;

void setup() 
{
  DSETUP();
  DPRINTLN("setup");

  stepper.setup();
  initLM393();

  isZero = isLimitDetected();
}

void loop() 
{
    //
    // step the motor until step count is complete
    //
    if(!stepper.isMoving())
    {
      //
      // restart the movement to make it continuous.
      //
      DPRINTLN("starting");
      stepper.startMovement(kCounterClockwise, 1.0, stepper.getMaxSpeed());
    }
    else
    {
      if(isLimitDetected())
      {
        if(!isZero)
        {
          //
          // just detected zero, print out acculumated steps
          //
          const long stepsPerRevolution = stepper.getStepAccumulator();
          DPRINT("Zero step: "); DPRINTLN(stepsPerRevolution);
          stepper.setStepAccumulator(0);
        }
        isZero = true;
      }
      else
      {
        isZero = false;
      }
    }
}

/*************************** LM393 **************************************/
const int ENCODER_PIN = 14; // output of LM393 to gpio pin on arduino
const int OBSTACLE = HIGH; // value for obstacle detected
const int NO_OBSTACLE = LOW; // value for no obstacle detected

boolean lastDetected = false; // no obstacle

void initLM393() {
  pinMode(ENCODER_PIN, INPUT);
}

/**
 * Determine if the encoder detects an obstacle.
 * 
 * @return true if encoder detected obstacle, false if not
 */
boolean isLimitDetected() {
  //
  // 1. read the output pin of the sensor
  //    if it is HIGH, there is an obstacle
  //    otherwise there is not an obstacle
  // 2. if value has changed then
  // 3. - save new value
  // 4. - print new value
  //
  boolean obstacleDetected = (OBSTACLE == digitalRead(ENCODER_PIN));
  if (obstacleDetected != lastDetected) {
    if ((lastDetected = obstacleDetected) == true) {
      DPRINTLN("limit detected");
    }
  }
  return lastDetected;
}

