#include <Zumo32U4Motors.h>
#include <Zumo32U4Encoders.h>
#include <Zumo32U4.h>
#include <Wire.h>

#include "button.h"       //include your button class from last week
#include "event_timer.h"  //include your shiny, new event timer class

#include "segments.h"
#include "params.h"
#include "ultrasonic.h"

Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4LineSensors lineSensors;

volatile int16_t countsLeft = 0;
volatile int16_t countsRight = 0;

DebouncedButton buttonC(HIGH);
EventTimer timer;

volatile uint8_t readyToPID = 0;

// Define the robot direction of movement
enum ROBOT_STATE
{
    ROBOT_STOP,
    ROBOT_FORWARD,
    ROBOT_RIGHT,
    ROBOT_LEFT,
    ROBOT_SPIN

};

ROBOT_STATE robotState = ROBOT_STOP; // default to idle


void setup()
{
  Serial.begin(115200);
  Serial.println("Hej.");

  noInterrupts(); //disable interupts while we mess with the Timer4 registers
  
  //sets up timer 4
  TCCR4A = 0x00; //disable some functionality -- no need to worry about this
  TCCR4B = 0x0C; //sets the prescaler -- look in the handout for values
  TCCR4C = 0x04; //toggles pin 6 at the timer frequency
  TCCR4D = 0x00; //normal mode

  OCR4C = 124;   //TOP goes in OCR4C 
  TIMSK4 = 0x04; //enable overflow interrupt
  
  interrupts(); //re-enable interrupts

  buttonC.Init(17); //don't forget to call Init()
}

void loop() 
{  
  //simple event-driven structure
  if(buttonC.getState() == DebouncedButton::Pressed)          HandleButtonC();
  if(timer.CheckExpired())        HandleTimer();  //use this for the 1 second wait
  if(CheckDistance())             HandleDistance();
  if(readyToPID)                  HandlePID();
}


bool robot_move(const ROBOT_STATE move_type)
{
    if (robotState == ROBOT_STOP) 
    {
        Serial.print("HALT! \n");
        motors.setSpeeds (0,0);
        
    }

    else if (robotState == ROBOT_FORWARD) 
    {
        Serial.print("ONWARD! \n");
        motors.setSpeeds (50,50);

    }

    else if (robotState == ROBOT_RIGHT) 
    {
        Serial.print("SKRTTT! \n");
        motors.setSpeeds (50,20);

    }

    else if (robotState == ROBOT_LEFT) 
    {
        Serial.print("SCREECHHH! \n");
        motors.setSpeeds (20,50);

    }

    else if (robotState == ROBOT_SPIN)
    {
        Serial.print("I'M SPINNING! \n");
        motors.setSpeeds (-50,50);
    }

    else 
    {
        Serial.print("MOVE TYPE WRONG! \n");
        return false;
    }
}



bool CheckDistance(void)
{
  bool retVal = false;

  static int16_t leftWheelPrev = 0;

  //get the wheel motion
  noInterrupts();
  int16_t leftWheelDistance = abs(countsLeft - startLeft);
  interrupts();

  int16_t leftWheelTarget = segments[iSegment].distance;
  
  if(leftWheelDistance >= leftWheelTarget && leftWheelPrev < leftWheelTarget) retVal = true;

  leftWheelPrev = leftWheelDistance;

  static int16_t rightWheelPrev = 0;

  //get the wheel motion
  noInterrupts();
  int16_t rightWheelDistance = abs(countsRight - startRight);
  interrupts();

  int16_t rightWheelTarget = segments[iSegment].distance;
  
  if(rightWheelDistance >= rightWheelTarget && rightWheelPrev < rightWheelTarget) retVal = true;

  rightWheelPrev = rightWheelDistance;

  return retVal;
}


void HandleButtonC(void)
{
  
  Serial.println("HandleButtonC()");
//  if(buttonC.getState() == DebouncedButton::Pressed)
  {
    robotState = ROBOT_STOP;
    timer.Start(1000);
        
  }

//  else
//  {
//    //nothing to do here...
//    //how might you edit this so that pressing the button always starts over?
//  }
}

void HandleTimer(void)
{
  Serial.println("HandleTimer()");
  if(timer.CheckExpired() == false)
  {
    iSegment = 0;
    Drive(iSegment);
    robotState = ROBOT_FORWARD;
  }
}


void HandleDistance(void)
{
  Serial.println("HandleDistance()");
  Serial.println(robotState);

  if(robotState == ROBOT_FORWARD)
  {
    if(++iSegment < segCount) //guard condition from the state machine
    {
      Drive(iSegment);
    }

    else
    {
      motors.setSpeeds(0,0); //don't forget to stop!
      targetLeft = 0;
      targetRight = 0;
      robotState = ROBOT_STOP;    
    }
  }
}

void Drive(int s)
{
  Serial.print("Drive(): ");
  Serial.println(segments[s].distance);

  //start the timer, then command the motors; note that "setSpeeds()" actually just sets the duty cycle, not speed
  targetLeft = segments[s].leftMotorSpeed;
  targetRight = segments[s].rightMotorSpeed;
  timer.Start(segments[s].distance);
  noInterrupts();
  startLeft = encoders.getCountsLeft();
  interrupts();
}

  
void getDistance()
{
  if (getDistance > 30)
    {
      robotState = ROBOT_RIGHT;
    }
  else if (getDistance < 30)
    {
      robotState = ROBOT_LEFT ;
    }
  else
    {
      robotState = ROBOT_FORWARD;
    }
}

void HandlePID(void)
{
   //clear the timer flag
    readyToPID = 0;

    //for tracking previous counts
    static int16_t prevLeft = 0;
    static int16_t prevRight = 0;

    //error sum
    static int16_t sumLeft = 0;
    static int16_t sumRight = 0;

    /*
     * Do PID stuffs here. Note that we turn off interupts while we read countsLeft/Right
     * so that it won't get accidentally updated (in the ISR) while we're reading it.
     */
    noInterrupts();
    int16_t speedLeft = countsLeft - prevLeft;
    int16_t speedRight = countsRight - prevRight;

    prevLeft = countsLeft;
    prevRight = countsRight;
    interrupts();

    int16_t errorLeft = targetLeft - speedLeft;
    sumLeft += errorLeft;
    
    float effortLeft = Kp * errorLeft + Ki * sumLeft;
    
    int16_t errorRight = targetRight - speedRight;
    sumRight += errorRight;
    
    float effortRight = Kp * errorRight + Ki * sumRight;

    motors.setSpeeds(effortLeft, effortRight); //up to you to add the right motor

//    Serial.print(millis());
//    Serial.print('\t');
//    Serial.print(targetLeft);
//    Serial.print('\t');
//    Serial.print(targetRight);
//
//    Serial.print('\n');
}

/*
 * ISR for timing. Basically, raise a flag on overflow. Timer4 is set up to run with a pre-scaler 
 * of 1024 and TOP is set to 249. Clock is 16 MHz, so interval is dT = (1024 * 250) / 16 MHz = 16 ms.
 */
ISR(TIMER4_OVF_vect)
{
  //Capture a "snapshot" of the encoder counts for later processing
  countsLeft = encoders.getCountsLeft();
  countsRight = encoders.getCountsRight();

  readyToPID = 1;
}
