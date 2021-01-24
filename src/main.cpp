#include <Arduino.h>


//Solar Tracker Project- By USMAN QURESHI
//##################################################################################################################################

#define MAX_SERVO_ANGLE 180
#define MIN_SERVO_ANGLE 0
#define DEFAULT_ANGLE 90
#define LIGHT_INTENSITY_THRESHOLD 150
#define DEGREE_INCREMENT 2
#define DEGREE_DECREMENT -2

//Photo-Resistor Class
class LDR
{
public:
  //function that creates an LDR on the inputted pin, p
  LDR(int p)
  {
    photoPin = p;
  }
  
  //function that reads the value of the LDR, and returns that value
  int lightIntensity()
  {
    return analogRead(photoPin);
  }

private:
  //the pin-value of the LDR is private
  int photoPin;
};

//Class that defines a servo
class Servo
{
public:
  //function that creates a servo on pin, p
  Servo(int p)
  {
    servoPin = p;
  }

  //function that initializes a created servo. To control it the pin is an output.
  void initialize()
  {
    pinMode(servoPin, OUTPUT);
  }

  //function that controls movement of servo; using specs on HS-422 data sheet, (925/90)
  //is a conversion value that allows a degree-based control.
  void setAngle(int deg)
  {
    //save input of degree the servo should be at
    currentAngle = deg;

    if( currentAngle > MAX_SERVO_ANGLE )
    {
      currentAngle = MAX_SERVO_ANGLE;
      return;
    }
    else if( currentAngle < MIN_SERVO_ANGLE ) {
      currentAngle = MIN_SERVO_ANGLE;
      return;
    }

    //artificial PWM control of servo, specifically HS-422
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(575 + (925 / 90) * currentAngle);
    digitalWrite(servoPin, LOW);
    delayMicroseconds(20000);
  }

  void rotate(int deg)
  {
    setAngle(currentAngle + deg);    
  }

  int angle()
  {
    return currentAngle;
  }

//function that will turn off the servo
void stop() 
{
  digitalWrite(servoPin,LOW);
}

void updateUsingLightSensor(LDR& lightSensorA, LDR& lightSensorB)
{
  if (abs(lightSensorA.lightIntensity() - lightSensorB.lightIntensity()) < LIGHT_INTENSITY_THRESHOLD)
  {
    stop();
  }
  else if (lightSensorA.lightIntensity() < lightSensorB.lightIntensity())
  {
    rotate(DEGREE_INCREMENT);
  }
  else
  {
    rotate(DEGREE_DECREMENT);
  }
}

private:
  //the servo pin and the degree the servo will rotate to is private
  int servoPin;
  int currentAngle;
};

//attach 4 LDRS on A0 A1 A2 A3 and two servos on pin 4 and 5
LDR topLightSensor(15);
LDR bottomLightSensor(32);
LDR rightLightSensor(14);
LDR leftLightSensor(33);
Servo topServo(12);
Servo bottomServo(13);

//initialize the servo to 90 degree, and set a 70 error margin with photo-resistors. servo1direction dictates how servo1 will
//move WRT the angle of servo

void setup()
{
  //initialize the servo, and wait 10s before functioning the main program.
  topServo.initialize();
  bottomServo.initialize();
  topServo.setAngle(DEFAULT_ANGLE);
  bottomServo.setAngle(DEFAULT_ANGLE);
}

void loop()
{
 topServo.updateUsingLightSensor(topLightSensor, bottomLightSensor);

  if (topServo.angle() < 90)
  {
    bottomServo.updateUsingLightSensor(leftLightSensor, rightLightSensor);
  }
  else
  {
    bottomServo.updateUsingLightSensor(rightLightSensor, leftLightSensor);
  }

  //wait 40ms before checking again
  delay(40);
}