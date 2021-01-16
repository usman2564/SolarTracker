#include <Arduino.h>

//Solar Tracker Project- By USMAN QURESHI
//##################################################################################################################################

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
  int read()
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
  void sweep(int deg)
  {
    //save input of degree the servo should be at
    sweepDegree = deg;

    //artificial PWM control of servo, specifically HS-422
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(575 + (925 / 90) * sweepDegree);
    digitalWrite(servoPin, LOW);
    delayMicroseconds(20000);
  }

//function that will turn off the servo
void servoStop() 
{
  digitalWrite(servoPin,LOW);
}
private:
  //the servo pin and the degree the servo will sweep to is private
  int servoPin;
  int sweepDegree;
};

//attach 4 LDRS on A0 A1 A2 A3 and two servos on pin 4 and 5
LDR photo1(1);
LDR photo2(2);
LDR photo3(0);
LDR photo4(3);
Servo servo(4);
Servo servo1(5);

//initialize the servo to 90 degree, and set a 70 error margin with photo-resistors. servo1direction dictates how servo1 will
//move WRT the angle of servo
int servoAngle = 90;
int servo1Angle = 90;
int servo1direction = 0;

void setup()
{
  //initialize the servo, and wait 10s before functioning the main program.
  servo.initialize();
  servo1.initialize();
  servo.sweep(servoAngle);
  servo1.sweep(servo1Angle);
  //Serial.begin(9600);
  delay(10000);
}

void loop()
{

  //check with LDR is brighter. The lesser value will be brighter.
  if (abs(photo1.read() - photo2.read()) < 25)
  {
    servo.servoStop();
  }
  else if (photo1.read() < photo2.read())
  {
    //check to make sure the angle doesnt pass 180 degree
    if (servoAngle != 180)
    {
      //change the servo's angle slightly (CW) and move it according to the brightness
      servoAngle = servoAngle + 2;
      servo.sweep(servoAngle);
    }
  }
  else if (photo2.read() < photo1.read())
  {
    //make sure the angle doesnt fall below 0 degree
    if (servoAngle != 0)
    {
      //change the servo's angle slightly (CCW) and move it according to the brightness
      servoAngle = servoAngle - 2;
      servo.sweep(servoAngle);
    }
  }

//servo1direction changes WRT servoAngle
if (servoAngle > 0 && servoAngle < 90)
{
  servo1direction = -1;
} else if (servoAngle > 90 && servoAngle < 180) {
  servo1direction = 1;
}


//check with LDR is brighter. The lesser value will be brighter.
  if(abs(photo3.read() - photo4.read()) < 25)
  {
    servo1.servoStop();
  }
  else if ((photo3.read() < photo4.read() && servo1direction == 1) || (photo4.read() < photo3.read() && servo1direction == -1))
  {
    //check to make sure the angle doesnt pass 180 degree
    if (servo1Angle != 0)
    {
      //change the servo's angle slightly and move it according to the brightness
      servo1Angle = servo1Angle - 2;
      servo1.sweep(servo1Angle);
    }
  }
  else if ((photo3.read() < photo4.read() && servo1direction == -1) || (photo4.read() < photo3.read() && servo1direction == 1))
  {
    //make sure the angle doesnt fall below 0 degree
    if (servo1Angle != 180)
    {
      //change the servo's angle slightly and move it according to the brightness
      servo1Angle = servo1Angle + 2;
      servo1.sweep(servo1Angle);
    }
  }

  //wait 40ms before checking again
  delay(40);
}