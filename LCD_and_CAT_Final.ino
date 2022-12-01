/* PiMart code for The PiMart C.A.T. or the Computer Advised Tracker
 *  Created By: Adam Belkahdir 
 *  CoCreated By: Jayson Lopez
*/
// an included library that allows the LCD (Liquid Crystal Display) to work
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x3F for a 16 chars and 2 line display


// defines main car pins
const int LeftWheelForward = 4;
const int RightWheelForward = 13;
const int LeftWheelReverse = 2;
const int RightWheelReverse = 12;
const int RightEngine = 11;     //These seem to not be needed as per the Robotic Behavior assignment
const int LeftEngine = 3;
const int rightEchoPin = 9;
const int rightTriggerPin = 8;
const int leftEchoPin = 7;
const int leftTriggerPin = 10;

const int StateGap = 5;
const int MinDistance = 1;    //Some clamp values so the distance measurements don't go above or below these.
const int MaxDistance = 100;
const int ReverseDistance = 10;
const int StopDistance = 25 - StateGap;    //The distance in which the cat will stop. (in cm)
const int FollowDistance = 25 + StateGap;    //The bottom bound where the cat will start to follow. Any distance above this value is what the CAT considers alright to pursue
const int MaxFollowDistance = 75;    //The upper bound where the cat will start to ignore. Any distance above this value the CAT just ignores
const int DistanceMeasurementErrorBounds = 28;    //This distance determines the allowed difference in distances before actually turning left or right.
const int TravelSpeed = 999;

//defines an assortment of characters and phrases for the LCD to display
String group = "PiMart C.A.T. ";
String c = "                  ";
String names = "Adam,Alyna,Jayson,Jessie,and,Caden";

//The following byte arrays are character definitions for the LCD.
byte Heart[8] =
{
0b00000,
0b00000,
0b01010,
0b11111,
0b11111,
0b01110,
0b00100,
0b00000
};

byte GoLeftEye[8] =
{
0b00000,
0b00000,
0b11000,
0b00110,
0b00001,
0b00110,
0b11000,
0b00000
};
byte GoRightEye[8] =
{
0b00000,
0b00000,
0b00011,
0b01100,
0b10000,
0b01100,
0b00011,
0b00000
};
byte ReverseEye[8] =
{
0b00000,
0b00000,
0b00000,
0b00000,
0b00000,
0b00000,
0b00000,
0b00000
};
byte ForwardEye[8] =
{
0b00000,
0b01110,
0b01010,
0b11011,
0b11011,
0b01010,
0b01110,
0b00000
};
byte ReverseMouth[8] =
{
0b00000,
0b00100,
0b00100,
0b01010,
0b01010,
0b10001,
0b10001,
0b00000
};
byte GoMouthL[8] =
{
0b10000,
0b10000,
0b11001,
0b01011,
0b01110,
0b00100,
0b00000,
0b00000
};
byte GoMouthR[8] =
{
0b00001,
0b00001,
0b10011,
0b11010,
0b01110,
0b00100,
0b00000,
0b00000
};
byte Blush[8] =
{
0b01010,
0b01010,
0b11111,
0b01010,
0b01010,
0b11111,
0b01010,
0b01010
};
byte PlaceHolder[8] =
{
0b00000,
0b00000,
0b00000,
0b00000,
0b00000,
0b00000,
0b00000,
0b00000
};

void ForwardFace(){   //A method printing to the LCD the array assortment of characters to make the "forward" face.
  lcd.clear();
  lcd.setCursor(4,0);
  lcd.write(4);
  lcd.setCursor(11,0);
  lcd.write(4);
  lcd.setCursor(7,1);
  lcd.write(6);
  lcd.write(7);
  lcd.setCursor(3,1);
  lcd.write(0);
  lcd.setCursor(12,1);
  lcd.write(0);
}

void ReverseFace(){   //A method printing to the LCD the array assortment of characters to make the "reverse" face.
  lcd.clear();
  lcd.setCursor(4,0);
  lcd.write(1);
  lcd.setCursor(11,0);
  lcd.write(2);
  lcd.setCursor(7,1);
  lcd.write(5);
  lcd.write(5);
  lcd.setCursor(3,1);
  lcd.write(3);
  lcd.setCursor(12,1);
  lcd.write(3);
}
void LeftFace(){   //A method printing to the LCD the array assortment of characters to make the "left" face.
  lcd.clear();
  lcd.setCursor(4,0);
  lcd.write(1);
  lcd.setCursor(11,0);
  lcd.write(1);
  lcd.setCursor(7,1);
  lcd.write(6);
  lcd.write(7);
  lcd.setCursor(3,1);
  lcd.write(0);
  lcd.setCursor(12,1);
  lcd.write(0);
}
void RightFace(){   //A method printing to the LCD the array assortment of characters to make the "right" face.
  lcd.clear();
  lcd.setCursor(4,0);
  lcd.write(2);
  lcd.setCursor(11,0);
  lcd.write(2);
 lcd.setCursor(7,1);
  lcd.write(6);
  lcd.write(7);
  lcd.setCursor(3,1);
  lcd.write(0);
  lcd.setCursor(12,1);
  lcd.write(0);
}
void StopFace(){   //A method printing to the LCD the array assortment of characters to make the "stop" face.
  lcd.clear();
  lcd.setCursor(4,0);
  lcd.print("v");
  lcd.setCursor(11,0);
  lcd.print("v");
  lcd.setCursor(7,1);
  lcd.print("-");
  lcd.print("-");
  lcd.setCursor(3,1);
  lcd.print("Z");
  lcd.setCursor(12,1);
  lcd.print("Z");
}






//The following is the actual code

class Wheel
{
  public:             // Access specifier
    int forwardPinNumber;
    int reversePinNumber;
    int enginePinNumber;
    
    Wheel(int enginePin, int forwardPin, int reversePin)    //The initializer method for a singular wheel. This method sets all the pin numbers and modes for each pin.
    {
      forwardPinNumber = forwardPin;    //Stores all the pins as fields to access.
      reversePinNumber = reversePin;
      enginePinNumber = enginePin;
      pinMode(enginePinNumber, OUTPUT);   //Sets all of these wheel pins as output pins for the Arduino.
      pinMode(forwardPinNumber, OUTPUT);
      pinMode(reversePinNumber, OUTPUT);
    }
  
    void GoForward(int TravelSpeed)   //A method that moves this wheel forward.
    {
      analogWrite(enginePinNumber, TravelSpeed);    //Changes the pin speed
      analogWrite(forwardPinNumber, TravelSpeed);
      digitalWrite(forwardPinNumber, HIGH);   //Has the wheel move forward by applying voltage to the forward pin.
      digitalWrite(reversePinNumber, LOW);
    }
    
    void Reverse(int TravelSpeed)
    {
      analogWrite(enginePinNumber, TravelSpeed);    //Changes the pin speed
      analogWrite(reversePinNumber, TravelSpeed);
      digitalWrite(forwardPinNumber, LOW);    //Has the wheel move in reverse by applying voltage to the reverse pin.
      digitalWrite(reversePinNumber, HIGH);
    }
    
    void Stop()
    {
      analogWrite(enginePinNumber, 0);    //Stops all pins
      analogWrite(forwardPinNumber, 0);
      analogWrite(reversePinNumber, 0);
      digitalWrite(forwardPinNumber, LOW);    //Disables both the forward and reverse pins of the wheel.
      digitalWrite(reversePinNumber, LOW);
    }
};

class UltraSensor
{
  public:             // Access specifier
    int echoPinNumber;
    int triggerPinNumber;
    
    UltraSensor(int echoPin, int triggerPin)    //The initializer method for an ultrasound sensor. This method sets all the pin numbers and modes for each pin.
    {
      echoPinNumber = echoPin;    //Stores all the pins as fields to access.
      triggerPinNumber = triggerPin;
      pinMode(echoPin, INPUT);
      pinMode(triggerPin, OUTPUT);
    }

    int Scan(int minDist, int maxDist)    //A method that will return the distance within the known bounds.
    {
      digitalWrite(triggerPinNumber, LOW);   //Disables the ultrasound emitter to make sure there are no extra waves.
      delayMicroseconds(2);
      digitalWrite(triggerPinNumber, HIGH);    //Triggers the ultrasound emitter.
      delayMicroseconds(10);
      digitalWrite(triggerPinNumber, LOW);   //Disables the ultrasound emitter.

      long duration = pulseIn(echoPinNumber, HIGH);   //Has the ultrasound sensors receiver wait and interpret the wave signals that return.
      int distance = duration * 0.034 / 2;    //Converts the time it took for the waves to arrive into a distance.

      if (distance < minDist)   //Clamps the distance so we don't get... unwanted values
        distance = minDist;
      else if (distance > maxDist)
        distance = maxDist;

      return distance;
    }

    int TakeAverageDistance(int amountOfScans, int delayPerScan)    //Takes multiple scans and averages them out for a closer approximation of what the distance really is.
    {
        int avgDist = 0;
        for (int i = 0; i < amountOfScans; i++)
        {
          avgDist += Scan(MinDistance, MaxDistance);
          delayMicroseconds(delayPerScan);
        }
        avgDist /= amountOfScans;
        return avgDist;
    }
};

Wheel leftWheel(LeftEngine, LeftWheelForward, LeftWheelReverse);    //Defines the wheels as objects which we control separately.
Wheel rightWheel(RightEngine, RightWheelForward, RightWheelReverse);
UltraSensor leftSensor(leftEchoPin, leftTriggerPin);    //Defines both sensors as objects we can control individually.
UltraSensor rightSensor(rightEchoPin, rightTriggerPin);

void setup()
{
  Serial.begin(9600);   //Sets up the serial monitor
  lcd.begin(16, 2);   //Sets up the LCD
  lcd.print("PiMart Robot");
  lcd.setBacklight(1);
  lcd.createChar(0, Heart);
  lcd.createChar(1, GoLeftEye);
  lcd.createChar(2, GoRightEye);
  lcd.createChar(3, Blush);
  lcd.createChar(4, ForwardEye);
  lcd.createChar(5, ReverseMouth);
  lcd.createChar(6, GoMouthL);
  lcd.createChar(7, GoMouthR);
  delay (1000);
}

//Methods for cat
void MoveForward()    //Moves the cat forward
{
  ForwardFace();
  leftWheel.GoForward(TravelSpeed);   //Moves both wheels forward
  rightWheel.GoForward(TravelSpeed);
}

void MoveReverse()    //Moves the cat in reverse
{
  ReverseFace();
  leftWheel.Reverse(TravelSpeed);   //Moves both wheels in reverse
  rightWheel.Reverse(TravelSpeed);
}

void MoveLeft()   //Moves the cat to the left (in place)
{
  LeftFace();
  leftWheel.Reverse(TravelSpeed);
  rightWheel.GoForward(TravelSpeed);  //i changed from stop to forward
  /*rightWheel.GoForward(TravelSpeed);
  leftWheel.Stop();*/
}

void MoveRight()   //Moves the cat to the right (in place)
{
  RightFace();
  leftWheel.GoForward(TravelSpeed); //i changed from stop to forward
  rightWheel.Reverse(TravelSpeed);

  /*leftWheel.GoForward(TravelSpeed);
  rightWheel.Stop();*/
}

void MoveStop()   //Stops all wheels to stop the cat.
{
  StopFace();
  leftWheel.Stop();
  rightWheel.Stop();
}

bool NumberInErrorBounds(int num1, int num2)    //Compares two numbers to determine if they are within a certain margin of error from each other.
{
  int err = DistanceMeasurementErrorBounds;
  return num1 >= num2 - err && num1 <= num2 + err;
}

void loop()
{
  delay(10);
  Serial.println(" ");    //These prints are to separate the data in the serial monitor to make the data readable.
  Serial.println(" ");
  Serial.println(" ");
  Serial.println(" ");
  Serial.println(" ");
  Serial.println(" ");
  Serial.println(" ");
  int leftDistance = leftSensor.Scan(MinDistance, MaxDistance);    //Has the left sensor scan between the minimum and maximum distances.
  int rightDistance = rightSensor.Scan(MinDistance, MaxDistance);    //Has the right sensor scan between the minimum and maximum distances.
  Serial.print("Left Distance: ");
  Serial.print(leftDistance);
  Serial.println(" cm");
  Serial.print("Right Distance: ");
  Serial.print(rightDistance);
  Serial.println(" cm");

  if (leftDistance < ReverseDistance || rightDistance < ReverseDistance)    //Checks if either of the sensors are too close to something. Will reverse if there is.
  {
    MoveReverse();
    Serial.println("Reverse");
    return;
  }

  if (leftDistance < StopDistance || rightDistance < StopDistance)    //Checks if either of the sensors are detecting something in the stopping range. Stops the cat if there's something there.
  {
    MoveStop();
    Serial.println("Stop (Stop distance)");
    return;
  }

  if (leftDistance < MaxFollowDistance || rightDistance < MaxFollowDistance)    //Checks if one of the sensors are detecting something.
  {
    if (NumberInErrorBounds(leftDistance, rightDistance))   //Checks if both distance measurements are close enough. If they are the cat moves forward.
    {
      Serial.println("Forward");
      MoveForward();    //Moves the cat forward
    }
    else    //At this point it can only be either of the two extremes. We already know that the distances are not within their error bounds.
    {
      if (leftDistance > rightDistance + DistanceMeasurementErrorBounds)    //Checks if there is something in the left side of the cat, in which case it moves toward it.
      {
        MoveLeft();   //Moves to the left
        Serial.println("Left");
      }
      else
      {
        MoveRight();    //Moves to the right
        Serial.println("Right");
      }
    }
  }
  else
  {
    MoveStop();   //Stops the cat if both sensors are out of bounds.
    Serial.println("Stop");
  }
}
