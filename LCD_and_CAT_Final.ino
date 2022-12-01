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


void ForwardFace(){
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

void ReverseFace(){
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
void LeftFace(){
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
void RightFace(){
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
void StopFace(){
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
    
    Wheel(int enginePin, int forwardPin, int reversePin)
    {
      forwardPinNumber = forwardPin;
      reversePinNumber = reversePin;
      enginePinNumber = enginePin;
      pinMode(enginePinNumber, OUTPUT);
      pinMode(forwardPinNumber, OUTPUT);
      pinMode(reversePinNumber, OUTPUT);
    }
  
    
    void GoForward(int TravelSpeed)
    {
      analogWrite(enginePinNumber, TravelSpeed);
      analogWrite(forwardPinNumber, TravelSpeed);
      digitalWrite(forwardPinNumber, HIGH);
      digitalWrite(reversePinNumber, LOW);
    }
    
    void Reverse(int TravelSpeed)
    {
      analogWrite(enginePinNumber, TravelSpeed);
      analogWrite(reversePinNumber, TravelSpeed);
      digitalWrite(forwardPinNumber, LOW);
      digitalWrite(reversePinNumber, HIGH);
    }
    
    void Stop()
    {
      analogWrite(enginePinNumber, 0);
      analogWrite(forwardPinNumber, 0);
      analogWrite(reversePinNumber, 0);
      digitalWrite(forwardPinNumber, LOW);
      digitalWrite(reversePinNumber, LOW);
    }
};

class UltraSensor
{
  public:             // Access specifier
    int echoPinNumber;
    int triggerPinNumber;
    
    UltraSensor(int echoPin, int triggerPin)
    {
      echoPinNumber = echoPin;
      triggerPinNumber = triggerPin;
      pinMode(echoPin, INPUT);
      pinMode(triggerPin, OUTPUT);
    }

    int Scan(int minDist, int maxDist)
    {
      digitalWrite(triggerPinNumber,LOW);
      delayMicroseconds(2);
      digitalWrite(triggerPinNumber,HIGH);
      delayMicroseconds(10);
      digitalWrite(triggerPinNumber,LOW);

      long duration = pulseIn(echoPinNumber, HIGH);
      int distance = duration * 0.034 / 2;

      if (distance < minDist)   //Clamps the distance so we don't get... unwanted values
        distance = minDist;
      else if (distance > maxDist)
        distance = maxDist;

      return distance;
    }

    int TakeAverageDistance(int amountOfScans, int delayPerScan)
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

Wheel leftWheel(LeftEngine, LeftWheelForward, LeftWheelReverse);
Wheel rightWheel(RightEngine, RightWheelForward, RightWheelReverse);
UltraSensor leftSensor(leftEchoPin, leftTriggerPin);
UltraSensor rightSensor(rightEchoPin, rightTriggerPin);

void setup()
{
  Serial.begin(9600);
  lcd.begin(16, 2);
  Serial.begin(9600);
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

//methods for car
void MoveForward()
{
  ForwardFace();
  leftWheel.GoForward(TravelSpeed);
  rightWheel.GoForward(TravelSpeed);
}

void MoveReverse()
{
  ReverseFace();
  leftWheel.Reverse(TravelSpeed);
  rightWheel.Reverse(TravelSpeed);
}

void MoveLeft()
{
  LeftFace();
  leftWheel.Reverse(TravelSpeed);
  rightWheel.GoForward(TravelSpeed); //i changed from stop to forward
  /*rightWheel.GoForward(TravelSpeed);
  leftWheel.Stop();*/
}

void MoveRight()
{
  RightFace();
  leftWheel.GoForward(TravelSpeed); //i changed from stop to forward
  rightWheel.Reverse(TravelSpeed);

  /*leftWheel.GoForward(TravelSpeed);
  rightWheel.Stop();*/
}

void MoveStop()
{
  StopFace();
  leftWheel.Stop();
  rightWheel.Stop();
}

bool NumberInErrorBounds(int num1, int num2)
{
  int err = DistanceMeasurementErrorBounds;
  return num1 >= num2 - err && num1 <= num2 + err;
}

void loop()
{
  delay(10);
  Serial.println(" ");
  Serial.println(" ");
  Serial.println(" ");
  Serial.println(" ");
  Serial.println(" ");
  Serial.println(" ");
  Serial.println(" ");
  int leftDistance = leftSensor.Scan(MinDistance, MaxDistance);    //Has the left sensor scan between the minimum and maximum distances.
  int rightDistance = rightSensor.Scan(MinDistance, MaxDistance);
  Serial.print("Left Distance: ");
  Serial.print(leftDistance);
  Serial.println(" cm");
  Serial.print("Right Distance: ");
  Serial.print(rightDistance);
  Serial.println(" cm");

  if (leftDistance < ReverseDistance || rightDistance < ReverseDistance)
  {
    MoveReverse();
    Serial.println("Reverse");
    return;
  }

  if (leftDistance < StopDistance || rightDistance < StopDistance)
  {
    MoveStop();
    Serial.println("Stop (Stop distance)");
    return;
  }

  if (leftDistance < MaxFollowDistance || rightDistance < MaxFollowDistance)
  {
    if (NumberInErrorBounds(leftDistance, rightDistance))   //Checks if both distance measurements are close enough.
    {
      Serial.println("Forward");
      MoveForward();
    }
    else    //At this point it can only be either of the two extremes.
    {
      if (leftDistance > rightDistance + DistanceMeasurementErrorBounds)
      {
        MoveLeft();
        Serial.println("Right");
      }
      else
      {
        MoveRight();
        Serial.println("Left");
      }
    }
  }
  else
  {
    MoveStop();
    Serial.println("Stop");
  }
}
