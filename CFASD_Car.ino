#include <AFMotor.h>
#include <Servo.h>
#include <HC05.h>
#include <NewPing.h>
#include <Arduino.h>

// Pin Definitions
#define HC05_PIN_RXD  12
#define HC05_PIN_TXD  13

#define HCSR04_PIN_TRIG 7
#define HCSR04_PIN_ECHO 8

#define SERVO_PIN_ATTACH 9

// Maximum Distance (cm) for Ultrasonic Sensor to detect
#define MAX_DISTANCE 300





// object initialization

SoftwareSerial mySerial(HC05_PIN_RXD, HC05_PIN_TXD);  //Bluetooth Function

NewPing hcsr04(HCSR04_PIN_TRIG, HCSR04_PIN_ECHO, MAX_DISTANCE); // Sensor Function

AF_DCMotor leftMotor  (2, MOTOR12_1KHZ);
AF_DCMotor rightMotor (4, MOTOR34_1KHZ);


// Global variables and declaration

Servo myServo;
const int servoSMTargetPosition = 90; //Position when event is detected
int distance = 100;
boolean goesForward = false;
char command;


void setup()
{


  // put your setup code here, to run once:
  Serial.begin(9600);

  while (!Serial); // wait for serial port to connect. Needed for native USB
  mySerial.begin(9600);

  // Default Bluetooth password '1234'

  mySerial.println("Bluetooth Turned On....");
  myServo.attach(SERVO_PIN_ATTACH);
  myServo.write(servoSMTargetPosition);
  delay(100);

}

void loop()
{
  // put your main code here, to run repeatedly:

  myServo.write(90);
  while (mySerial.available())
  {
    command = mySerial.read();
    Serial.println("Command Receieved....");
    delay(10);
     if (command != '\0') 
    {
      break;
    }
  
  }

  // Function Calls upon commands
  {
    if (command == '1')
    {
      forward_car();
    }
    else if (command == '2')
    {
      back_car();
    }
    else if (command == '3')
    {
      right_car();
    }
    else if (command == '4')
    {
      left_car();
    }

    else if (command == '5')
    {
      self_drive();
    }

    command = '\0';
  }
}

// Simple Forward Command
void forward_car()
{
  distance = readPing();
  
  leftMotor.run(FORWARD);
  leftMotor.setSpeed(255);
  rightMotor.run(FORWARD);
  rightMotor.setSpeed(255);
 
  while(distance > 100) 
   {
      distance = readPing();
   }

  stop_car();
}

//Simple Back Command
void back_car()
{
  
  leftMotor.run(BACKWARD);
  leftMotor.setSpeed(255);
  rightMotor.run(BACKWARD);
  rightMotor.setSpeed(255);
  
  delay(2000);
  stop_car ();
  
}

// Somple Right Command
void right_car()
{
  myServo.write(0);
  delay(1000);
  myServo.write(90);
  delay(1000);
  leftMotor.run(FORWARD);
  leftMotor.setSpeed(190);
  rightMotor.run(BACKWARD);
  rightMotor.setSpeed(190);
  
  do 
   {
      distance = readPing();
   }
  while(distance < 300);
  
  stop_car ();
}

// Simple Left Command
void left_car()
{
  myServo.write(180);
  delay(1000);
  myServo.write(90);
  delay(1000);
  leftMotor.run(BACKWARD);
  leftMotor.setSpeed(190);
  rightMotor.run(FORWARD);
  rightMotor.setSpeed(190);
  
  do 
   {
      distance = readPing();
   }
  while(distance < 300);
  
  stop_car ();
}

void self_drive()
{
  int distanceRight = 0;
  int distanceLeft = 0;
  delay(50);

  if (distance <= 20) 
  {
    stop_car();
    delay(300);
    selfBack_car();
    delay(400);
    stop_car();
    delay(300);
    distanceRight = lookRight();
    delay(300);
    distanceLeft = lookLeft();
    delay(300);

    if (distanceRight >= distanceLeft) 
    {
      selfRight_car();
      stop_car();
    }
    else 
    {
      selfLeft_car();
      stop_car();
    }
  }
  else
  {
    selfForward_car();
  }
  distance = readPing();
}



//Self Drive Mode functins Starts here

void selfForward_car()
{
  if (!goesForward)
  {
    goesForward = true;

    leftMotor.run(FORWARD);
    leftMotor.setSpeed(255);
    rightMotor.run(FORWARD);
    rightMotor.setSpeed(255);
  }

}

void selfBack_car()
{

  goesForward = false;

  leftMotor.run(BACKWARD);
  leftMotor.setSpeed(255);
  rightMotor.run(BACKWARD);
  rightMotor.setSpeed(255);

}

void selfRight_car()
{
  leftMotor.run(FORWARD);
  leftMotor.setSpeed(190);
  rightMotor.run(BACKWARD);
  rightMotor.setSpeed(190);

  delay(500);

  leftMotor.run(FORWARD);
  leftMotor.setSpeed(190);
  rightMotor.run(BACKWARD);
  rightMotor.setSpeed(190);
}

void selfLeft_car()
{
  leftMotor.run(BACKWARD);
  leftMotor.setSpeed(190);
  rightMotor.run(FORWARD);
  rightMotor.setSpeed(190);

  delay(500);

  leftMotor.run(BACKWARD);
  leftMotor.setSpeed(190);
  rightMotor.run(FORWARD);
  rightMotor.setSpeed(190);
}


// Stop Car Function
void stop_car ()
{
  leftMotor.run(RELEASE);
  rightMotor.run(RELEASE);
}

int lookRight() 
{
  myServo.write(50);
  delay(500);
  int distance = readPing();
  delay(100);
  myServo.write(115);
  return distance;
}

int lookLeft() 
{
  myServo.write(170);
  delay(500);
  int distance = readPing();
  delay(100);
  myServo.write(115);
  return distance;
  delay(100);
}

int readPing() 
{
  delay(70);
  int cm = hcsr04.ping_cm();
  if (cm == 0) 
  {
    cm = 250;
  }
  return cm;
}
