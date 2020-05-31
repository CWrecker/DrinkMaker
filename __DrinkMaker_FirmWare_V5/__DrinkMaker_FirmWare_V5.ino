#include "Arduino.h"
#include "Servo.h"
//#include <OneWire.h>
//#include <DallasTemperature.h>
//#define ONE_WIRE_BUS 45

//------------------------------------------
//             PIN ASSIGNMENT
//------------------------------------------
//Relay Block 1
int Relay1 = 22;//peltier 12V
int Relay2 = 23;//airpump-ch2
int Relay3 = 24;//airpump-ch3
int Relay4 = 25;//airpump-ch4
int Relay5 = 26;//airpump-ch5
int Relay6 = 27;//airpump-ch6
int Relay7 = 28;//airpump-ch7
int Relay8 = 29;//airpump-ch8
//Relay Block 2
int Relay9 = 30;//linearactuator 1-down
int Relay10 = 31;//linearactuator 1-up
int Relay11 = 32;//linearactuator 2-down
int Relay12 = 33;//linearactuator 2-up
//Slider Stepper motor (initialize slider tray stepper motor on pins 13, 20, 34)
bool SERVODIR = false;
const int stepPin = 13;
const int dirPin = 20;//direction, left=0, right=1
const int enPin = 34;//enable motor high=on
int count = 0;

//MINT Stepper motor (initialize MINT stepper motor on pins 12, 21, 35)
const int mintstepPin = 12;
const int mintdirPin = 21;//direction, left=0, right=1
const int mintenPin = 35;//enable motor high=on

//int ENA = 6;//for old analog motor code
//int IN1 = 5;//for old analog motor code
//int IN2 = 4;//for old analog motor code


//SUGAR Motor
int ENB = 3;
int IN3 = 2;
int IN4 = 1;


//---------------------------------------------
//           DEFINE VARIABLES
//---------------------------------------------
//Imports



//# of times it muddles
int SETMuddleTime = 5;
//# of times the lime cutter cycles
int SETLimeTime = 5;
//# of sec to dispense
int liquidMeasure1 = 3;
int liquidMeasure2 = 3;
int liquidMeasure3 = 3;
int liquidMeasure4 = 3;
int liquidMeasure5 = 3;
int liquidMeasure6 = 3;
int liquidMeasure7 = 3;
int liquidMeasure8 = 3;

//Pumps
int SETPump1Measure = liquidMeasure1;
int SETPump2Measure = liquidMeasure2;
int SETPump3Measure = liquidMeasure3;
int SETPump4Measure = liquidMeasure4;
int SETPump5Measure = liquidMeasure5;
int SETPump6Measure = liquidMeasure6;
int SETPump7Measure = liquidMeasure7;
int SETPump8Measure = liquidMeasure8;



//#include <Stepper.h>
//
//const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution
//const int rolePerMinute = 15;         // Adjustable range of 28BYJ-48 stepper is 0~17 rpm

// initialize the stepper library on pins 8 through 11:
//Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11);
//STEPPER MOTOR PINS





/*
  OneWire oneWire(ONE_WIRE_BUS);
  DallasTemperature sensors(&oneWire);
*/

const int timeout = 100;       //define timeout of 10 sec
int menuOption = 0;
long time0;
//------------------------------------------
//             ***PROGRAM SETUP***
//------------------------------------------
void setup() {

    Serial.begin(9600);
    while (!Serial) ; // wait for serial port to connect. Needed for native USB
    Serial.println("start");

  
  //  // put your setup code here, to run once:
  //myStepper.setSpeed(rolePerMinute);
  //// Setup Serial which is useful for debugging
  //    // Use the Serial Monitor to view printed messages
  //    Serial.begin(9600);
  //    while (!Serial) ; // wait for serial port to connect. Needed for native USB
  //    Serial.println("start");
  //    //sensors.begin();


  //SLIDER STEPPER MOTOR SETUP
  // Sets the two pins as Outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, LOW);

  //MINT STEPPER MOTOR SETUP
  // Sets the two pins as Outputs
  pinMode(mintstepPin, OUTPUT);
  pinMode(mintdirPin, OUTPUT);
  pinMode(mintenPin, OUTPUT);
  digitalWrite(mintenPin, LOW);

  //-----------------------------------------
  //              RELAY SETUP
  //-----------------------------------------

  //PELTIER
  pinMode(Relay1, OUTPUT);
  digitalWrite(Relay1, HIGH);

  //Pumps

  pinMode(Relay2, OUTPUT);
  digitalWrite(Relay2, HIGH);

  pinMode(Relay3, OUTPUT);
  digitalWrite(Relay3, HIGH);

  pinMode(Relay4, OUTPUT);
  digitalWrite(Relay4, HIGH);

  pinMode(Relay5, OUTPUT);
  digitalWrite(Relay5, HIGH);

  pinMode(Relay6, OUTPUT);
  digitalWrite(Relay6, HIGH);

  pinMode(Relay7, OUTPUT);
  digitalWrite(Relay7, HIGH);

  pinMode(Relay8, OUTPUT);
  digitalWrite(Relay8, HIGH);

  pinMode(Relay9, OUTPUT);
  digitalWrite(Relay9, HIGH);

  pinMode(Relay10, OUTPUT);
  digitalWrite(Relay10, HIGH);

  pinMode(Relay11, OUTPUT);
  digitalWrite(Relay11, HIGH);

  pinMode(Relay12, OUTPUT);
  digitalWrite(Relay12, HIGH);


//  //AUGER MOTORS
//
  pinMode(ENB, OUTPUT);
//  pinMode(IN1, OUTPUT);
//  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
//  pinMode(ENA, OUTPUT);
//  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
//









  menuOption = menu();



attachInterrupt(digitalPinToInterrupt(2), isrCount, RISING);





}

//-----------------------------------------
//              ***START LOOP***
//-----------------------------------------
void loop() {






  /*
    //TEMPERATURE COOLER
    sensors.requestTemperatures();
    Serial.print(sensors.getTempCByIndex(0));
    if (sensors.getTempCByIndex(0) < lowtemp){
      digitalWrite(relay1, LOW);
    if (sensors.getTempCByIndex(0) > hightemp){
      digitalWrite(relay1, HIGH);
  */



  if (menuOption == 1) {
    LimeCut();
  }
  if (menuOption == 2) {
    LimeCut();
  }

  if (menuOption == 3) {
    Muddle();
  }
  if (menuOption == 4) {
    Muddle();
  }
  
  if (menuOption == 5) {
    FiveRev();
  }
  if (menuOption == 6) {
    TenRev();
  }

  if (menuOption == 7) {
    TwentyRev();
  }

  if (menuOption == 8) {
    revmotor();
  }

  if (menuOption == 9) {
    Sugar();
  }

  if (menuOption == 10) {
    Mint();
  }
  
  if (menuOption == 11) {
    Pump1(SETPump1Measure);
  }
  if (menuOption == 12) {
    Pump2(SETPump2Measure);
  }
  if (menuOption == 13) {
    Pump3(SETPump3Measure);
  }
  if (menuOption == 14) {
    Pump4(SETPump4Measure);
  }
  if (menuOption == 15) {
    Pump5(SETPump5Measure);
  }
  if (menuOption == 16) {
    Pump6(SETPump6Measure);
  }
  if (menuOption == 17) {
    Pump7(SETPump7Measure);
  }

  if (menuOption == 18) {
    Peltier();
  }


  //if(menuOption == '7'){SlideRight();}
  //if(menuOption == '6'){SlideLeft();}

  // put your main code here, to run repeatedly:






  if (millis() - time0 > timeout)
  {
    menuOption = menu();
  }




}

//-------------------------------------
//          ***END LOOP***
//-------------------------------------

//-------------------------------------
//          FUNCTIONS
//-------------------------------------
//Peltier Function

void Peltier()
{
  digitalWrite(Relay1, LOW);
  delay(1500000);
  digitalWrite(Relay1, HIGH);
}



//MUDDLE FUNCTION
void Muddle() {

  digitalWrite(Relay10, LOW);
  digitalWrite(Relay9, HIGH);
  delay(5000);

  for (int MuddleCycle = 1; MuddleCycle < SETMuddleTime; MuddleCycle++) {
    //Down
    digitalWrite(Relay10, LOW);
    digitalWrite(Relay9, HIGH);
    delay(2000);
    //Up
    digitalWrite(Relay9, LOW);
    digitalWrite(Relay10, HIGH);
    delay(2000);
  }
  digitalWrite(Relay9, LOW);
  digitalWrite(Relay10, HIGH);
  delay(5000);
  //Turn Off
  digitalWrite(Relay9, HIGH);
  digitalWrite(Relay10, HIGH);
}

//LIME CUT FUNCTION
void LimeCut() {

  digitalWrite(Relay12, LOW);
  digitalWrite(Relay11, HIGH);
  delay(5000);

  for (int LimeCycle = 1; LimeCycle < SETLimeTime; LimeCycle++) {
    //Down
    digitalWrite(Relay12, LOW);
    digitalWrite(Relay11, HIGH);
    delay(2000);
    //Up
    digitalWrite(Relay11, LOW);
    digitalWrite(Relay12, HIGH);
    delay(2000);
  }
  digitalWrite(Relay11, LOW);
  digitalWrite(Relay12, HIGH);
  delay(5000);
  //Turn Off
  digitalWrite(Relay11, HIGH);
  digitalWrite(Relay12, HIGH);


}

//PUMP 1 FUNCTION

void Pump1(int Pump1Measure)
{
  int Time = Pump1Measure * 1000;
  digitalWrite(Relay2, LOW);
  delay(Time);
  digitalWrite(Relay2, HIGH);
}

//PUMP 2 Function
void Pump2(int Pump2Measure)
{
  int Time = Pump2Measure * 1000;
  digitalWrite(Relay3, LOW);
  delay(Time);
  digitalWrite(Relay3, HIGH);
}

//PUMP 3 FUNCTION

void Pump3(int Pump3Measure)
{
  int Time = Pump3Measure * 1000;
  digitalWrite(Relay4, LOW);
  delay(Time);
  digitalWrite(Relay4, HIGH);
}

//PUMP 4 FUNCTION

void Pump4(int Pump4Measure)
{
  int Time = Pump4Measure * 1000;
  digitalWrite(Relay5, LOW);
  delay(Time);
  digitalWrite(Relay5, HIGH);
}

//PUMP 5 FUNCTION

void Pump5(int Pump5Measure)
{
  int Time = Pump5Measure * 1000;
  digitalWrite(Relay6, LOW);
  delay(Time);
  digitalWrite(Relay6, HIGH);
}

//PUMP 6 FUNCTION
void Pump6(int Pump6Measure)
{
  int Time = Pump6Measure * 1000;
  digitalWrite(Relay7, LOW);
  delay(Time);
  digitalWrite(Relay7, HIGH);
}


//PUMP 7 FUNCTION
void Pump7(int Pump7Measure)
{
  int Time = Pump7Measure * 1000;
  digitalWrite(Relay8, LOW);
  delay(Time);
  digitalWrite(Relay8, HIGH);
}


//SLIDER STEPPER DIRRECTION
void revmotor (){

  SERVODIR = !SERVODIR;
  
}

void isrCount()
{
  count++;
  
}

void FiveRev() {
digitalWrite(dirPin,SERVODIR); // Enables the motor to move in a particular direction
analogWrite(stepPin,127); 


do {
  //Serial.println(count);
    delay(1);
  if(count>=2000){
  Serial.println("CHECK");
  analogWrite(stepPin,0); 
  digitalWrite(enPin, HIGH);
  break;
  }
} while (count<=2000);

 count = 0;


  }




void TenRev() {
   
digitalWrite(dirPin,SERVODIR); // Enables the motor to move in a particular direction
analogWrite(stepPin,127); 


do {
  //Serial.println(count);
  delay(1);
  if(count>=4000){
  Serial.println("CHECK");
  analogWrite(stepPin,0); 
  digitalWrite(enPin, HIGH);
  break;
  }
} while (count<=4000);

 count = 0;



  
}

void TwentyRev() {
  
digitalWrite(dirPin,SERVODIR); // Enables the motor to move in a particular direction
analogWrite(stepPin,127); 


    
do {
  //Serial.println(count);
    delay(1);
  if(count>=8000){
  Serial.println("CHECK");
  analogWrite(stepPin,0); 
  digitalWrite(enPin, HIGH);
  break;
  }
} while (count<=8000);

 count = 0;


}

//void SlideRight() {
//  // step one revolution  in one direction:
//  Serial.println("clockwise");
//  myStepper.step(stepsPerRevolution);
//  delay(500);
//  myStepper.step(0);
//}
//
//void SlideLeft() {
//  // step one revolution in the other direction:
//  Serial.println("counterclockwise");
//  myStepper.step(-stepsPerRevolution);
//  delay(500);
//  myStepper.step(0);
//}
//
//void SlideLeftandRight() {
//  // step one revolution in the other direction:
//  Serial.println("counterclockwise");
//
//  myStepper.setSpeed(1);
//  myStepper.step(-stepsPerRevolution / 8);
//
//  myStepper.setSpeed(5);
//  myStepper.step(-stepsPerRevolution / 4);
//
//  myStepper.setSpeed(8);
//  myStepper.step(-stepsPerRevolution / 2);
//
//  myStepper.setSpeed(10);
//  myStepper.step(-stepsPerRevolution / 1);
//
//  myStepper.step(0);
//
//  Serial.println("clockwise");
//
//  myStepper.setSpeed(1);
//  myStepper.step(stepsPerRevolution / 8);
//
//  myStepper.setSpeed(5);
//  myStepper.step(stepsPerRevolution / 4);
//
//  myStepper.setSpeed(8);
//  myStepper.step(stepsPerRevolution / 2);
//
//  myStepper.setSpeed(10);
//  myStepper.step(stepsPerRevolution / 1);
//
//  myStepper.step(0);
//}



//MINT STEPPER MOTOR

void MINTisrCount()
{
  count++;
  
}


void Mint() {//10 rev
   
digitalWrite(mintdirPin,false); // Enables the motor to move in a particular direction
analogWrite(mintstepPin,127); 


do {
  //Serial.println(count);
  delay(1);
  if(count>=4000){
  Serial.println("CHECK");
  analogWrite(stepPin,0); 
  digitalWrite(enPin, HIGH);
  break;
  }
} while (count<=4000);

 count = 0;

}



void Sugar() {
  //Turn on motor b

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 200);
  delay(2000);
  analogWrite(ENB, 0);

}


int menu()
{

  Serial.println(F("\nWhich component would you like to test?"));
  Serial.println(F("(0-15)"));

  Serial.println(F("(menu) send anything else or press on board reset button\n"));
  while (!Serial.available());

  // Read data from serial monitor if received
  while (Serial.available())
  {
    //char c = Serial.read();

    char buffer[2];
    if (Serial.available() >= 2) {
      for (int i = 0; i < 2; i++) {
        buffer[i] = Serial.read();
      }


      if (isAlphaNumeric(buffer[0]) && isAlphaNumeric(buffer[1]))
      {
        if (buffer[0] == '0' && buffer[1] == '0')
        {
          Serial.println(F("Now Testing 0"));
          return 1;
        }
        else if (buffer[0] == '0' && buffer[1] == '1')
        {
          Serial.println(F("Now Testing 1"));
          return 2;
        }
        else if (buffer[0] == '0' && buffer[1] == '2')
        {
          Serial.println(F("Now Testing 2"));
          return 3;
        }
        else if (buffer[0] == '0' && buffer[1] == '3')
        {
          Serial.println(F("Now Testing 3"));
          return 4;
        }
        else if (buffer[0] == '0' && buffer[1] == '4')
        {
          Serial.println(F("Now Testing 4"));
          return 5;
        }
        else if (buffer[0] == '0' && buffer[1] == '5')
        {
          Serial.println(F("Now Testing 5"));
          return 6;
        }
        else if (buffer[0] == '0' && buffer[1] == '6')
        {
          Serial.println(F("Now Testing 6"));
          return 7;
        }
        else if (buffer[0] == '0' && buffer[1] == '7')
        {
          Serial.println(F("Now Testing 7"));
          return 8;
        }
        else if (buffer[0] == '0' && buffer[1] == '8')
        {
          Serial.println(F("Now Testing 8"));
          return 9;
        }
        else if (buffer[0] == '0' && buffer[1] == '9')
        {
          Serial.println(F("Now Testing 9"));
          return 10;
        }
        else if (buffer[0] == '1' && buffer[1] == '0')
        {
          Serial.println(F("Now Testing 10"));
          return 11;
        }
        else if (buffer[0] == '1' && buffer[1] == '1')
        {
          Serial.println(F("Now Testing 11"));
          return 12;
        }
        else if (buffer[0] == '1' && buffer[1] == '2')
        {
          Serial.println(F("Now Testing 12"));
          return 13;
        }
        else if (buffer[0] == '1' && buffer[1] == '3')
        {
          Serial.println(F("Now Testing 13"));
          return 14;
        }
        else if (buffer[0] == '1' && buffer[1] == '4')
        {
          Serial.println(F("Now Testing 14"));
          return 15;
        }
        else if (buffer[0] == '1' && buffer[1] == '5')
        {
          Serial.println(F("Now Testing 15"));
          return 16;
        }
        else if (buffer[0] == '1' && buffer[1] == '6')
        {
          Serial.println(F("Now Testing 16"));
          return 17;
        }
        else if (buffer[0] == '1' && buffer[1] == '7')
        {
          Serial.println(F("Now Testing 17"));
          return 18;
        }
        else if (buffer[0] == '1' && buffer[1] == '8')
        {
          Serial.println(F("Now Testing 18"));
          return 19;
        }
        else if (buffer[0] == '1' && buffer[1] == '9')
        {
          Serial.println(F("Now Testing 19"));
          return 20;
        }
        else
        {
          Serial.println(F("illegal input!"));
          return 0;
        }
        time0 = millis();

      }
    }
  }
}
