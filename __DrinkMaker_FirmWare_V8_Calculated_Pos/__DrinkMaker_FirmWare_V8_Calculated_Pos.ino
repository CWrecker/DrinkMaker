//#include "Arduino.h"
//#include "Servo.h"
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
bool SERVODIR = true;
const int stepPin = 44;
const int dirPin = 20;//direction, left=0, right=1
const int enPin = 34;//enable motor high=on, low=off
const int SlideINT = 2;//Set interupt input pin to 2
long count = 0;
long slidePos = 0;
long LastPos;

//MINT Stepper motor (initialize MINT stepper motor on pins 12, 21, 35)
const int MINTstepPin = 12;
const int MINTdirPin = 21;//direction, left=0, right=1
const int MINTenPin = 35;//enable motor high=off, low=on
const int MintINT = 3;//Set interupt input pin to 3
int MINTcount = 0;

//int ENA = 6;//for old analog motor code
//int IN1 = 5;//for old analog motor code
//int IN2 = 4;//for old analog motor code


//SUGAR Motor
//int ENB = 3;
//int IN3 = 2;
// IN4 = 1;


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
 

  //SLIDER STEPPER MOTOR SETUP
 //sets pin 13 to 3921.16.50Hz
 TCCR5B=TCCR5B & B11111000|B00000010; //for PWM fequency of 3921.16 Hz on Pin 44,45,46 
  // Sets the two pins as Outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);
 // digitalWrite(enPin, HIGH);
  pinMode (SlideINT, INPUT_PULLUP);


  //MINT STEPPER MOTOR SETUP
  // Sets the two pins as Outputs
  pinMode(MINTstepPin, OUTPUT);
  pinMode(MINTdirPin, OUTPUT);
  pinMode(MINTenPin, OUTPUT);
  digitalWrite(MINTenPin, HIGH);
  pinMode (MintINT, INPUT_PULLUP);



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
//  pinMode(ENB, OUTPUT);
//  pinMode(IN1, OUTPUT);
//  pinMode(IN2, OUTPUT);
//  pinMode(IN3, OUTPUT);
//  pinMode(IN4, OUTPUT);
//  pinMode(ENA, OUTPUT);
//  analogWrite(ENA, 0);
//  analogWrite(ENB, 0);
//




  menuOption = menu();

  
  attachInterrupt(digitalPinToInterrupt(SlideINT), isrCount, RISING);//set Pin to interupt mode, count on a rising signal from slider stepper
  attachInterrupt(digitalPinToInterrupt(MintINT), isrMINTCount, RISING);//set Pin 3 to interupt mode, count on a rising signal from from MINT stepper 

  
}

//-----------------------------------------
//              ***START LOOP***
//-----------------------------------------
void loop() {

  // put your main code here, to run repeatedly:

 //Slider Position Tracker
 


  /*
    //TEMPERATURE COOLER
    sensors.requestTemperatures();
    Serial.print(sensors.getTempCByIndex(0));
    if (sensors.getTempCByIndex(0) < lowtemp){
      digitalWrite(relay1, LOW);
    if (sensors.getTempCByIndex(0) > hightemp){
      digitalWrite(relay1, HIGH);
  */



  if (menuOption == 0) {
    ;//null
  }
  
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
    MuddlePos();
  }
  if (menuOption == 6) {
    MintPos();
  }

  if (menuOption == 7) {
    LiquidPos();
  }

  if (menuOption == 8) {
    LimePos();
  }

  if (menuOption == 9) {
    HomePos();  
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
  delay(5000);//set for coolong time, need tigger and remove delay to let other things work
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


//SLIDER STEPPER DIRECTION
void revmotor (){
slidePos = 10000;
  SERVODIR = !SERVODIR; // Enables the motor to move in a particular direction
  
}

void isrCount()
{
  count++;
  
    if (SERVODIR == true)
      { ++slidePos;
       // Serial.println(slidePos);
      }
    else if (SERVODIR == false)
        {
          --slidePos;
          //Serial.println(slidePos);
        }
  
}


//SLIDER STEPPER DISTANCE SET
void MuddlePos() {

//if(slidePos <= 5000){
//  SERVODIR = false;
//}
//else{
//  SERVODIR = true;
//}
  
LastPos = slidePos;
Serial.println("Last Position Is: ");
Serial.println(LastPos);


while(slidePos <= 5000)

{
  SERVODIR = true;
  digitalWrite(dirPin,SERVODIR);
  analogWrite(stepPin,127); //50% duty cycle

    LastPos = slidePos;
//  Serial.println("My Position Is increasing: ");
//  Serial.println(slidePos);



if(slidePos>=5000){
  Serial.println("Went Right to Muddle Position");
    Serial.println(slidePos);
  analogWrite(stepPin,0); 
  digitalWrite(enPin, HIGH);
  
  return;
  }

  
 
}


  
while(slidePos >= 5000)

{
  SERVODIR = false;
  digitalWrite(dirPin,SERVODIR);
  analogWrite(stepPin,127); //50% duty cycle

    LastPos = slidePos;
//  Serial.println("My Position Is decreasing: ");
//  Serial.println(slidePos);

  
  if(slidePos<=5000){
  Serial.println("Went Left to Muddle Position");
    Serial.println(slidePos);
  analogWrite(stepPin,0); 
  digitalWrite(enPin, HIGH);
  
  return;
  }
  
}


///////////



 


  }




void MintPos() {

//if(slidePos <= 47500){
//  SERVODIR = false;
//}
//else{
//  SERVODIR = true;
//}

LastPos = slidePos;
Serial.println("Last Position Is: ");
Serial.println(LastPos);

while(slidePos <= 47500)

{
  SERVODIR = true;
  digitalWrite(dirPin,SERVODIR);
  analogWrite(stepPin,127); //50% duty cycle

LastPos = slidePos;
//  Serial.println("My Position Is increasing: ");
//  Serial.println(slidePos);
//  slidePos = slidePos + 100;


if(slidePos>=47500){
  Serial.println("Went Right to Mint Position");
    Serial.println(slidePos);
  analogWrite(stepPin,0); 
  digitalWrite(enPin, HIGH);
  
  return;
  }

  
 
}


  
while(slidePos >= 47500)

{
  SERVODIR = false;
  digitalWrite(dirPin,SERVODIR);
  analogWrite(stepPin,127); //50% duty cycle
  LastPos = slidePos;

//  Serial.println("My Position Is decreasing: ");
//  Serial.println(slidePos);
//  slidePos = slidePos - 100;
  
  if(slidePos<=47500){
  Serial.println("Went Left to Mint Position");
    Serial.println(slidePos);
  analogWrite(stepPin,0); 
  digitalWrite(enPin, HIGH);
  
  return;
  }
  
}






















  
}

void LiquidPos() {
//if(slidePos <= 110000){
//  SERVODIR = false;
//}
//else{
//  SERVODIR = true;
//}

LastPos = slidePos;
Serial.println("Last Position Is: ");
Serial.println(LastPos);


while(slidePos <= 110000)

{
  SERVODIR = true;
  digitalWrite(dirPin,SERVODIR);
  analogWrite(stepPin,127); //50% duty cycle
  
//  Serial.println("My Position Is increasing: ");
//  Serial.println(slidePos);
//  slidePos = slidePos + 100;


if(slidePos>=110000){
  Serial.println("Went Right to Liquid Position");
    Serial.println(slidePos);
  analogWrite(stepPin,0); 
  digitalWrite(enPin, HIGH);
  
  return;
  }

  
 
}


  
while(slidePos >= 110000)

{
  SERVODIR = false;
  digitalWrite(dirPin,SERVODIR);
  analogWrite(stepPin,127); //50% duty cycle
  
//  Serial.println("My Position Is decreasing: ");
//  Serial.println(slidePos);
//  slidePos = slidePos - 100;
  
  if(slidePos<=110000){
  Serial.println("Went Left to Liquid Position");
    Serial.println(slidePos);
  analogWrite(stepPin,0); 
  digitalWrite(enPin, HIGH);
  
  return;
  }
  
}


///////////


}

void LimePos(){ 

//if(slidePos <= 160000){
//  SERVODIR = false;
//}
//else{
//  SERVODIR = true;
//}


  LastPos = slidePos;
Serial.println("Last Position Is: ");
Serial.println(LastPos);


while(slidePos <= 160000)

{
  SERVODIR = true;
  digitalWrite(dirPin,SERVODIR);
  analogWrite(stepPin,127); //50% duty cycle
  
//  Serial.println("My Position Is increasing: ");
//  Serial.println(slidePos);
//  slidePos = slidePos + 100;


if(slidePos>=160000){
  Serial.println("Went Right to Lime Position");
    Serial.println(slidePos);
  analogWrite(stepPin,0); 
  digitalWrite(enPin, HIGH);
  
  return;
  }

  
 
}


  
while(slidePos >= 160000)

{
  SERVODIR = false;
  digitalWrite(dirPin,SERVODIR);
  analogWrite(stepPin,127); //50% duty cycle
  
//  Serial.println("My Position Is decreasing: ");
//  Serial.println(slidePos);
//  slidePos = slidePos - 100;
  
  if(slidePos<=160000){
  Serial.println("Went Left to Lime Position");
    Serial.println(slidePos);
  analogWrite(stepPin,0); 
  digitalWrite(enPin, HIGH);
  
  return;
  }
  
}


///////////


}


void HomePos(){ 

//if(slidePos <= 160000){
//  SERVODIR = false;
//}
//else{
//  SERVODIR = true;
//}


  LastPos = slidePos;
Serial.println("Last Position Is: ");
Serial.println(LastPos);





  
while(slidePos >= 5)

{
  SERVODIR = false;
  digitalWrite(dirPin,SERVODIR);
  analogWrite(stepPin,127); //50% duty cycle

  
  Serial.println("My Position Is decreasing: ");
  Serial.println(slidePos);
//  slidePos = slidePos - 100;
  
  if(slidePos<=5){
  Serial.println("Went Left to HOME Position");
    Serial.println(slidePos);
  analogWrite(stepPin,0); 
  digitalWrite(enPin, HIGH);
  
  return;
  }
  
}


///////////


}


//MINT STEPPER MOTOR

void isrMINTCount()
{
  MINTcount++; 
}

void Mint() {//10 rev
   
digitalWrite(MINTdirPin,true); // Enables the motor to move in a particular direction
analogWrite(MINTstepPin,127); //50% duty cycle

do {
  Serial.println(MINTcount);
  delay(1);
  if(MINTcount>=4000){
  Serial.println("CHECKmint");
  analogWrite(MINTstepPin,0); 
  digitalWrite(MINTenPin, LOW);
  break;
  }
} while (MINTcount<=4000);

 MINTcount = 0;

}



//void Sugar() {
  //Turn on motor b

 // digitalWrite(IN3, LOW);
//  digitalWrite(IN4, HIGH);
//  analogWrite(ENB, 200);
//  delay(2000);
//  analogWrite(ENB, 0);

//}


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
          Serial.println(F("Now Testing 0nothing"));
          return 0;
        }
        else if (buffer[0] == '0' && buffer[1] == '1')
        {
          Serial.println(F("Now Testing 1Lime Cutter"));
          return 1;
        }
        else if (buffer[0] == '0' && buffer[1] == '2')
        {
          Serial.println(F("Now Testing 2Lime Cutter2"));
          return 2;
        }
        else if (buffer[0] == '0' && buffer[1] == '3')
        {
          Serial.println(F("Now Testing 3Muddle"));
          return 3;
        }
        else if (buffer[0] == '0' && buffer[1] == '4')
        {
          Serial.println(F("Now Testing 4Muddle"));
          return 4;
        }
        else if (buffer[0] == '0' && buffer[1] == '5')
        {
          Serial.println(F("Now Testing 5Five Slider"));
          return 5;
        }
        else if (buffer[0] == '0' && buffer[1] == '6')
        {
          Serial.println(F("Now Testing 6Ten Rev Slider"));
          return 6;
        }
        else if (buffer[0] == '0' && buffer[1] == '7')
        {
          Serial.println(F("Liquid position"));
          return 7;
        }
        else if (buffer[0] == '0' && buffer[1] == '8')
        {
          Serial.println(F("Lime Position"));

          return 8;
        }
        else if (buffer[0] == '0' && buffer[1] == '9')
        {
          Serial.println(F("Rev Slider motor"));
          return 9;
        }
        else if (buffer[0] == '1' && buffer[1] == '0')
        {
          Serial.println(F("Now Testing 10Mint motor5 rev"));
          return 10;
        }
        else if (buffer[0] == '1' && buffer[1] == '1')
        {
          Serial.println(F("Now Testing 11Pump 1"));
          return 11;
        }
        else if (buffer[0] == '1' && buffer[1] == '2')
        {
          Serial.println(F("Now Testing 12Pump2"));
          return 12;
        }
        else if (buffer[0] == '1' && buffer[1] == '3')
        {
          Serial.println(F("Now Testing 13Pump3"));
          return 13;
        }
        else if (buffer[0] == '1' && buffer[1] == '4')
        {
          Serial.println(F("Now Testing 14Pump4"));
          return 14;
        }
        else if (buffer[0] == '1' && buffer[1] == '5')
        {
          Serial.println(F("Now Testing 15Pump5"));
          return 15;
        }
        else if (buffer[0] == '1' && buffer[1] == '6')
        {
          Serial.println(F("Now Testing 16Pump6"));
          return 16;
        }
        else if (buffer[0] == '1' && buffer[1] == '7')
        {
          Serial.println(F("Now Testing 17Pump7"));
          return 17;
        }
        else if (buffer[0] == '1' && buffer[1] == '8')
        {
          Serial.println(F("Now Testing 18Peltier"));
          return 18;
        }
        else if (buffer[0] == '1' && buffer[1] == '9')
        {
          Serial.println(F("Now Testing 19nothing"));
          return 19;
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
