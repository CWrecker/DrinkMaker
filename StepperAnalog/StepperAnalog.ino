bool SERVODIR = false;
const int stepPin = 6;
const int dirPin = 4;
const int enPin = 7;

int count = 0;


void setup() {
  // put your setup code here, to run once:

    Serial.begin(115200);
    while (!Serial) ; // wait for serial port to connect. Needed for native USB
    Serial.println("start");


  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, LOW);




  
//
digitalWrite(dirPin,SERVODIR); // Enables the motor to move in a particular direction
//  // Makes 400 pulses for making one full cycle rotation
analogWrite(stepPin,127); 


attachInterrupt(digitalPinToInterrupt(2), isrCount, RISING);



}

void loop() {
  // put your main code here, to run repeatedly:


//digitalWrite(dirPin,SERVODIR); // Enables the motor to move in a particular direction
//  // Makes 400 pulses for making one full cycle rotation
//





//for (int x = 0; x < 800; x++){
//digitalWrite(stepPin,HIGH);
//delay(2);
//digitalWrite(stepPin,LOW);  
//delay(2);
//}

//for( int x = 0; x <4000; x++){
//digitalWrite(stepPin,LOW);
//digitalWrite(stepPin,HIGH);
//delay(1);
//}

 
 if (count>=8000){
  analogWrite(stepPin,0); 
//  digitalWrite(stepPin, HIGH);
  digitalWrite(enPin, HIGH);
  }






}





void isrCount()
{
  count++;
  Serial.println(count);
}
