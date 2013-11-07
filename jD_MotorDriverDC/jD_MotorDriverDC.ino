/* 
 Motor driver for antenna tracker using L293D H-Bridge
 version 0.1 22/10/2013
 Marko Suikkanen
 */


//include libraries
#include <SoftwareSerial.h>


//define software serial and motor control pins
SoftwareSerial mySerial(6,5); // RX, TX

#define M1A 7
#define M1B 8
#define M1P 9

#define M2A 12
#define M2B 11
#define M2P 10

#define M1maxspeed 255
#define M1speed2 175
#define M1speed1 120
#define M2maxspeed 180
#define M2speed2 160
#define M2speed1 120

int M1speed=0;
int M2speed=0;
byte M1status = 0; //1 for cw, 2 for ccw, 0 for stop
byte M2status = 0; //1 for cw, 2 for ccw, 0 for stop

long prevM1millis=0, curM1millis=0;
long prevM2millis=0, curM2millis=0;
#define Mot1delay 2
#define Mot2delay 5

char inbyte;


void setup () {
  Serial.begin(57600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  // set the data rate for the SoftwareSerial port
  mySerial.begin(57600);

  //set outputs
  pinMode(M1A, OUTPUT);  // IN1
  pinMode(M1B, OUTPUT);  // IN2
  pinMode(M1P, OUTPUT);  // PWM --> L293D Enable 1-2
  pinMode(M2A, OUTPUT);  // IN4
  pinMode(M2B, OUTPUT);  // IN3
  pinMode(M2P, OUTPUT);  // PWM --> L293D Enable 3-4

}

void loop() {


  if (mySerial.available()) {
    inbyte = mySerial.read();

    if(inbyte == 'A') M1status = 1; // Full speed up
    if(inbyte == 'B') M1status= 2;  // 2/3 speed up
    if(inbyte == 'C') M1status = 3; // 1/3 speed up
    if(inbyte == 'D') M1status = 4; // Full speed down
    if(inbyte == 'E') M1status = 5; // 2/3 speed down
    if(inbyte == 'F') M1status= 6;  // 1/3 speed down
    if(inbyte == 'G') M1status = 7; // stop

    if(inbyte == 'H') M2status = 1;
    if(inbyte == 'I') M2status = 2;
    if(inbyte == 'J') M2status = 3;
    if(inbyte == 'K') M2status = 4;
    if(inbyte == 'L') M2status = 5;
    if(inbyte == 'M') M2status = 6;
    if(inbyte == 'N') M2status = 7;




    Serial.println(inbyte);
  }
  curM1millis = millis(); 
  if((curM1millis - prevM1millis) > Mot1delay) {
    prevM1millis = curM1millis; 
    switch(M1status){
    case 1:

      digitalWrite(M1A, HIGH); 
      digitalWrite(M1B, LOW);
      if(M1speed <M1maxspeed) analogWrite(M1P, M1speed++);
      else analogWrite(M1P,M1maxspeed);
      break;

    case 2:

      digitalWrite(M1A, HIGH); 
      digitalWrite(M1B, LOW);       
      if(M1speed <M1speed2) analogWrite(M1P, M1speed++);
      else analogWrite(M1P,M1speed2);

      break;

    case 3:
      digitalWrite(M1A, HIGH); 
      digitalWrite(M1B, LOW); 
      if(M1speed <M1speed1) analogWrite(M1P, M1speed++);
      else analogWrite(M1P,M1speed1);
      break;

    case 4:

      digitalWrite(M1A, LOW); 
      digitalWrite(M1B, HIGH); 
      if(M1speed <M1maxspeed) analogWrite(M1P, M1speed++);
      else analogWrite(M1P,M1maxspeed);
      break;

    case 5:

      digitalWrite(M1A, LOW); 
      digitalWrite(M1B, HIGH); 
      if(M1speed <M1speed2) analogWrite(M1P, M1speed++);
      else analogWrite(M1P,M1speed2);

      break;

    case 6:
      digitalWrite(M1A, LOW); 
      digitalWrite(M1B, HIGH); 
      if(M1speed <M1speed1) analogWrite(M1P, M1speed++);
      else analogWrite(M1P,M1speed1);
      break;
    case 7:
      digitalWrite(M1A, LOW); 
      digitalWrite(M1B, LOW); 
      analogWrite(M1P, 0);
      M1speed=0;
      break;
    }
  }
  curM2millis = millis(); 
  if((curM2millis - prevM2millis) > Mot2delay) {
    prevM2millis = curM2millis;

    switch(M2status){
    case 1:

      digitalWrite(M2A, HIGH); 
      digitalWrite(M2B, LOW); 
      if(M2speed <M2maxspeed) analogWrite(M2P, M2speed++);   
      else  analogWrite(M2P,M2maxspeed);
      break;

    case 2:
      digitalWrite(M2A, HIGH); 
      digitalWrite(M2B, LOW); 
      if(M2speed <M2speed2) analogWrite(M2P, M2speed++);
      else analogWrite(M2P,M1speed2);

      break;

    case 3:
      digitalWrite(M2A, HIGH); 
      digitalWrite(M2B, LOW); 
      if(M2speed <M2speed1) analogWrite(M2P, M2speed++);
      else analogWrite(M2P,M1speed1);

      break;

    case 4:
      digitalWrite(M2A, LOW); 
      digitalWrite(M2B, HIGH); 
      if(M2speed <M2maxspeed) analogWrite(M2P, M2speed++);
      else analogWrite(M2P,M2maxspeed);

      break;

    case 5:
      digitalWrite(M2A, LOW); 
      digitalWrite(M2B, HIGH); 
      if(M2speed <M2speed2) analogWrite(M2P, M2speed++);
      else analogWrite(M2P,M1speed2);

      break;

    case 6:

      digitalWrite(M2A, LOW); 
      digitalWrite(M2B, HIGH); 
      if(M2speed <M2speed1)  analogWrite(M2P, M2speed++);
      else analogWrite(M2P,M1speed1);

    case 7:
      digitalWrite(M2A, LOW); 
      digitalWrite(M2B, LOW); 
      analogWrite(M2P, 0);
      M2speed=0;
      break;
    }
  }
}





























