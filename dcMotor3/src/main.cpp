#include <Arduino.h>
#include <Encoder.h>
#define button25 10
#define button50 11
#define button100 12
#define buttonStop 13
// Define constants
const int Enable = 9; // L298N ENA
const int motorPin1 = 7;   // L298N IN1
const int motorPin2 = 8;   // L298N IN2
const int encoderPinA = 3; // Encoder Channel A
const int encoderPinB = 2; // Encoder Channel B

Encoder myEnc(encoderPinA, encoderPinB);

long previousMillis = 0;
long currentMillis = 0;

volatile long currentEnc = 0;
volatile long previousEnc = 0;
volatile long prevPos = 0;
volatile long currentPos = 0;

int rot = 0;
float oldRotSpeed = 0;

 int motorSpeed = 0; // Adjust this value for the desired speed (0-255)


void setup() {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(button25, INPUT);
  pinMode(button50, INPUT);
  pinMode(button100, INPUT);
  pinMode(buttonStop, INPUT); 
  Serial.begin(9600);
}

float updateEncoder(void){
  currentEnc = myEnc.read();
  const int readPerRev =3393; //encoder read per revolution

  float rotSpeed;
  const int interval = 1000;
  currentMillis = millis();

  if(currentMillis - previousMillis >= interval){
    previousMillis = currentMillis;
   
    rotSpeed= (float)((currentEnc - previousEnc)*60/(readPerRev));
     previousEnc = currentEnc;
    return rotSpeed;
  }
}
void changeSpeed(){
  

  if (digitalRead(button25) == HIGH)
  {
    motorSpeed = 64;
    Serial.println("25%");
    delay(1000);
  }
  else if (digitalRead(button50) == HIGH)
  {
    motorSpeed = 128;
    Serial.println("50%");
    delay (1000);
  }
 
  else if (digitalRead(button100) == HIGH)
  {
    motorSpeed = 255;
    Serial.println("100%");
    delay(1000);
  }
  else if (digitalRead(buttonStop) == HIGH)
  {
    motorSpeed = 0;
    Serial.println("Stop");
    delay(1000);
  }

}

void loop() {
  // Run the motor clockwise
  
  analogWrite(Enable, motorSpeed);
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  float newRotSpeed;
  newRotSpeed = updateEncoder();
  changeSpeed();
  if(newRotSpeed != oldRotSpeed){
    Serial.print("Rotational Speed: ");
    Serial.print(newRotSpeed);
    Serial.println(" RPM");
    oldRotSpeed = newRotSpeed;
  }
  
  // Serial.print("encoder:");
  // Serial.println(currentEnc);

}

