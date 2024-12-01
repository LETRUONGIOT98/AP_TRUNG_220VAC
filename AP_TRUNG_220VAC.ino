#include <PID_v1.h>
#define PIN_OUTPUT D7

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=125, Ki=0.5, Kd=0.1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
#include "DHT.h"
#define DHTPIN D5
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
/*struct Button {
  const uint8_t PIN;
  uint32_t numberKeyPresses;
  bool pressed;
};

Button button1 = {12, 0, false};//chân d6*/
#define quat D1
#define suong D2
int nguongam= 60;
void setup(){ 
  Serial.begin(9600);
  pinMode(D6, INPUT_PULLUP);
  pinMode(quat, OUTPUT);
  pinMode(suong,OUTPUT);
  pinMode(PIN_OUTPUT, OUTPUT);
  dht.begin();
 //attachInterrupt(digitalPinToInterrupt(D6), chay, CHANGE);//CHANGE
  Setpoint = 40;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(0, 255);
  //attachInterrupt(button1.PIN, chay, CHANGE);
}

void loop()
{  
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if(t >= Setpoint+3){
    digitalWrite(quat,LOW);
  }
 if(t >= Setpoint-3){
  digitalWrite(quat,HIGH);
 }

 if(t >= nguongam+3){
    digitalWrite(suong,LOW);
  }
 if(h >= nguongam-3){
  digitalWrite(suong,HIGH);
 }
  if(!isnan(t)){
 Input = t;
 }
  myPID.Compute();
  int Pwm = map(Output, 0,255,255,0);
  Serial.print("T: ");
  Serial.println(t);
  Serial.print("PWM: ");
  Serial.println(Pwm);
  analogWrite(PIN_OUTPUT,Pwm);

}
