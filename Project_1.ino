#include <Servo.h>
#include <PID_v1.h>
#define enA 9
#define in1 2
#define in2 3
#define enB 10
#define in3 4
#define in4 7



double Setpoint, Input, Output,Outputtemp;
double Kp = 50, Ki = 0.01, Kd = 0.0001;
const int potpin = A0;
double theta, rawpot;
int motoroutput;

PID PID_control(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
pinMode(potpin,INPUT);
pinMode(enA, OUTPUT);
pinMode(enB, OUTPUT);
pinMode(in1, OUTPUT);
pinMode(in2, OUTPUT);
pinMode(in3, OUTPUT);
pinMode(in4, OUTPUT);




PID_control.SetMode(AUTOMATIC);
PID_control.SetTunings(Kp, Ki, Kd);
Serial.begin(9600);


}

void loop() {

rawpot = analogRead(potpin);
theta = map(rawpot,0,1023,0,270);

if(theta<=125){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  Input = theta;
  Setpoint = 125;
  PID_control.Compute();
  motoroutput=map(Output,0,270,0,255);
  Serial.println(Input);
  if(motoroutput<50){
    motoroutput=50;
  }
  analogWrite(enA,motoroutput);
  analogWrite(enB,motoroutput);


}
if(theta>125){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  Input = abs((theta-115)-115);
  PID_control.Compute();
  motoroutput=map(Output,0,270,0,255);
  Serial.println(Input);
  if(motoroutput<50){
    motoroutput=50;
  }
analogWrite(enA,motoroutput);
analogWrite(enB,motoroutput);


}

}
