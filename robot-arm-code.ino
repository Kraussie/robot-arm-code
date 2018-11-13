//LIBRARIES
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//DRIVER SETTINGS
#define MIN_PULSE_WIDTH       650
#define MAX_PULSE_WIDTH       2350
#define FREQUENCY             50
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//READ INPUTS
int joyX = A2;
int joyY = A3;
int joyBut = 2;
int butUp = 3;
int butDown = 4;

//MOTOR OUTPUTS
int motorGrab = 3;
int motorArm = 2;
int motorTilt = 1;
int motorBase = 0;

//GLOBAL VARIABLES
int tiltAngle = 90;

void setup() {
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);

  pinMode(joyX, INPUT);
  pinMode(joyY, INPUT);
  pinMode(joyBut, INPUT);
  pinMode(butUp, INPUT);
  pinMode(butDown, INPUT);

  Serial.begin(9600);
  Serial.print("this works");
}

void baseMove() {
  int pulse_wide, pulse_width, joyXVal;

  //read joystick-x
  joyXVal = analogRead(joyX);

  //Pulse width conversion
  pulse_wide = map(joyXVal, 0, 1020, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  
  //Control Base Motor
  pwm.setPWM(motorBase, 0, pulse_width);
}

void armMove() {
  int pulse_wide, pulse_width, joyYVal;

  //read joystick-x
  joyYVal = analogRead(joyY);

  //Pulse width conversion
  pulse_wide = map(joyYVal, 0, 1020, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  
  //Control Arm Motor
  pwm.setPWM(motorArm, 0, pulse_width);
}

void grab() {
  int pulse_wide, pulse_width, buttonState;

  //read joystick-x
  buttonState = digitalRead(joyBut);

  //Pulse width conversion
  pulse_wide = map(buttonState, 1, 0, 500, 1500);
  pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  
  //Control Grabber Motor
  pwm.setPWM(motorGrab, 0, pulse_width);
}

void grabTilt() {
  int pulse_wide, pulse_width;
  
  if (digitalRead(butUp) == LOW) {
    tiltAngle = tiltAngle + 5;
  } 
  if (digitalRead(butDown) == LOW) {
    tiltAngle = tiltAngle - 5;
  } 

  //Pulse width conversion
  pulse_wide = map(tiltAngle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  
  //Control Grabber Tilt Motor
  pwm.setPWM(motorTilt, 0, pulse_width);

  //print tilt angle var
  Serial.println(tiltAngle);
}

void loop() {

  //Control Base Motor
  baseMove();

  //Control Arm Tilt Motor
  armMove();

  //Control Grabber Motor
  grab();

  //Control Grabber Tilt Motor
  grabTilt();

  delay(100);
}
