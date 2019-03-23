/**
   Vesion 2.1 of softvare for our soccer robots DANCZI/MARCI.
   Robot can in this version:
   Move using PID
   Follow ball
   Detect a line
   Move dribler
   Detect a ball in dribler
   Kick goal
   Made by XLC team
*/
#include <Wire.h>
#include <PixyViSy.h>
#include <SPI.h>
#include <Pixy.h>
#include <XLCPixy.h>
#include <stdint.h>
#include <Motor.h>
#include <LiquidCrystal.h>

#define FOC_LEN_X 266
#define FOC_LEN_Y 237

#define GOAL_SIG 1
#define BALL_SIG 2

#define MIN_GOAL_AREA 100
//motors pins
#define A_DIR 8
#define A_PWM 9
#define B_DIR 11
#define B_PWM 10
#define C_DIR 5
#define C_PWM 4
#define D_DIR 2
#define D_PWM 3
#define DRIBLER_PWM 6
//light sensors pins
#define LINE_SENSORS_COUNT 6
#define LINE_FRONT_PIN A13
#define LINE_BACK_PIN A10
#define LINE_LEFT_PIN_1 A12
#define LINE_LEFT_PIN_2 A8
#define LINE_RIGHT_PIN_1 A11
#define LINE_RIGHT_PIN_2 A9

#define LINE_THRESH_PIN 12
#define LINE_TIME 200
#define STOP_LINE_TIME 40
#define MUTEX(state) mutex[0] = mutex[1] = mutex[2] = state
#define LINE_MAX_DIFF_TIME 20000
#define LINE_USE_INT 1
#define LINE_THRESH 150
int follow_compass = 0;
uint8_t line_use_int = LINE_USE_INT;
uint8_t light_pwm = LINE_THRESH;
uint8_t motion_last_dir;
uint8_t spd_line = 100;

uint32_t volatile ws_tmp[] = {0, 0, 0 , 0, 0, 0};
uint32_t ws[] = {0, 0, 0 , 0, 0, 0};
uint8_t mutex[] = {0, 0, 0 , 0, 0, 0};
uint8_t touch_line;
uint8_t last_motor_dir;
long start;
int out_line_time = 200;
int touch_line_dir;
int moving_tipe = 2; //1=ball,2=gool kicking
ISR(PCINT2_vect)
{
  if (!mutex[0] && !ws_tmp[0] && read_line_sensor(0)) {
    ws_tmp[0] = micros();
    motors_off();
  }

  if (!mutex[1] && !ws_tmp[1] && read_line_sensor(1)) {
    ws_tmp[1] = micros();
    motors_off();
  }

  if (!mutex[2] && !ws_tmp[2] && read_line_sensor(2)) {
    ws_tmp[2] = micros();
    motors_off();
  }

  if (!mutex[3] && !ws_tmp[3] && read_line_sensor(3)) {
    ws_tmp[3] = micros();
    motors_off();
  }

  if (!mutex[4] && !ws_tmp[4] && read_line_sensor(4)) {
    ws_tmp[4] = micros();
    motors_off();
  }

  if (!mutex[5] && !ws_tmp[5] && read_line_sensor(5)) {
    ws_tmp[5] = micros();
    motors_off();
  }
}

PixyViSy pixyViSy(FOC_LEN_X, FOC_LEN_Y, PIXYVISY_GOAL | PIXYVISY_BALL);
uint16_t goal_distance, ball_distance;
int8_t ball_angle;
int8_t last_ball_angle = 0;
long no_ball = 0;
long no_see_ball = 0;
uint16_t last_distance = 0;
uint16_t last_input = 0;
char action;

int slide = 1;

Motor motorA = Motor(A_DIR, A_PWM);
Motor motorB = Motor(B_DIR, B_PWM);
Motor motorC = Motor(C_DIR, C_PWM);
Motor motorD = Motor( D_DIR, D_PWM);

int PID_P = 0;
long PID_I = 0;
int PID_D = 0;
//////////////////////////////////////////////////////
#define P  3
#define I  0.008
#define D  13
//////////////////////////////////////////////////////
#define max_I 100
#define min_I -100
//////////////////////////////////////////////////////
int spd = 150;//absolute speed of robot
//////////////////////////////////////////////////////
int x = 0;
int input;
int setpoint = 0;
int out;
int error;
int frequency = 20;
long last_time = 0;
int last_error = 0;
int line_dir;
int compensation;
long long now = 0;
long long while_time;
int error_index = -1;
int errors[2] {0, 0};//delay for D parameter of PID
int PID(int16_t speeds[3], int setpoint, int feedback) {
  if ( input > 180 || input < -180) {
    input = last_input;
  }
  int16_t maximal_value = -1000;
  int16_t minimal_value = 1000;
  for (int16_t i = 0; i < 4; i++) {
    if (speeds[i] < minimal_value) {
      minimal_value = speeds[i];
    }
    if (maximal_value < speeds[i])
      maximal_value = speeds[i];
  }
  maximal_value = 255 - maximal_value;
  minimal_value = -255 - minimal_value;

  input = feedback;

  error = setpoint - input;

  error_index++;
  if (error_index == 2) {
    error_index = 0;
  }
  errors[error_index] = error;

  now = millis();
  if (error != 0) {
    PID_P = error * P;
    if (PID_I + ((now - last_time) * error ) * I  < min_I) {
      PID_I = min_I;
    }
    else if (PID_I + ((now - last_time) * error ) * I > max_I ) {
      PID_I = max_I;
    }
    else {
      PID_I += ((now - last_time) * error ) * I ;
    }
    PID_D = (errors[1 - error_index] - error) * D;

    out = PID_P + PID_I - PID_D;
    if (out > maximal_value) {
      out = maximal_value;
    }
    else if (out < minimal_value) {
      out =  minimal_value;
    }

  }
  else {
    PID_P = 0;
    PID_I = 0;
    PID_D = 0;
  }
  last_input = input;
  last_time = now;
  last_error = error;
  return -out;
}

void na_mieste(int vstup) {
  int16_t speeds[4] = {0, 0, 0, 0};
  compensation = PID(speeds, 0, vstup);
  motorA.go(speeds[0] + compensation);
  motorB.go(speeds[1] + compensation);
  motorC.go(speeds[2] + compensation);
  motorD.go(speeds[3] + compensation);
}
void vpred(int vstup) {
  last_motor_dir = 0;
  int16_t speeds[4] = { -spd, -spd, spd, spd};
  compensation = PID(speeds, 0, vstup);
  motorA.go(speeds[0] + compensation);
  motorB.go(speeds[1] + compensation);
  motorC.go(speeds[2] + compensation);
  motorD.go(speeds[3] + compensation);
}
void vzad(int vstup) {
  last_motor_dir = 4;
  int16_t speeds[4] = {spd, spd, -spd, -spd};
  compensation = PID(speeds, 0, vstup);
  motorA.go(speeds[0] + compensation);
  motorB.go(speeds[1] + compensation);
  motorC.go(speeds[2] + compensation);
  motorD.go(speeds[3] + compensation);
}
void vpravo(int vstup) {
  last_motor_dir = 2;
  int16_t speeds[4] = {spd, -spd, -spd, spd};
  compensation = PID(speeds, 0, vstup);
  motorA.go(speeds[0] + compensation);
  motorB.go(speeds[1] + compensation);
  motorC.go(speeds[2] + compensation);
  motorD.go(speeds[3] + compensation);
}
void vlavo(int vstup) {
  last_motor_dir = 6;
  int16_t speeds[4] = { -spd, spd, spd, -spd};
  compensation = PID(speeds, 0, vstup);
  motorA.go(speeds[0] + compensation);
  motorB.go(speeds[1] + compensation);
  motorC.go(speeds[2] + compensation);
  motorD.go(speeds[3] + compensation);
}
void vlavo_vpred(int vstup) {
  last_motor_dir = 7;
  int16_t speeds[4] = { -spd, 0, spd, 0};
  compensation = PID(speeds, 0, vstup);
  motorA.go(speeds[0] + compensation);
  motorB.go(speeds[1] + compensation);
  motorC.go(speeds[2] + compensation);
  motorD.go(speeds[3] + compensation);
}
void vpravo_vzad(int vstup) {
  last_motor_dir = 3;
  int16_t speeds[4] = {spd, 0, -spd, 0};
  compensation = PID(speeds, 0, vstup);
  motorA.go(speeds[0] + compensation);
  motorB.go(speeds[1] + compensation);
  motorC.go(speeds[2] + compensation);
  motorD.go(speeds[3] + compensation);
}
void vlavo_vzad(int vstup) {
  last_motor_dir = 5;
  int16_t speeds[4] = {0, -spd, 0, spd};
  compensation = PID(speeds, 0, vstup);
  motorA.go(speeds[0] + compensation);
  motorB.go(speeds[1] + compensation);
  motorC.go(speeds[2] + compensation);
  motorD.go(speeds[3] + compensation);
}
void vpravo_vpred(int vstup) {
  last_motor_dir = 1;
  int16_t speeds[4] = {0, spd, 0, -spd};
  compensation = PID(speeds, 0, vstup);
  motorA.go(speeds[0] + compensation);
  motorB.go(speeds[1] + compensation);
  motorC.go(speeds[2] + compensation);
  motorD.go(speeds[3] + compensation);
}
void motors_off() {
  motorA.go(0);
  motorB.go(0);
  motorC.go(0);
  motorD.go(0);
}
void out_line(int vstup) {
  last_motor_dir = 4;
  long long x = millis();
  while (millis() - x < 400) {
    int16_t speeds[4] = {spd_line, spd_line, -spd_line, -spd_line};
    motorA.go(speeds[0]);
    motorB.go(speeds[1]);
    motorC.go(speeds[2]);
    motorD.go(speeds[3]);
  }
  motors_off();
  delay(STOP_LINE_TIME);
}
int angle_ball() {
  pixyViSy.update();
  ball_angle = pixyViSy.getBallAngle();
  if (ball_angle == 0) {
    no_ball++;
    if (no_ball > 5) {
      ball_angle = 0;
    }
    else {
      ball_angle = last_ball_angle;
    }
  }
  else {
    no_ball = 0;
    last_ball_angle = ball_angle;
  }
  return ball_angle;
}
int distance_ball() {

  pixyViSy.update();
  ball_distance = pixyViSy.getBallDist();
  if (ball_distance == ~0) {
    no_see_ball++;
    if (no_see_ball < 3) {
      //ball_distance = 0;
       ball_distance = last_distance;
    }
    else{
       no_see_ball=4;
      }

  }
  else {
    no_see_ball = 0;
    last_distance = ball_distance;
  }
  return ball_distance;
}
void goal() {

  pixyViSy.update();
  action = pixyViSy.getGoalAction();
  if (compass() < 20 && compass() > -20) {
    switch (action) {
      case 'K': {
          // na_mieste(compass());
          //vpred(compass());
           motors_off();
          digitalWrite(31, HIGH);
          delay(100);
          digitalWrite(31, LOW);
           motors_off();
          delay(20);
          Serial.println("kick");
          break;
        }
      case 'L': {
          vlavo(compass());
          Serial.println("vlavo");
          break;
        }
      case 'R': {
          vpravo(compass());
          Serial.println("vpravo");
          break;
        }
      default: {
          Serial.println("ERROR!");
          vpred(0);
          break;
        }
    }
  }
  else {
    if (compass() > 0) {
      motorA.go(60);
      motorB.go(60);
      motorC.go(60);
      motorD.go(60);

    }
    else {
      motorA.go(-60);
      motorB.go(-60);
      motorC.go(-60);
      motorD.go(-60);
    }
  }
}
void ball() {
  if (distance_ball() == ~0) {
    analogWrite(DRIBLER_PWM, 0);
    motorA.go(70);
    motorB.go(70);
    motorC.go(70);       //no object
    motorD.go(70);
  }
  else {
    follow_compass = compass();
    vpred(angle_ball());
    moving_tipe = 1;
  }
}
void setup()
{
  Serial.begin(9600);
  Wire.begin();
  interrupts();
  pinMode(DRIBLER_PWM, OUTPUT);
  pinMode(29, INPUT);
  pinMode(31, OUTPUT);
  setup_line_sensors();
  pixyViSy.setGoalSig(GOAL_SIG);
  pixyViSy.setBallSig(BALL_SIG);
  pixyViSy.setMinGoalArea(MIN_GOAL_AREA);
  LCD_setup();
  start_menu();
}

void loop()
{
  /* analogWrite(DRIBLER_PWM, 255);
    goal();*/

  line_dir = line_sensors_dir();
  if (line_dir == 255) {
    if (!digitalRead(29)) {
      analogWrite(DRIBLER_PWM, 255);
      moving_tipe = 2;
      goal();
    }
    else {
      analogWrite(DRIBLER_PWM, 255);
      moving_tipe = 1;
      //spd = 70;
      ball();           //BALL
    }
  }
  else {
    if (line_dir != 255) {
      if (moving_tipe == 1) {
        out_line(compass() - follow_compass);
      }
      else {
        //moving_tipe==2

        switch (line_dir) {
          case 0:
            while_time = millis();
            while (millis() - while_time < LINE_TIME) {
              vzad(compass());
            }
            motors_off();
            delay(STOP_LINE_TIME);
            break;
          case 1:
            while_time = millis();
            while (millis() - while_time < LINE_TIME) {
              vlavo_vzad(compass());
            }
            motors_off();
            delay(STOP_LINE_TIME);
            break;
          case 2:
            while_time = millis();
            while (millis() - while_time < LINE_TIME) {
              vlavo(compass());
            }
            motors_off();
            delay(STOP_LINE_TIME);
            break;
          case 3:
            while_time = millis();
            while (millis() - while_time < LINE_TIME) {
              vlavo_vpred(compass());
            }
            motors_off();
            delay(STOP_LINE_TIME);
            break;
          case 4:
            while_time = millis();
            while (millis() - while_time < LINE_TIME) {
              vpred(compass());
            }
            motors_off();
            delay(STOP_LINE_TIME);
            break;
          case 5:
            while_time = millis();
            while (millis() - while_time < LINE_TIME) {
              vpravo_vpred(compass());
            }
            motors_off();
            delay(STOP_LINE_TIME);
            break;
          case 6:
            while_time = millis();
            while (millis() - while_time < LINE_TIME) {
              vpravo(compass());
            }
            motors_off();
            delay(STOP_LINE_TIME);
            break;
          case 7:
            while_time = millis();
            while (millis() - while_time < LINE_TIME) {
              vpravo_vzad(compass());
            }
            motors_off();
            delay(STOP_LINE_TIME);
            break;
          case 255:
            Serial.println("error");
            break;
        }
      }
    }
  }
}
