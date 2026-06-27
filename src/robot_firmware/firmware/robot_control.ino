#include <PID_v1.h>

/**** Motor Right ****/ 
#define L298N_enA 9  // Speed
#define L298N_in1 12 // Direction
#define L298N_in2 13 // Direction
#define RIGHT_ENCODER_PHASE_A 3 // interrupt pin (encoder phaseA)
#define RIGHT_ENCODER_PHASE_B 5 // direction pin (encoder phaseB)
/**** Motor Left ****/ 
#define L298N_enB 11  // Speed
#define L298N_in3 7 // Direction
#define L298N_in4 8 // Direction
#define LEFT_ENCODER_PHASE_A 2 // interrupt pin (encoder phaseA)
#define LEFT_ENCODER_PHASE_B 4 // direction pin (encoder phaseB)

const float ENCODER_PPR = 11.0;
const float GEAR_RATIO  = 35.0;
const float RPM_TO_RADS = 0.10472;

volatile unsigned int rightEncoderCounter = 0;
volatile unsigned int leftEncoderCounter = 0;
String rightEncoderSign = "p"; 
String leftEncoderSign = "p"; 
double rightWheelMeasVel = 0.0;   // rad/s
double leftWheelMeasVel = 0.0;    // rad/s
bool isLeftWheelCmd = false;
bool isRightWheelCmd = false;
char value[] = "00.00";
uint8_t valueIdx = 0;
bool isCmdComplete = false;
bool isRightWheelForward = true;
bool isLeftWheelForward = true;
double rightWheelCmdVel = 0.0;
double leftWheelCmdVel = 0.0;

unsigned long lastMs = 0;
const unsigned long interval = 100;

double rightWheelCmd = 0.0;
double leftWheelCmd = 0.0;
double kpR = 12.0;
double kiR = 5.0;
double kdR = 0.8;
double kpL = 12.0;
double kiL = 5.0;
double kdL = 0.8;
PID rightMotor(&rightWheelMeasVel, &rightWheelCmd, &rightWheelCmdVel, kpR, kiR, kdR, DIRECT);
PID leftMotor(&leftWheelMeasVel, &leftWheelCmd, &leftWheelCmdVel, kpL, kiL, kdL, DIRECT);

// Interrupt Callbacks
// When A rises, if B is HIGH ---> clockwise direction
// When A rises, if B is LOW  ---> counterclockwise direction
void rightEncoderCallback() {
  rightEncoderCounter++;
  rightEncoderSign = (digitalRead(RIGHT_ENCODER_PHASE_B) == HIGH) ? "p" : "n";
}

void leftEncoderCallback() {
  leftEncoderCounter++;
  leftEncoderSign = (digitalRead(LEFT_ENCODER_PHASE_B) == HIGH) ? "n" : "p";
}

void setup() {
  /**** Motor Right ****/ 
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(RIGHT_ENCODER_PHASE_B, INPUT);

  /**** Motor Left ****/ 
  pinMode(L298N_enB, OUTPUT);
  pinMode(L298N_in3, OUTPUT);
  pinMode(L298N_in4, OUTPUT);
  pinMode(LEFT_ENCODER_PHASE_B, INPUT);

  // Whenever phaseA goes from low to high (RISING) call the callback
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PHASE_A), rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PHASE_A), leftEncoderCallback, RISING);

  // Clockwise Direction Right Motor
  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);
  // Clockwise Direction Left Motor
  digitalWrite(L298N_in3, HIGH);
  digitalWrite(L298N_in4, LOW);

  rightMotor.SetMode(AUTOMATIC);
  leftMotor.SetMode(AUTOMATIC);

  Serial.begin(115200);
}

void loop() {

  if (Serial.available()) {
    char chr = Serial.read();
    if (chr == 'r') {
      isRightWheelCmd = true;
      isLeftWheelCmd = false;
      valueIdx = 0;
      isCmdComplete = false;
    } else if (chr == 'l') {
      isRightWheelCmd = false;
      isLeftWheelCmd = true;
      valueIdx = 0;
      isCmdComplete = false;
    } else if (chr == 'p') {
      if (isRightWheelCmd && !isRightWheelForward) {
        digitalWrite(L298N_in1, HIGH - digitalRead(L298N_in1));
        digitalWrite(L298N_in2, HIGH - digitalRead(L298N_in2));
        isRightWheelForward = true;
      } else if (isLeftWheelCmd && !isLeftWheelForward) {
        digitalWrite(L298N_in3, HIGH - digitalRead(L298N_in3));
        digitalWrite(L298N_in4, HIGH - digitalRead(L298N_in4));
        isLeftWheelForward = true;
      }
    } else if (chr == 'n') {
      if (isRightWheelCmd && isRightWheelForward) {
        digitalWrite(L298N_in1, HIGH - digitalRead(L298N_in1));
        digitalWrite(L298N_in2, HIGH - digitalRead(L298N_in2));
        isRightWheelForward = false;
      } else if (isLeftWheelCmd && isLeftWheelForward) {
        digitalWrite(L298N_in3, HIGH - digitalRead(L298N_in3));
        digitalWrite(L298N_in4, HIGH - digitalRead(L298N_in4));
        isLeftWheelForward = false;
      }
    } else if (chr == ',') {
      if (isRightWheelCmd) {
        rightWheelCmdVel = atof(value);
      } else if (isLeftWheelCmd) {
        leftWheelCmdVel = atof(value);
        isCmdComplete = true;
      }
      valueIdx = 0;
      value[0] = '0';
      value[1] = '0';
      value[2] = '.';
      value[3] = '0';
      value[4] = '0';
      value[5] = '\0';
    } else {
      if (valueIdx < 5) {
        value[valueIdx] = chr;
        valueIdx++;
      }
    }
  }

  unsigned long currentMs = millis();
  if ((currentMs - lastMs) >= interval) {

    noInterrupts();
    unsigned int rCounterCopy = rightEncoderCounter;
    unsigned int lCounterCopy = leftEncoderCounter;
    rightEncoderCounter = 0;
    leftEncoderCounter = 0;
    interrupts();

    rightWheelMeasVel = 10 * rCounterCopy * (60.0 / (ENCODER_PPR * GEAR_RATIO)) * RPM_TO_RADS;
    leftWheelMeasVel = 10 * lCounterCopy * (60.0 / (ENCODER_PPR * GEAR_RATIO)) * RPM_TO_RADS;
    
    rightMotor.Compute();
    leftMotor.Compute();
    if (rightWheelCmdVel == 0.0) {
      rightWheelCmd = 0.0;
    }
    if (leftWheelCmdVel == 0.0) {
      leftWheelCmd = 0.0;
    }
    
    //String encoderRead = "r" + rightEncoderSign + String(rightWheelMeasVel) + ",l" + leftEncoderSign + String(leftWheelMeasVel) + ",";
    //Serial.println(encoderRead);
    
    // PID tuning test on SerialPlotter
    Serial.print(leftWheelCmdVel);
    Serial.print(" ");
    Serial.println(leftWheelMeasVel);

    lastMs = currentMs;

    analogWrite(L298N_enA, rightWheelCmd);
    analogWrite(L298N_enB, leftWheelCmd);
  }
}