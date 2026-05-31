/**** Motor Left ****/ 
#define L298N_enA 9  // Speed
#define L298N_in1 12 // Direction
#define L298N_in2 13 // Direction
#define right_encoder_phaseA 3 // interrupt pin (encoder phaseA)
#define right_encoder_phaseB 5 // direction pin (encoder phaseB)

const float ENCODER_PPR    = 11.0;
const float GEAR_RATIO     = 35.0;
const float LOOP_PERIOD_MS = 100.0;
const float RPM_TO_RADS    = 0.10472;

volatile unsigned int rightEncoderCounter = 0;
volatile int rightEncoderSign = 1; // +1 clockwise, -1 counterclockwise
double rightWheelMeasVel = 0.0;    // rad/s

// Interrupt Callback
// When A rises, if B is HIGH ---> clockwise direction
// When A rises, if B is LOW  ---> counterclockwise direction
void rightEncoderCallback() {
  rightEncoderCounter++;
  rightEncoderSign = (digitalRead(right_encoder_phaseB) == HIGH) ? 1 : -1;
}

void setup() {
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(right_encoder_phaseB, INPUT);

  // Whenever phaseA goes from low to high (RISING) call the callback
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);

  // Clockwise Direction
  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);

  Serial.begin(115200);
}

void loop() {
  noInterrupts();
  unsigned int rightCount = rightEncoderCounter;
  int rightSign = rightEncoderSign;
  rightEncoderCounter = 0;
  interrupts();

  // Velocity calculation
  float cyclesPerSecond = 1000.0 / LOOP_PERIOD_MS;
  float rpm = cyclesPerSecond * rightCount * (60.0 / (ENCODER_PPR * GEAR_RATIO));
  rightWheelMeasVel = rightSign * rpm * RPM_TO_RADS;

  Serial.println(rightWheelMeasVel);

  analogWrite(L298N_enA, 100);
  delay((int)LOOP_PERIOD_MS);
}

