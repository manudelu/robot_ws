// Motor Left Speed
#define L298N_enA 9
// Motor Left Direction
#define L298N_in1 12
#define L298N_in2 13

// Velocity value received from serial comm
double cmd = 0.0;

void setup() {
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);

  // Clockwise Direction
  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);

  Serial.begin(115200);
}

void loop() {
  if(Serial.available()) {
    cmd = Serial.readString().toDouble();
  }

  analogWrite(L298N_enA, cmd*100);
}
