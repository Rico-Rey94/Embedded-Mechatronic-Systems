const int ENB_PIN = 5;
const int IN3_PIN = 8;
const int IN4_PIN = 9;
const int ENCODER_A_PIN = 2;
const int ENCODER_B_PIN = 3;

// Change this to match your encoder's pulses per motor shaft revolution
const float ENCODER_PULSES_PER_MOTOR_REV = 360.0
const float GEAR_RATIO = 34.0;
const float PULSES_PER_REV = ENCODER_PULSES_PER_MOTOR_REV * GEAR_RATIO; // Output shaft pulses/rev

volatile long encoderCount = 0;

// For RPM calculation
unsigned long lastRPMTime = 0;
const unsigned long RPM_INTERVAL_MS = 200; // Measure every 200 ms
long lastEncoderCount = 0;
float rpm = 0.0;

void setup() {
  delay(2000);
  Serial.begin(115200);
  pinMode(ENB_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR, RISING);
  setMotorDirection(true);
  setMotorPWM(150);
  lastRPMTime = millis();
  Serial.println("Step 2: Motor RPM Measurement Started");
  Serial.println("Motor running forward at PWM = 150");
  Serial.println("Time(ms)\tCount\tDeltaCount\tRPM");
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastRPMTime >= RPM_INTERVAL_MS) {
    calculateRPM();
    lastRPMTime = currentTime;
  }
  // The motor keeps running forward at PWM=150 during the test
}

void encoderISR() {
  int bState = digitalRead(ENCODER_B_PIN);
  if (bState == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void setMotorDirection(bool forward) {
  if (forward) {
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
  } else {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
  }
}

void setMotorPWM(int pwmValue) {
  pwmValue = constrain(pwmValue, 0, 255);
  analogWrite(ENB_PIN, pwmValue);
}

void stopMotor() {
  analogWrite(ENB_PIN, 0);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}

void calculateRPM() {
  noInterrupts();
  long currentCount = encoderCount;
  interrupts();
  long deltaCount = currentCount - lastEncoderCount;
  lastEncoderCount = currentCount;
  float deltaTimeMinutes = RPM_INTERVAL_MS / 60000.0;
  rpm = (deltaCount / PULSES_PER_REV) / deltaTimeMinutes;
  float rpmMagnitude = abs(rpm); // Always positive RPM
  Serial.print(millis());
  Serial.print("\t\t");
  Serial.print(currentCount);
  Serial.print("\t");
  Serial.print(deltaCount);
  Serial.print("\t\t");
  Serial.println(rpmMagnitude);
}
