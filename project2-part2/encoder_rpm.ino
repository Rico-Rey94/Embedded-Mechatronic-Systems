const int ENA_PIN = 5;
const int IN1_PIN = 8;
const int IN2_PIN = 9;

const int ENCODER_A_PIN = 2;
const int ENCODER_B_PIN = 3;

const float PULSES_PER_REV = 360.0;
volatile long encoderCount = 0;

// For RPM calculation
unsigned long lastRPMTime = 0;
const unsigned long RPM_INTERVAL_MS = 200; // Measure every 200 ms
long lastEncoderCount = 0;
float rpm = 0.0;

void setup() {
  Serial.begin(115200);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
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
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  }
}

void setMotorPWM(int pwmValue) {
  pwmValue = constrain(pwmValue, 0, 255);
  analogWrite(ENA_PIN, pwmValue);
}

void stopMotor() {
  analogWrite(ENA_PIN, 0);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
}

void calculateRPM() {
  noInterrupts();
  long currentCount = encoderCount;
  interrupts();

  long deltaCount = currentCount - lastEncoderCount;
  lastEncoderCount = currentCount;
  float deltaTimeMinutes = RPM_INTERVAL_MS / 60000.0;

  rpm = (deltaCount / PULSES_PER_REV) / deltaTimeMinutes;

  float rpmMagnitude = abs(rpm); // Use abs(rpm) for always positive RPM

  Serial.print(millis());
  Serial.print("\t\t");
  Serial.print(currentCount);
  Serial.print("\t");
  Serial.print(deltaCount);
  Serial.print("\t\t");
  Serial.println(rpmMagnitude);
}
