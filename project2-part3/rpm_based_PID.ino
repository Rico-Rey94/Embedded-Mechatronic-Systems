const int ENA_PIN = 5;
const int IN1_PIN = 8;
const int IN2_PIN = 9;
const int ENCODER_A_PIN = 2;
const int ENCODER_B_PIN = 3;

// For your encoder
const float PULSES_PER_REV = 360.0;

volatile long encoderCount = 0;

const unsigned long CONTROL_INTERVAL_MS = 100; // PID/evaluation every 100 ms
unsigned long lastControlTime = 0;
long lastEncoderCount = 0;

// RPM filter
float measuredRPM = 0.0;
float filteredRPM = 0.0;
const float RPM_FILTER_ALPHA = 0.7;

// PID control
float setpointRPM = 120.0; // Change as needed
float Kp = 2.0;
float Ki = 1.0;
float Kd = 0.05;

float error = 0.0;
float prevError = 0.0;
float integral = 0.0;
float derivative = 0.0;
float controlOutput = 0.0;
int pwmCommand = 0;

const float INTEGRAL_LIMIT = 100.0; // Integral anti-windup

void setup() {
  Serial.begin(115200);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR, RISING);

  setMotorDirection(true);
  stopMotor();

  lastControlTime = millis();

  Serial.println("Closed-Loop PID Speed Control Started");
  Serial.println("Setpoint RPM = 120");
  Serial.println("Time(ms)\tSetpoint\tMeasuredRPM\tPWM\tError");
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastControlTime >= CONTROL_INTERVAL_MS) {
    lastControlTime = currentTime;
    updateRPM();
    updatePID();
    applyControl();
    printStatus();
  }
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

void updateRPM() {
  noInterrupts();
  long currentCount = encoderCount;
  interrupts();

  long deltaCount = currentCount - lastEncoderCount;
  lastEncoderCount = currentCount;

  float deltaTimeMinutes = CONTROL_INTERVAL_MS / 60000.0;
  measuredRPM = (deltaCount / PULSES_PER_REV) / deltaTimeMinutes;
  measuredRPM = abs(measuredRPM);

  filteredRPM = RPM_FILTER_ALPHA * filteredRPM + (1.0 - RPM_FILTER_ALPHA) * measuredRPM;
}

void updatePID() {
  float dt = CONTROL_INTERVAL_MS / 1000.0;
  error = setpointRPM - filteredRPM;

  integral += error * dt;
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  derivative = (error - prevError) / dt;

  controlOutput = Kp * error + Ki * integral + Kd * derivative;
  prevError = error;
}

void applyControl() {
  int basePWM = 70; // Feedforward to overcome static friction
  pwmCommand = basePWM + (int)controlOutput;
  pwmCommand = constrain(pwmCommand, 0, 255);

  setMotorDirection(true);
  setMotorPWM(pwmCommand);
}

void printStatus() {
  Serial.print(millis());
  Serial.print("\t\t");
  Serial.print(setpointRPM);
  Serial.print("\t\t");
  Serial.print(filteredRPM);
  Serial.print("\t\t");
  Serial.print(pwmCommand);
  Serial.print("\t");
  Serial.println(error);
}
