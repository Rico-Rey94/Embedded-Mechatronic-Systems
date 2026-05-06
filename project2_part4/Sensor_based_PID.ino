// Pin Definitions – Your Wiring
const int ENB_PIN         = 5;   // L298N ENB (PWM)
const int IN3_PIN         = 8;   // L298N IN3
const int IN4_PIN         = 9;   // L298N IN4
const int ENCODER_A_PIN   = 2;   // Encoder channel A (interrupt)
const int ENCODER_B_PIN   = 3;   // Encoder channel B
// SENSOR PINS
const int IR_PIN          = 6;   // IR sensor OUT
const int BUTTON_PIN      = 7;   // Push button
const int LDR_PIN         = A0;  // LDR OUT
const int TEMP_PIN        = A1;  // Temp sensor OUT
// Encoder/motor configuration
const float ENCODER_PULSES_PER_MOTOR_REV = 360.0;
const float GEAR_RATIO = 34.0;
const float PULSES_PER_REV = ENCODER_PULSES_PER_MOTOR_REV * GEAR_RATIO; // Output shaft
const unsigned long CONTROL_INTERVAL_MS = 100;
// PID Params
float setpointRPM = 120.0;
float Kp = 2.0;
float Ki = 1.0;
float Kd = 0.15;
const float INTEGRAL_LIMIT = 100.0;
volatile long encoderCount = 0;
long lastEncoderCount = 0;
unsigned long lastControlTime = 0;
float measuredRPM = 0.0;
float filteredRPM = 0.0;
const float RPM_FILTER_ALPHA = 0.9;
float error = 0.0, prevError = 0.0, integral = 0.0, derivative = 0.0, controlOutput = 0.0;
int pwmCommand = 0;
// Button debouncing
bool lastButtonState = HIGH, buttonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 50;
// Mode select
int mode = 0; // 0 = speed control, 1 = sensor adaptive
// Sensor thresholds (adjust for your hardware)
const int OBSTACLE_DETECTED_STATE = LOW;
const int LIGHT_LOW_THRESHOLD = 400;
const int LIGHT_HIGH_THRESHOLD = 700;
const float TEMP_WARNING_C = 35.0;
const float TEMP_LIMIT_C = 45.0;

void setup() {
  Serial.begin(115200);
  pinMode(ENB_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR, RISING);
  pinMode(IR_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  stopMotor();
  setMotorDirection(true);
  lastControlTime = millis();
  Serial.println("Smart Motor Control - Mode 0 = Speed, 1 = Adaptive");
}

void loop() {
  updateButton();
  unsigned long now = millis();
  if (now - lastControlTime >= CONTROL_INTERVAL_MS) {
    lastControlTime = now;
    updateRPM();
    if (mode == 0) {
      runSpeedControlMode();
    } else {
      runAdaptiveMode();
    }
  }
}

// Encoder interrupt routine for direction
void encoderISR() {
  int bState = digitalRead(ENCODER_B_PIN);
  if (bState == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

// Button logic with debounce for mode switching
void updateButton() {
  bool reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (reading != buttonState) {
      buttonState = reading;
      // Active LOW
      if (buttonState == LOW) {
        mode++;
        if (mode > 1) mode = 0;
        integral = 0.0;
        prevError = 0.0;
        Serial.print("Mode changed to: ");
        Serial.println(mode);
      }
    }
  }
  lastButtonState = reading;
}

// Calculate measured RPM using encoder counts
void updateRPM() {
  noInterrupts();
  long currentCount = encoderCount;
  interrupts();
  long deltaCount = currentCount - lastEncoderCount;
  lastEncoderCount = currentCount;
  float deltaTimeMinutes = CONTROL_INTERVAL_MS / 60000.0;
  measuredRPM = ((float)deltaCount / PULSES_PER_REV) / deltaTimeMinutes;
  measuredRPM = abs(measuredRPM);
  // FIX: use '*' for multiplication
  filteredRPM = RPM_FILTER_ALPHA * filteredRPM + (1.0 - RPM_FILTER_ALPHA) * measuredRPM;
}

// Set motor direction (true=forward, false=reverse)
void setMotorDirection(bool forward) {
  if (forward) {
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
  } else {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
  }
}

// Set motor PWM value (0-255)
void setMotorPWM(int pwmValue) {
  pwmValue = constrain(pwmValue, 0, 255);
  analogWrite(ENB_PIN, pwmValue);
}

// Stop the motor
void stopMotor() {
  analogWrite(ENB_PIN, 0);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}

// Standard PID controller
void updatePID() {
  float dt = CONTROL_INTERVAL_MS / 1000.0;
  error = setpointRPM - filteredRPM;
  integral += error * dt;
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  derivative = (error - prevError) / dt;
  // FIX: use '*' for multiplication
  controlOutput = Kp * error + Ki * integral + Kd * derivative;
  prevError = error;
}

// --------------------------------
// Mode 1: Speed Control (picture: maintain constant speed using encoder feedback)
void runSpeedControlMode() {
  setpointRPM = 120.0; // fixed target speed
  setMotorDirection(true);
  updatePID();
  pwmCommand = (int)controlOutput;
  pwmCommand = constrain(pwmCommand, 0, 255);
  setMotorPWM(pwmCommand);
  int ldrValue = analogRead(LDR_PIN);
  bool obstacleDetected = (digitalRead(IR_PIN) == OBSTACLE_DETECTED_STATE);
  float tempC = readTemperatureC();
  printStatus(ldrValue, obstacleDetected, tempC);
}

// --------------------------------
// Mode 2: Sensor-Adaptive Mode 
void runAdaptiveMode() {
  int ldrValue = analogRead(LDR_PIN);
  bool obstacleDetected = (digitalRead(IR_PIN) == OBSTACLE_DETECTED_STATE);
  float tempC = readTemperatureC();

  setpointRPM = 120.0; // default speed

  // Light-based adaptation (low light -> reduce speed)
  if (ldrValue < LIGHT_LOW_THRESHOLD) {
    setpointRPM = 60.0;
  } else if (ldrValue > LIGHT_HIGH_THRESHOLD) {
    setpointRPM = 140.0;
  } else {
    setpointRPM = 100.0;
  }

  // Temperature adaptation (high temp -> limit output)
  if (tempC >= TEMP_WARNING_C && tempC < TEMP_LIMIT_C) {
    setpointRPM *= 0.7; // reduce speed if warning
  }
  // critical temperature - stop motor
  if (tempC >= TEMP_LIMIT_C) {
    stopMotor();
    Serial.println("TEMP LIMIT EXCEEDED - MOTOR STOPPED");
    printStatus(ldrValue, obstacleDetected, tempC);
    return;
  }

  // Obstacle detection (stop motor)
  if (obstacleDetected) {
    stopMotor();
    Serial.println("OBSTACLE DETECTED - MOTOR STOPPED");
    printStatus(ldrValue, obstacleDetected, tempC);
    return;
  }

  setMotorDirection(true);
  updatePID();
  pwmCommand = (int)controlOutput;
  pwmCommand = constrain(pwmCommand, 0, 255);
  setMotorPWM(pwmCommand);
  printStatus(ldrValue, obstacleDetected, tempC);
}

// Read analog temperature sensor (e.g., LM35, 10mV/°C)
float readTemperatureC() {
  int raw = analogRead(TEMP_PIN);
  float voltage = raw * (5.0 / 1023.0);
  float tempC = voltage * 100.0;
  return tempC;
}

// Serial print all sensor data and states
void printStatus(int ldrValue, bool obstacleDetected, float tempC) {
  Serial.print("Mode: ");
  Serial.print(mode);
  Serial.print(" | Setpoint RPM: ");
  Serial.print(setpointRPM);
  Serial.print(" | Measured RPM: ");
  Serial.print(filteredRPM);
  Serial.print(" | PWM: ");
  Serial.print(pwmCommand);
  Serial.print(" | LDR: ");
  Serial.print(ldrValue);
  Serial.print(" | Temp C: ");
  Serial.print(tempC);
  Serial.print(" | Obstacle: ");
  Serial.print(obstacleDetected ? "YES" : "NO");
  Serial.print(" | Encoder Count: ");
  Serial.println(encoderCount);
}
