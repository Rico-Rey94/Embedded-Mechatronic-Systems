const int ENA_PIN = 5;
const int IN1_PIN = 8;
const int IN2_PIN = 9;

// Comment out encoder-related code
// const int ENCODER_A_PIN = 2;
// const int ENCODER_B_PIN = 3;

// volatile long encoderCount = 0;

void setup() {
  // Serial.begin(115200);

  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  // pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  // pinMode(ENCODER_B_PIN, INPUT_PULLUP);

  // attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR, RISING);

  // Serial.println("Basic Motor Control Test Started");
  // Serial.println("Motor will run forward, stop, backward, stop...");
}

void loop() {
  // Run forward at medium speed
  setMotorDirection(true);
  setMotorPWM(150);
  // printEncoderData();
  delay(3000);

  // Stop
  stopMotor();
  // printEncoderData();
  delay(2000);

  // Run backward at medium speed
  setMotorDirection(false);
  setMotorPWM(150);
  // printEncoderData();
  delay(3000);

  // Stop
  stopMotor();
  // printEncoderData();
  delay(2000);
}

/*
// Comment out encoder ISR
void encoderISR() {
  int bState = digitalRead(ENCODER_B_PIN);
  if (bState == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}
*/

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

/*
// Comment out encoder print
void printEncoderData() {
  unsigned long currentTime = millis();
  noInterrupts();
  long currentCount = encoderCount;
  interrupts();
  long deltaCount = currentCount - lastEncoderCount;
  lastEncoderCount = currentCount;
  Serial.print("Encoder Count = ");
  Serial.print(currentCount);
  Serial.print(" | Change since last print = ");
  Serial.println(deltaCount);
}
*/
