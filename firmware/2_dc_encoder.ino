// --- Pinout Silnik 1 ---
const int IN1 = 4;
const int IN2 = 5;
const int ENCODER_C1 = 8;
const int ENCODER_C2 = 9;

// --- Pinout Silnik 2 ---
const int IN3 = 6;
const int IN4 = 7;
const int ENCODER_C3 = 10;
const int ENCODER_C4 = 11;

// --- PWM ---
const int PWM_FREQ = 500; // Hz
const int PWM_RES = 8;     // 8-bit (0–255)
const int PWM_CH1 = 0;
const int PWM_CH2 = 1;
const int MIN_EFFECTIVE_PWM = 20;

// ⚙️ Stałe silnika JGB37-520
const float GEAR_RATIO = 56.0;
const int PPR_MOTOR = 11;
const int QUADRATURE_FACTOR = 4;
const float TICKS_PER_REVOLUTION = PPR_MOTOR * GEAR_RATIO * QUADRATURE_FACTOR;
int MAX_RPM = 187;

// --- PID ---
float KP = 1.5;
float KI = 0.3;
float KD = 0.05;

// --- Struktura silnika ---
struct Motor {
  int in1, in2;
  int pwmCh;
  volatile long encoderTicks = 0;
  volatile int8_t lastEncoded = 0;
  float currentRPM = 0.0;
  float targetRPM = 0.0;
  bool running = false;
  int motorSpeed = 0;
  unsigned long lastPidTime = 0;
  float errorSum = 0;
  float lastError = 0;
  long lastEncoderTicks = 0;
  bool velocityPrinting = false;

  float KP, KI, KD;

  Motor(int a, int b, int ch, float kp, float ki, float kd) : in1(a), in2(b), pwmCh(ch),
    encoderTicks(0), lastEncoded(0), currentRPM(0), targetRPM(0),
    running(false), motorSpeed(0), lastPidTime(0), errorSum(0),
    lastError(0), lastEncoderTicks(0), velocityPrinting(false),
    KP(kp), KI(ki), KD(kd) {}
};

// Motor motor1(IN1, IN2, PWM_CH1, 6.0, 3.0, 0.05);
// Motor motor2(IN3, IN4, PWM_CH2, 6.0, 3.0, 0.05);

Motor motor1(IN1, IN2, PWM_CH1, 1.5, 0.3, 0.05);
Motor motor2(IN3, IN4, PWM_CH2, 1.5, 0.3, 0.05);

// --- Obsługa enkoderów ---
void IRAM_ATTR handleEncoder1() {
  int8_t MSB = digitalRead(ENCODER_C1);
  int8_t LSB = digitalRead(ENCODER_C2);
  int8_t encoded = (MSB << 1) | LSB;
  int8_t sum = (motor1.lastEncoded << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) motor1.encoderTicks++;
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) motor1.encoderTicks--;
  motor1.lastEncoded = encoded;
}

void IRAM_ATTR handleEncoder2() {
  int8_t MSB = digitalRead(ENCODER_C3);
  int8_t LSB = digitalRead(ENCODER_C4);
  int8_t encoded = (MSB << 1) | LSB;
  int8_t sum = (motor2.lastEncoded << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) motor2.encoderTicks++;
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) motor2.encoderTicks--;
  motor2.lastEncoded = encoded;
}

// --- Sterowanie PWM ---
void setMotor(Motor &m, int speed) {
  int absSpeed = constrain(abs(speed), 0, 255);
  if (speed > 0) {
    ledcDetachPin(m.in2);
    digitalWrite(m.in2, LOW);
    ledcAttachPin(m.in1, m.pwmCh);
    ledcWrite(m.pwmCh, absSpeed);
  } else if (speed < 0) {
    ledcDetachPin(m.in1);
    digitalWrite(m.in1, LOW);
    ledcAttachPin(m.in2, m.pwmCh);
    ledcWrite(m.pwmCh, absSpeed);
  } else {
    ledcDetachPin(m.in1);
    ledcDetachPin(m.in2);
    digitalWrite(m.in1, LOW);
    digitalWrite(m.in2, LOW);
    ledcWrite(m.pwmCh, 0);
  }
}

// --- PID ---
void updatePID(Motor &m) {
  if (m.targetRPM == 0) {
    setMotor(m, 0);
    m.motorSpeed = 0;
    m.errorSum = 0;
    m.lastError = 0;
    m.currentRPM = 0;
    return;
  }
  unsigned long now = millis();
  float deltaTime = (now - m.lastPidTime) / 1000.0;
  if (deltaTime < 0.01) return;

  long deltaTicks = m.encoderTicks - m.lastEncoderTicks;
  m.currentRPM = (deltaTicks / deltaTime) / TICKS_PER_REVOLUTION * 60.0;
  m.lastEncoderTicks = m.encoderTicks;

  float error = m.targetRPM - m.currentRPM;
  m.errorSum += error * deltaTime;
  m.errorSum = constrain(m.errorSum, -500.0, 500.0);
  float derivative = (error - m.lastError) / deltaTime;

  float feedForward = (m.targetRPM / MAX_RPM) * 150.0;
  float pidOutput = feedForward 
                + m.KP * error 
                + m.KI * m.errorSum 
                + m.KD * derivative;

  if (pidOutput > 0 && pidOutput < MIN_EFFECTIVE_PWM) pidOutput = MIN_EFFECTIVE_PWM;
  else if (pidOutput < 0 && pidOutput > -MIN_EFFECTIVE_PWM) pidOutput = -MIN_EFFECTIVE_PWM;

  pidOutput = constrain(pidOutput, -255, 255);
  m.motorSpeed = round(pidOutput);
  setMotor(m, m.motorSpeed);

  m.lastError = error;
  m.lastPidTime = now;

  if (m.velocityPrinting) {
    Serial.printf("Motor PWM CH%d: %.2f RPM (%ld ticks)\n", m.pwmCh, m.currentRPM, m.encoderTicks);
  }
}

// --- Serial command handler ---
void handleSerialInput() {
  static String input = "";
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      input.trim();

      if (input.startsWith("rpm ")) {
        float targetRPM = input.substring(4).toFloat();
        motor1.targetRPM = targetRPM;
        motor2.targetRPM = targetRPM;
        motor1.running = true;
        motor2.running = true;
        motor1.errorSum = 0;
        motor2.errorSum = 0;
        motor1.lastError = 0;
        motor2.lastError = 0;
        motor1.lastPidTime = millis();
        motor2.lastPidTime = millis();
        Serial.printf("Motor 1 target RPM: %.2f\n", motor1.targetRPM);
        Serial.printf("Motor 2 target RPM: %.2f\n", motor2.targetRPM);
      } else if (input.equalsIgnoreCase("stop")) {
        motor1.running = false;
        setMotor(motor1, 0);
        motor1.targetRPM = 0;
        motor1.errorSum = 0;
        motor1.lastError = 0;
        Serial.println("Motor 1 stopped");

        motor2.running = false;
        setMotor(motor2, 0);
        motor2.targetRPM = 0;
        motor2.errorSum = 0;
        motor2.lastError = 0;
        Serial.println("Motor 2 stopped");
      } else if (input.startsWith("CMD,")) {
        String values = input.substring(4);
        int commaIndex = values.indexOf(',');
        if (commaIndex != -1) {
          int rpm2 = values.substring(0, commaIndex).toInt();
          int rpm1 = values.substring(commaIndex + 1).toInt();
          motor1.targetRPM = rpm1;
          motor1.running = true;
          motor2.targetRPM = rpm2;
          motor2.running = true;
          motor1.errorSum = 0;
          motor2.errorSum = 0;
          motor1.lastError = 0;
          motor2.lastError = 0;
          motor1.lastPidTime = millis();
          motor2.lastPidTime = millis();
        }
      } else if (input.equals("ENC")) {
        long leftTicks = motor2.encoderTicks;
        long rightTicks = motor1.encoderTicks;

        Serial.printf("ENC,%ld,%ld\n", leftTicks, rightTicks);
      }

      // Silnik 1
      if (input.startsWith("rpm1 ")) {
        motor1.targetRPM = input.substring(5).toFloat();
        motor1.running = true;
        motor1.errorSum = 0;
        motor1.lastError = 0;
        motor1.lastPidTime = millis();
        Serial.printf("Motor 1 target RPM: %.2f\n", motor1.targetRPM);
      } else if (input.startsWith("speed1 ")) {
        motor1.motorSpeed = constrain(input.substring(7).toInt(), -255, 255);
        motor1.targetRPM = 0;
        motor1.running = true;
        setMotor(motor1, motor1.motorSpeed);
        Serial.printf("Motor 1 speed (RAW PWM): %d\n", motor1.motorSpeed);
      } else if (input.equalsIgnoreCase("stop1")) {
        motor1.running = false;
        setMotor(motor1, 0);
        motor1.targetRPM = 0;
        motor1.errorSum = 0;
        motor1.lastError = 0;
        Serial.println("Motor 1 stopped");
      } else if (input.equalsIgnoreCase("ticks1")) {
        Serial.printf("Motor 1 ticks: %ld\n", motor1.encoderTicks);
      } else if (input.equalsIgnoreCase("vel1 start")) {
        motor1.velocityPrinting = true;
      } else if (input.equalsIgnoreCase("vel1 stop")) {
        motor1.velocityPrinting = false;
      } else if (input.startsWith("p1 ")) { 
        motor1.KP = input.substring(3).toFloat(); 
        Serial.printf("Motor1 KP=%.2f\n", motor1.KP); 
      } else if (input.startsWith("i1 ")) { 
        motor1.KI = input.substring(3).toFloat(); 
        Serial.printf("Motor1 KI=%.2f\n", motor1.KI); 
      } else if (input.startsWith("d1 ")) { 
        motor1.KD = input.substring(3).toFloat(); 
        Serial.printf("Motor1 KD=%.2f\n", motor1.KD); 
      }

      // Silnik 2
      else if (input.startsWith("rpm2 ")) {
        motor2.targetRPM = input.substring(5).toFloat();
        motor2.running = true;
        motor2.errorSum = 0;
        motor2.lastError = 0;
        motor2.lastPidTime = millis();
        Serial.printf("Motor 2 target RPM: %.2f\n", motor2.targetRPM);
      } else if (input.startsWith("speed2 ")) {
        motor2.motorSpeed = constrain(input.substring(7).toInt(), -255, 255);
        motor2.targetRPM = 0;
        motor2.running = true;
        setMotor(motor2, motor2.motorSpeed);
        Serial.printf("Motor 2 speed (RAW PWM): %d\n", motor2.motorSpeed);
      } else if (input.equalsIgnoreCase("stop2")) {
        motor2.running = false;
        setMotor(motor2, 0);
        motor2.targetRPM = 0;
        motor2.errorSum = 0;
        motor2.lastError = 0;
        Serial.println("Motor 2 stopped");
      } else if (input.equalsIgnoreCase("ticks2")) {
        Serial.printf("Motor 2 ticks: %ld\n", motor2.encoderTicks);
      } else if (input.equalsIgnoreCase("vel2 start")) {
        motor2.velocityPrinting = true;
      } else if (input.equalsIgnoreCase("vel2 stop")) {
        motor2.velocityPrinting = false;
      } else if (input.startsWith("p2 ")) { 
        motor2.KP = input.substring(3).toFloat(); 
        Serial.printf("Motor2 KP=%.2f\n", motor2.KP); 
      } else if (input.startsWith("i2 ")) { 
        motor2.KI = input.substring(3).toFloat(); 
        Serial.printf("Motor2 KI=%.2f\n", motor2.KI); 
      } else if (input.startsWith("d2 ")) { 
        motor2.KD = input.substring(3).toFloat(); 
        Serial.printf("Motor2 KD=%.2f\n", motor2.KD); 
      }

      input = "";
    } else {
      input += ch;
    }
  }
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Setting up");

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(ENCODER_C1, INPUT_PULLUP);
  pinMode(ENCODER_C2, INPUT_PULLUP);
  pinMode(ENCODER_C3, INPUT_PULLUP);
  pinMode(ENCODER_C4, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_C1), handleEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_C2), handleEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_C3), handleEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_C4), handleEncoder2, CHANGE);

  ledcSetup(PWM_CH1, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH2, PWM_FREQ, PWM_RES);

  Serial.println("✅ 2x Silnik JGB37-520 z PID gotowe. Komendy: rpm1/rpm2 <val>, stop1/stop2, speed1/speed2 <val>, ticks1/ticks2, vel1/vel2 start/stop");
}

// --- Loop ---
void loop() {
  handleSerialInput();

  if (motor1.running) updatePID(motor1);
  if (motor2.running) updatePID(motor2);
}
