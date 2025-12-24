#include <Wire.h>
#include <MPU6050.h>
#include <avr/wdt.h>

MPU6050 mpu;

// Motor pins
const int enA = 3, in1 = 8, in2 = 9;
const int enB = 5, in3 = 10, in4 = 11;

// PID parameters (initial)
float Kp = 14.0;
float Ki = 0.6;
float Kd = 0.9;

// Angle and filter
float currentAngle = 0.0;
const float alpha = 0.95;  // complementary filter coeff

// Gyro bias
float gyroBias = 0.0;

// PID state
float integral = 0.0;
float previousError = 0.0;

// Timing
unsigned long prevMicros = 0;
unsigned long lastMPURead = 0;

// Bluetooth & balancing state
bool balancing = false;
int maxPWM = 150;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  Wire.begin();
  mpu.initialize();

  // --- Calibrate gyro bias (stationary) ---
  long sum = 0;
  const int samples = 200;
  for (int i = 0; i < samples; i++) {
    int16_t gx = mpu.getRotationX();
    sum += gx;
    delay(5);
  }
  gyroBias = sum / float(samples);
  Serial.print("Gyro bias = "); Serial.println(gyroBias);

  // Check MPU connection
  if (!mpu.testConnection()) {
    Serial.println("MPU connection failed!");
    Serial1.println("MPU connection failed!");
    while (1);
  }

  // Motor pins
  pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);

  prevMicros = micros();
  lastMPURead = millis();

  Serial.println("angle,output,Kp,Ki,Kd,maxPWM");
  Serial1.println("Self-balancing robot ready");
}

void loop() {
  checkBluetooth();

  if (!balancing) {
    stopMotors();
    return;
  }

  // Failsafe MPU timeout
  unsigned long nowMs = millis();
  if (nowMs - lastMPURead > 2000) {
    Serial.println("MPU timeout, resetting...");
    resetMPU();
    lastMPURead = nowMs;
  }

  // Timing
  unsigned long nowUs = micros();
  float dt = (nowUs - prevMicros) / 1e6;
  prevMicros = nowUs;
  if (dt <= 0 || dt > 0.1) dt = 0.01;

  // Read MPU (raw)
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  lastMPURead = millis();

  // Convert and filter
  float accAngle = atan2(ax, az) * 180.0 / PI;
  float gyroRate = (gy - gyroBias) / 131.0;  // subtract bias
  float newAngle = alpha * (currentAngle + gyroRate * dt)
                 + (1 - alpha) * accAngle;

  // Validate reading
  if (isnan(newAngle) || isinf(newAngle)) {
    // skip this cycle
  } else {
    currentAngle = newAngle;
  }

  // Fallen check
  if (abs(currentAngle) > 45) {
    Serial.println("Fallen! Stopping...");
    Serial1.println("Fallen!");
    balancing = false;
    stopMotors();
    return;
  }

  // PID
  float error = 0 - currentAngle;
  integral += error * dt;
  integral = constrain(integral, -50, 50);  // anti‑windup
  float derivative = (error - previousError) / dt;
  float output = Kp * error + Ki * integral + Kd * derivative;
  previousError = error;

  // Clamp and curve
  float clamped = constrain(output, -maxPWM, maxPWM);
  int speed = (int)((clamped == 0) ? 0 : (clamped > 0
    ? sqrt(clamped / maxPWM) * maxPWM
    : -sqrt(-clamped / maxPWM) * maxPWM));

  // Log CSV
  Serial.print(currentAngle); Serial.print(',');
  Serial.print(output);       Serial.print(',');
  Serial.print(Kp);           Serial.print(',');
  Serial.print(Ki);           Serial.print(',');
  Serial.print(Kd);           Serial.print(',');
  Serial.println(maxPWM);

  Serial1.print(currentAngle); Serial1.print(',');
  Serial1.println(output);

  // Drive motors
  controlMotors(-speed);
}

void controlMotors(int speed) {
  int pwm = abs(speed);
  bool dir = speed > 0;

  // Motor A
  digitalWrite(in1, dir); digitalWrite(in2, !dir);
  analogWrite(enA, pwm);

  // Motor B
  digitalWrite(in3, dir); digitalWrite(in4, !dir);
  analogWrite(enB, pwm);
}

void stopMotors() {
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
  analogWrite(enA, 0); analogWrite(enB, 0);
}

void resetMPU() {
  mpu.reset();
  delay(100);
  mpu.initialize();
  integral = 0;
  currentAngle = 0;
}

void checkBluetooth() {
  while (Serial1.available()) {
    String cmd = Serial1.readStringUntil('\n');
    cmd.trim();
    if (cmd.equalsIgnoreCase("START") || cmd == "s") {
      balancing = true;
      integral = 0; previousError = 0;
      Serial1.println("Balancing ON");
    }
    else if (cmd.equalsIgnoreCase("STOP") || cmd == "x") {
      balancing = false;
      stopMotors();
      Serial1.println("Balancing OFF");
    }
    else if (cmd.startsWith("P")) {
      Kp = cmd.substring(1).toFloat();
      Serial1.print("Kp="); Serial1.println(Kp);
    }
    else if (cmd.startsWith("I")) {
      Ki = cmd.substring(1).toFloat();
      Serial1.print("Ki="); Serial1.println(Ki);
    }
    else if (cmd.startsWith("D")) {
      Kd = cmd.substring(1).toFloat();
      Serial1.print("Kd="); Serial1.println(Kd);
    }
    else if (cmd.startsWith("M")) {
      maxPWM = cmd.substring(1).toInt();
      Serial1.print("maxPWM="); Serial1.println(maxPWM);
    }
    else if (cmd.equalsIgnoreCase("RST") || cmd == "r") {
      resetMPU();
      Serial1.println("MPU reset");
    }
  }
}
