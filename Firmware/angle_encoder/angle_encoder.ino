// Inverted Pendulum – AS5600 Encoder + TB6612FNG Motor (Updated: 2026-07-18 11:16)
// ─────────────────────────────────────────────────────
// AS5600 (I2C):  SDA=21, SCL=22
// TB6612FNG:     AIN1=25, AIN2=26, PWMA=27, STBY=33
// ─────────────────────────────────────────────────────

#include <Wire.h>
#include <AS5600.h>

// ── AS5600 Encoder ──
AS5600 encoder;
float zeroOffsetAngle = 0.0;

// ── Oscillation State ──
bool isOscillating = false;
unsigned long lastSwitchTime = 0;
bool movingForward = true;
int oscDuration = 400; // milliseconds to move in each direction (dynamic)

// ── Balancing State ──
bool isBalancing = false;
float kp = 15.0; // Lowered to reduce overshoot
float ki = 0.0;
float kd = 2.5;  // Increased to add damping
float integral = 0.0;
float prevError = 0.0;
unsigned long lastPidTime = 0;

// ── Advanced Control & Telemetry Variables ──
float currentAngle = 0.0;
float filteredDerivative = 0.0;
float alpha = 0.3; // Low-pass filter for derivative (lower = smoother, higher = more responsive)
unsigned long lastControlTime = 0;
unsigned long lastTelemetryTime = 0;
const unsigned long CONTROL_INTERVAL = 2000; // 2000 microseconds = 2ms (500 Hz PID loop)
const unsigned long TELEMETRY_INTERVAL = 10000; // 10000 microseconds = 10ms (100 Hz updates)

// ── TB6612FNG Motor Pins ──
#define MOTOR_AIN1  25
#define MOTOR_AIN2  26
#define MOTOR_PWMA  27
#define MOTOR_STBY  33

// PWM config (ESP32 LEDC)
#define PWM_FREQ     1000   // 1 kHz – lower frequency often gives more raw torque/speed
#define PWM_RES      8      // 8-bit: 0–255

int motorSpeed = 255;  // default PWM duty (0–255)

// ── Motor helpers ──
void motorForward(int speed) {
  digitalWrite(MOTOR_AIN1, HIGH);
  digitalWrite(MOTOR_AIN2, LOW);
  ledcWrite(MOTOR_PWMA, speed);
}

void motorReverse(int speed) {
  digitalWrite(MOTOR_AIN1, LOW);
  digitalWrite(MOTOR_AIN2, HIGH);
  ledcWrite(MOTOR_PWMA, speed);
}

void motorBrake() {
  digitalWrite(MOTOR_AIN1, HIGH);
  digitalWrite(MOTOR_AIN2, HIGH);
  ledcWrite(MOTOR_PWMA, 0);
}

void motorCoast() {
  digitalWrite(MOTOR_AIN1, LOW);
  digitalWrite(MOTOR_AIN2, LOW);
  ledcWrite(MOTOR_PWMA, 0);
}

// ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // ── I2C + AS5600 Calibration ──
  Wire.begin(21, 22);
  Wire.setClock(400000);
  if (!encoder.begin()) {
    Serial.println("AS5600 not found!");
    while (1);
  }
  
  // Wait 1 second to let pendulum settle
  delay(3000);
  
  // Collect 100 samples to find the exact hanging equilibrium
  long sum = 0;
  for (int i = 0; i < 300; i++) {
    sum += encoder.readAngle();
    delay(10);
  }
  float avgRaw = sum / 100.0;
  zeroOffsetAngle = avgRaw * (360.0 / 4096.0);

  // ── TB6612FNG pins ──
  pinMode(MOTOR_AIN1, OUTPUT);
  pinMode(MOTOR_AIN2, OUTPUT);
  pinMode(MOTOR_STBY, OUTPUT);

  // ESP32 Arduino Core 3.x LEDC API
  ledcAttach(MOTOR_PWMA, PWM_FREQ, PWM_RES);

  // Enable the driver (STBY HIGH = active)
  digitalWrite(MOTOR_STBY, HIGH);
  motorCoast();

  Serial.println("READY");
}

// ─────────────────────────────────────────────────────
void loop() {
  unsigned long nowMicros = micros();

  // ── 1) Check for Start/Stop commands (Immediate Parsing) ──
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "1") {
      isOscillating = true;
      isBalancing = false;
      movingForward = true;
      lastSwitchTime = millis();
      motorForward(motorSpeed);
    } else if (cmd == "0") {
      isOscillating = false;
      isBalancing = false;
      motorBrake();
    } else if (cmd == "F") {
      isOscillating = false;
      isBalancing = false;
      motorForward(motorSpeed);
    } else if (cmd == "B") {
      isOscillating = false;
      isBalancing = false;
      motorReverse(motorSpeed);
    } else if (cmd == "A") {
      isOscillating = false;
      isBalancing = true;
      integral = 0.0;
      float rawAngle = encoder.readAngle() * (360.0 / 4096.0);
      float angle = rawAngle - zeroOffsetAngle;
      while (angle < 0) angle += 360.0;
      while (angle >= 360.0) angle -= 360.0;
      prevError = 180.0 - angle;
      filteredDerivative = 0.0;
      lastControlTime = micros();
    } else if (cmd.startsWith("P,")) {
      motorSpeed = constrain(cmd.substring(2).toInt(), 0, 255);
      if (isOscillating && movingForward) motorForward(motorSpeed);
      if (isOscillating && !movingForward) motorReverse(motorSpeed);
    } else if (cmd.startsWith("D,")) {
      oscDuration = max(10L, cmd.substring(2).toInt());
    } else if (cmd.startsWith("KP,")) {
      kp = cmd.substring(3).toFloat();
    } else if (cmd.startsWith("KI,")) {
      ki = cmd.substring(3).toFloat();
    } else if (cmd.startsWith("KD,")) {
      kd = cmd.substring(3).toFloat();
    }
  }

  // ── 2) Strictly Timed 500 Hz Control Loop ──
  if (nowMicros - lastControlTime >= CONTROL_INTERVAL) {
    float dt = (nowMicros - lastControlTime) / 1000000.0;
    lastControlTime = nowMicros;

    // Read raw sensor values
    float rawAngle = encoder.readAngle() * (360.0 / 4096.0);
    float angle = rawAngle - zeroOffsetAngle;
    while (angle < 0) angle += 360.0;
    while (angle >= 360.0) angle -= 360.0;
    currentAngle = angle; // Store globally for telemetry

    // Non-blocking Oscillation Logic
    if (isOscillating) {
      if (millis() - lastSwitchTime >= oscDuration) {
        lastSwitchTime = millis();
        movingForward = !movingForward;
        
        if (movingForward) {
          motorForward(motorSpeed);
        } else {
          motorReverse(motorSpeed);
        }
      }
    }

    // PID Balancing Logic
    if (isBalancing) {
      float error = 180.0 - angle;
      
      // Shortest-path wrap
      if (error > 180.0) error -= 360.0;
      if (error < -180.0) error += 360.0;

      integral += error * dt;
      integral = constrain(integral, -100.0, 100.0);

      // Raw derivative (with protection against dt = 0)
      float rawDerivative = 0.0;
      if (dt > 0.0) {
        rawDerivative = (error - prevError) / dt;
      }
      prevError = error;

      // Low-pass filter (EMA) applied to derivative to eliminate sensor noise jitter
      filteredDerivative = (alpha * rawDerivative) + ((1.0 - alpha) * filteredDerivative);

      float output = (kp * error) + (ki * integral) + (kd * filteredDerivative);
      
      // Flip the control direction when the pendulum is below the horizontal axis (stable region)
      // vs above the horizontal axis (unstable region). The physics of the system flip at the X-axis.
      bool aboveHorizontal = (angle > 90.0 && angle < 270.0);
      if (!aboveHorizontal) {
        output = -output;
      }
      
      // Linear deadband mapping (Smooth transition from starting torque at 50 to max power at 255)
      int speed = 0;
      float absOutput = abs(output);
      if (absOutput > 0.05) {
        // Map output 0..255 smoothly to motor power range 50..255
        speed = 50 + (int)(absOutput * (255 - 50) / 255.0);
        speed = constrain(speed, 50, 255);
      }

      // Flipped motor direction for the upright region to prevent anti-balancing
      if (output > 0) {
      } else {
        motorReverse(speed);
        motorForward(speed);
      }
    }
  }

  // ── 3) Throttled Telemetry at 100 Hz (Drastically reduces serial buffer jitter) ──
  if (nowMicros - lastTelemetryTime >= TELEMETRY_INTERVAL) {
    lastTelemetryTime = nowMicros;
    Serial.println(currentAngle, 2);
  }
}