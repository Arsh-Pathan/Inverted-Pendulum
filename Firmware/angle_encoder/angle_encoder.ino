// ─────────────────────────────────────────────────────
// Inverted Pendulum – AS5600 Encoder + TB6612FNG Motor
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
const int oscDuration = 400; // increased from 150ms to 400ms for larger travel distance

// ── TB6612FNG Motor Pins ──
#define MOTOR_AIN1  25
#define MOTOR_AIN2  26
#define MOTOR_PWMA  27
#define MOTOR_STBY  33

// PWM config (ESP32 LEDC)
#define PWM_FREQ     1000   // 1 kHz – lower frequency often gives more raw torque/speed
#define PWM_RES      8      // 8-bit: 0–255

int motorSpeed = 150;  // default PWM duty (0–255)

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
  delay(1000);
  
  // Collect 100 samples to find the exact hanging equilibrium
  long sum = 0;
  for (int i = 0; i < 100; i++) {
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
  // ── 1) Read & send angle (always, every loop) ──
  float rawAngle = encoder.readAngle() * (360.0 / 4096.0);
  
  // Calculate relative to our calibrated hanging down position
  // The hanging position is exactly 180 degrees (bottom).
  // This makes straight UP exactly 0 degrees.
  float angle = rawAngle - zeroOffsetAngle + 180.0;
  
  // Wrap to 0-360
  while (angle < 0) angle += 360.0;
  while (angle >= 360.0) angle -= 360.0;

  Serial.println(angle, 2);

  // ── 2) Check for Start/Stop commands ──
  // Send '1' to start oscillating, '0' to stop
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "1") {
      isOscillating = true;
      movingForward = true;
      lastSwitchTime = millis();
      motorForward(255); // max power
    } else if (cmd == "0") {
      isOscillating = false;
      motorBrake();
    }
  }

  // ── 3) Non-blocking Oscillation Logic ──
  if (isOscillating) {
    if (millis() - lastSwitchTime >= oscDuration) {
      lastSwitchTime = millis();
      movingForward = !movingForward;
      
      if (movingForward) {
        motorForward(255); // max power
      } else {
        motorReverse(255); // max power
      }
    }
  }
}