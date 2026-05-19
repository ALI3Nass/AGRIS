#include <Arduino.h>
#include <Bluepad32.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

/* ================= PIN DEFINITIONS ================= */
// L298N #1 — Front
#define FL_IN1  26
#define FL_IN2  27
#define FR_IN1  32
#define FR_IN2  33
#define FL_EN   14
#define FR_EN   25

// L298N #2 — Rear
#define RL_IN1  19
#define RL_IN2  18
#define RR_IN1  17
#define RR_IN2  21
#define RL_EN   23
#define RR_EN   16

/* ================= PWM CONFIG ================= */
#define PWM_FREQ  1000
#define PWM_RES   8
#define CH_FL     0
#define CH_FR     1
#define CH_RL     2
#define CH_RR     3

/* ================= TUNING ================= */
#define DEADZONE    20
#define MAX_FWD     200   // Max forward/backward PWM
#define MAX_STR     180   // Max strafe PWM
#define MAX_ROT     120    // Max rotation PWM (~1/3 of forward)
#define EXPO        2.2f  // Expo curve — higher = more gradual low-end
#define RAMP_STEP   8     // Lower = smoother acceleration (was 15)
#define LED_PIN     2

/* ================= GLOBALS ================= */
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

int g_fwd = 0, g_str = 0, g_rot = 0;
bool g_stop = false;

// Ramping state
int cur_fl = 0, cur_fr = 0, cur_rl = 0, cur_rr = 0;

/* ================= EXPO + AXIS PROCESSING ================= */
// Returns a value in range [-maxOut, maxOut] with expo curve applied
int processAxis(int val, int maxOut) {
  if (abs(val) < DEADZONE) return 0;

  // Normalize to 0.0–1.0 after deadzone removal (DS4 axes are ±512)
  float normalized = (float)(abs(val) - DEADZONE) / (512.0f - DEADZONE);
  normalized = constrain(normalized, 0.0f, 1.0f);

  // Apply expo curve
  float curved = pow(normalized, EXPO);

  return (int)((val > 0 ? 1.0f : -1.0f) * curved * maxOut);
}

/* ================= RAMP HELPER ================= */
int rampTo(int current, int target) {
  if (current < target) return min(current + RAMP_STEP, target);
  if (current > target) return max(current - RAMP_STEP, target);
  return current;
}

/* ================= MOTOR FUNCTIONS ================= */
void stopMotors() {
  cur_fl = 0; cur_fr = 0;
  cur_rl = 0; cur_rr = 0;
  ledcWrite(CH_FL, 0); ledcWrite(CH_FR, 0);
  ledcWrite(CH_RL, 0); ledcWrite(CH_RR, 0);
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, LOW);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, LOW);
  digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, LOW);
  digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, LOW);
}

void setMotor(int ch, int in1, int in2, int spd) {
  spd = constrain(spd, -255, 255);
  if (spd > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  } else if (spd < 0) {
    digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);  digitalWrite(in2, LOW);
  }
  ledcWrite(ch, abs(spd));
}

void mecanumDrive(int fwd, int str, int rot) {
  // Calculate target speeds
  int tgt_fl =  fwd + str + rot;
  int tgt_fr =  fwd - str - rot;
  int tgt_rl =  fwd - str + rot;
  int tgt_rr =  fwd + str - rot;

  // Normalize so no value exceeds 255
  int maxVal = max({abs(tgt_fl), abs(tgt_fr), abs(tgt_rl), abs(tgt_rr), 255});
  tgt_fl = tgt_fl * 255 / maxVal;
  tgt_fr = tgt_fr * 255 / maxVal;
  tgt_rl = tgt_rl * 255 / maxVal;
  tgt_rr = tgt_rr * 255 / maxVal;

  // Ramp current toward target
  cur_fl = rampTo(cur_fl, tgt_fl);
  cur_fr = rampTo(cur_fr, tgt_fr);
  cur_rl = rampTo(cur_rl, tgt_rl);
  cur_rr = rampTo(cur_rr, tgt_rr);

  // Apply
  setMotor(CH_FL, FL_IN1, FL_IN2, cur_fl);
  setMotor(CH_FR, FR_IN1, FR_IN2, cur_fr);
  setMotor(CH_RL, RL_IN1, RL_IN2, cur_rl);
  setMotor(CH_RR, RR_IN1, RR_IN2, cur_rr);
}

/* ================= BLUEPAD32 CALLBACKS ================= */
void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (!myControllers[i]) {
      myControllers[i] = ctl;
      digitalWrite(LED_PIN, HIGH);
      Serial.printf("Controller connected at slot %d\n", i);
      break;
    }
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      myControllers[i] = nullptr;
      stopMotors();
      digitalWrite(LED_PIN, LOW);
      Serial.println("Controller disconnected — motors stopped");
      break;
    }
  }
}

/* ================= PROCESS INPUT ================= */
void processGamepad(ControllerPtr ctl) {

  // Cross (X) = emergency stop
  if (ctl->buttons() & BUTTON_A) {
    g_fwd = 0; g_str = 0; g_rot = 0;
    g_stop = true;
    return;
  }
  g_stop = false;

  // Left stick Y  → Forward / Backward  (MAX_FWD)
  // Left stick X  → Strafe Left / Right  (MAX_STR)
  // Right stick X → Rotate               (MAX_ROT — limited to ~1/3)
  g_fwd = processAxis(-ctl->axisY(), MAX_FWD);
  g_str = processAxis( ctl->axisX(), MAX_STR);
  g_rot = processAxis( ctl->axisRX(), MAX_ROT);

  // L1 = slow mode (half speed)
  if (ctl->buttons() & BUTTON_SHOULDER_L) {
    g_fwd /= 2;
    g_str /= 2;
    g_rot /= 2;
  }
}

void processControllers() {
  for (auto ctl : myControllers) {
    if (ctl && ctl->isConnected() && ctl->isGamepad()) {
      processGamepad(ctl);
    }
  }
}

/* ================= SETUP ================= */
void setup() {
  // Disable brownout reset — prevents reboot on motor power spike
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Direction pins
  int dirPins[] = {FL_IN1, FL_IN2, FR_IN1, FR_IN2,
                   RL_IN1, RL_IN2, RR_IN1, RR_IN2};
  for (int p : dirPins) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }

  // PWM channels
  ledcSetup(CH_FL, PWM_FREQ, PWM_RES); ledcAttachPin(FL_EN, CH_FL);
  ledcSetup(CH_FR, PWM_FREQ, PWM_RES); ledcAttachPin(FR_EN, CH_FR);
  ledcSetup(CH_RL, PWM_FREQ, PWM_RES); ledcAttachPin(RL_EN, CH_RL);
  ledcSetup(CH_RR, PWM_FREQ, PWM_RES); ledcAttachPin(RR_EN, CH_RR);

  stopMotors();

  BP32.setup(&onConnectedController, &onDisconnectedController);
  // BP32.forgetBluetoothKeys();  // Uncomment only during pairing debug

  Serial.println("==============================");
  Serial.println("  Mecanum DS4 — Bluepad32");
  Serial.println("==============================");
  Serial.println("Waiting for controller...");
  Serial.println("Press PS button on DS4");
}

/* ================= LOOP ================= */
void loop() {
  BP32.update();
  processControllers();

  if (g_stop) {
    // Emergency stop only — hard immediate halt
    stopMotors();
  } else {
    // Always run mecanumDrive — ramps to 0 naturally on stick release
    mecanumDrive(g_fwd, g_str, g_rot);
  }

  delay(10);
}