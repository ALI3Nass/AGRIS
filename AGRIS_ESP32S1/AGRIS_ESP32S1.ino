#include <Arduino.h>
#include <Bluepad32.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

/* ================= PIN DEFINITIONS ================= */
// Driver 1 — Left side
#define FL_IN1   32    // AIN1
#define FL_IN2   33    // AIN2
#define FL_PWM   25    // PWMA  → Front-Left motor

#define RL_IN1   26    // BIN1
#define RL_IN2   27    // BIN2
#define RL_PWM   14    // PWMB  → Rear-Left motor

// Driver 2 — Right side
#define FR_IN1   19    // AIN1
#define FR_IN2   18    // AIN2
#define FR_PWM   23    // PWMA  → Front-Right motor

#define RR_IN1   17    // BIN1
#define RR_IN2   21    // BIN2
#define RR_PWM   16    // PWMB  → Rear-Right motor

// Shared STBY
#define STBY_PIN 13

/* ================= PWM CONFIG ================= */
#define PWM_FREQ   1000
#define PWM_RES    8
#define CH_FL      0
#define CH_RL      1
#define CH_FR      2
#define CH_RR      3

/* ================= TUNING ================= */
#define DEADZONE    10
#define MAX_FWD     175
#define MAX_STR     110
#define MAX_ROT     90
#define EXPO        2.2f
#define RAMP_STEP   5
#define LED_PIN     2

// Calibration — tweak if one wheel is faster than others
#define FL_GAIN  1.00f
#define FR_GAIN  1.00f
#define RL_GAIN  1.00f
#define RR_GAIN  1.00f

/* ================= GLOBALS ================= */
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

int g_fwd = 0, g_str = 0, g_rot = 0;
bool g_stop = false;

int cur_fl = 0, cur_fr = 0, cur_rl = 0, cur_rr = 0;

/* ================= EXPO + AXIS PROCESSING ================= */
int processAxis(int val, int maxOut) {
  if (abs(val) < DEADZONE) return 0;
  float normalized = (float)(abs(val) - DEADZONE) / (512.0f - DEADZONE);
  normalized = constrain(normalized, 0.0f, 1.0f);
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
  cur_fl = cur_fr = cur_rl = cur_rr = 0;
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
  // Standard mecanum mixing
  int fl = fwd + str + rot;
  int fr = fwd - str - rot;
  int rl = fwd - str + rot;
  int rr = fwd + str - rot;

  // Normalise — scale down only, never up
  int maxAbs = max({abs(fl), abs(fr), abs(rl), abs(rr)});
  if (maxAbs > 255) {
    fl = (int)((long)fl * 255 / maxAbs);
    fr = (int)((long)fr * 255 / maxAbs);
    rl = (int)((long)rl * 255 / maxAbs);
    rr = (int)((long)rr * 255 / maxAbs);
  }

  // Ramp
  cur_fl = rampTo(cur_fl, fl);
  cur_fr = rampTo(cur_fr, fr);
  cur_rl = rampTo(cur_rl, rl);
  cur_rr = rampTo(cur_rr, rr);

  // Send — apply gain for calibration
  setMotor(CH_FL, FL_IN1, FL_IN2, (int)(cur_fl * FL_GAIN));
  setMotor(CH_FR, FR_IN1, FR_IN2, (int)(cur_fr * FR_GAIN));
  setMotor(CH_RL, RL_IN1, RL_IN2, (int)(cur_rl * RL_GAIN));
  setMotor(CH_RR, RR_IN1, RR_IN2, (int)(cur_rr * RR_GAIN));
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

  g_fwd = processAxis(ctl->axisY(),  MAX_FWD);
  g_str = processAxis(ctl->axisX(),  MAX_STR);
  g_rot = processAxis(ctl->axisRX(), MAX_ROT);

  // L1 = slow mode
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
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // STBY — must be HIGH for drivers to operate
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);

  // Direction pins
  int dirPins[] = {
    FL_IN1, FL_IN2, FR_IN1, FR_IN2,
    RL_IN1, RL_IN2, RR_IN1, RR_IN2
  };
  for (int p : dirPins) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }

  // PWM setup
  ledcSetup(CH_FL, PWM_FREQ, PWM_RES); ledcAttachPin(FL_PWM, CH_FL);
  ledcSetup(CH_FR, PWM_FREQ, PWM_RES); ledcAttachPin(FR_PWM, CH_FR);
  ledcSetup(CH_RL, PWM_FREQ, PWM_RES); ledcAttachPin(RL_PWM, CH_RL);
  ledcSetup(CH_RR, PWM_FREQ, PWM_RES); ledcAttachPin(RR_PWM, CH_RR);

  stopMotors();

  BP32.setup(&onConnectedController, &onDisconnectedController);

  Serial.println("==============================");
  Serial.println("  Mecanum DS4 — TB6612FNG");
  Serial.println("==============================");
  Serial.println("Waiting for controller...");
  Serial.println("Press PS button on DS4");
}

/* ================= LOOP ================= */
void loop() {
  BP32.update();
  processControllers();

  if (g_stop) {
    stopMotors();
  } else {
    mecanumDrive(g_fwd, g_str, g_rot);
  }

  delay(10);
}
