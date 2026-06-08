#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <AsyncUDP.h>

// ── WiFi ─────────────────────────────────────────────────────
const char* SSID     = "Pixel 9";
const char* PASSWORD = "12341234";
#define UDP_PORT 4210

// ── Pin Definitions ──────────────────────────────────────────
// No SD card used → GPIO 12 and 13 are free
// GPIO 2 is safe for digital output (ignore the brief boot LED blink)
#define SERVO_PAN_PIN   12    // ⚠ strapping pin — leave floating/low during boot
#define SERVO_TILT_PIN  13
#define LASER_PIN        2

// ── Camera Pins — AI-Thinker ESP32-CAM (OV2640) ──────────────
// Source: randomnerdtutorials.com/esp32-cam-ai-thinker-pinout/
#define PWDN_GPIO_NUM   32    // Active LOW — must be driven
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM   26
#define SIOC_GPIO_NUM   27
#define Y9_GPIO_NUM     35
#define Y8_GPIO_NUM     34
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     36
#define Y5_GPIO_NUM     21
#define Y4_GPIO_NUM     19
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22

// ── Servo range — synced with agris.py ───────────────────────
#define PAN_SERVO_MIN     500
#define PAN_SERVO_MAX    2500
#define PAN_SERVO_CENTER 1632
#define TILT_SERVO_MIN   1050
#define TILT_SERVO_MAX   2200
#define TILT_SERVO_CENTER 1500

Servo panServo;
Servo tiltServo;
WebServer server(80);
AsyncUDP udp;

int  currentPan  = PAN_SERVO_CENTER;
int  currentTilt = TILT_SERVO_CENTER;
bool laserOn     = false;

// ── Camera Setup ─────────────────────────────────────────────
void setupCamera() {
  camera_config_t config;

  // LEDC channel/timer: camera driver uses LEDC for XCLK generation.
  // Timers 0 and 1 can conflict — use 0 here as the camera itself
  // manages XCLK via LEDC_CHANNEL_0 / LEDC_TIMER_0 internally.
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  config.pin_d0    = Y2_GPIO_NUM;
  config.pin_d1    = Y3_GPIO_NUM;
  config.pin_d2    = Y4_GPIO_NUM;
  config.pin_d3    = Y5_GPIO_NUM;
  config.pin_d4    = Y6_GPIO_NUM;
  config.pin_d5    = Y7_GPIO_NUM;
  config.pin_d6    = Y8_GPIO_NUM;
  config.pin_d7    = Y9_GPIO_NUM;
  config.pin_xclk  = XCLK_GPIO_NUM;
  config.pin_pclk  = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href  = HREF_GPIO_NUM;

  // AI-Thinker uses older esp32-camera API naming (pin_sscb_*)
  // If your esp32-camera library uses pin_sccb_*, swap these:
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn  = PWDN_GPIO_NUM;   // GPIO 32 — must be set LOW to power camera
  config.pin_reset = RESET_GPIO_NUM;  // -1 = software reset

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_QVGA;  // 320x240 — optimal for tracking
  config.jpeg_quality = 12;              // 0=best, 63=worst
  config.fb_count     = 2;

  // AI-Thinker PSRAM malloc failed — use internal DRAM instead.
  // Limit fb_count to 1 to avoid running out of DRAM.
  config.fb_location = CAMERA_FB_IN_DRAM;
  config.fb_count    = 1;
  config.grab_mode   = CAMERA_GRAB_WHEN_EMPTY;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init FAILED: 0x%x\n", err);
    Serial.println("Check: PSRAM present? Try fb_location = CAMERA_FB_IN_DRAM if this fails.");
  } else {
    Serial.println("Camera init OK");
  }

  // Sensor tuning after init
  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_framesize(s,     FRAMESIZE_QVGA);
    s->set_quality(s,       12);
    s->set_brightness(s,     1);  // -2 to 2
    s->set_contrast(s,       1);  // -2 to 2
    s->set_saturation(s,     0);  // -2 to 2
    s->set_whitebal(s,       1);  // auto white balance
    s->set_awb_gain(s,       1);
    s->set_exposure_ctrl(s,  1);  // auto exposure
  }
}

// ── MJPEG Stream Handler ─────────────────────────────────────
void handleStream() {
  WiFiClient client = server.client();
  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  server.sendContent(response);

  while (client.connected()) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Frame capture failed, skipping");
      delay(10);
      continue;
    }

    String header = "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ";
    header += fb->len;
    header += "\r\n\r\n";
    server.sendContent(header);
    client.write(fb->buf, fb->len);
    server.sendContent("\r\n");
    esp_camera_fb_return(fb);
  }
}

// ── UDP Command Parser ───────────────────────────────────────
// Packet format: "PAN:1500,TILT:1400,LASER:1"
void setupUDP() {
  if (udp.listen(UDP_PORT)) {
    Serial.printf("UDP listening on port %d\n", UDP_PORT);
    udp.onPacket([](AsyncUDPPacket packet) {
      String msg = String((char*)packet.data(), packet.length());

      Serial.print("UDP rx: ");
      Serial.println(msg);  // Keep this for debugging — remove once confirmed working

      int pIdx = msg.indexOf("PAN:");
      int tIdx = msg.indexOf("TILT:");
      int lIdx = msg.indexOf("LASER:");

      if (pIdx >= 0) {
        int panVal = msg.substring(pIdx + 4, msg.indexOf(',', pIdx)).toInt();
        currentPan = constrain(panVal, PAN_SERVO_MIN, PAN_SERVO_MAX);
        panServo.writeMicroseconds(currentPan);
      }
      if (tIdx >= 0) {
        int tiltVal = msg.substring(tIdx + 5, msg.indexOf(',', tIdx)).toInt();
        currentTilt = constrain(tiltVal, TILT_SERVO_MIN, TILT_SERVO_MAX);
        tiltServo.writeMicroseconds(currentTilt);
      }
      if (lIdx >= 0) {
        laserOn = msg.substring(lIdx + 6).toInt();
        digitalWrite(LASER_PIN, laserOn ? HIGH : LOW);
      }
    });
  } else {
    Serial.println("UDP listen FAILED");
  }
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\nBooting AI-Thinker ESP32-CAM AGRIS...");

  // Laser — GPIO 2
  // Note: GPIO2 has a built-in pull-down and is connected to the onboard
  // blue LED on some boards. It may blink briefly on boot — this is normal.
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);

  // Servos — use LEDC timers 2 & 3 to avoid conflict with camera XCLK
  // (camera driver claims timers 0 and 1 via LEDC_CHANNEL_0 / LEDC_TIMER_0)
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50);
  panServo.attach(SERVO_PAN_PIN,   PAN_SERVO_MIN, PAN_SERVO_MAX);
  tiltServo.attach(SERVO_TILT_PIN, TILT_SERVO_MIN, TILT_SERVO_MAX);
  panServo.writeMicroseconds(PAN_SERVO_CENTER);
  tiltServo.writeMicroseconds(TILT_SERVO_CENTER);
  Serial.println("Servos centered");

  // Camera
  setupCamera();

  // WiFi — station mode
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  Serial.print("Connecting to WiFi");
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 30) {
    delay(500);
    Serial.print(".");
    tries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.println("IP:         " + WiFi.localIP().toString());
    Serial.println("Stream URL: http://" + WiFi.localIP().toString() + "/stream");
    Serial.println("Update agris.py ESP32_IP to: " + WiFi.localIP().toString());
  } else {
    Serial.println("\nWiFi FAILED — check SSID/password");
  }

  // HTTP stream server
  server.on("/stream", handleStream);
  server.begin();
  Serial.println("HTTP server started");

  // UDP
  setupUDP();

  Serial.println("Ready!");
}

// ── Loop ──────────────────────────────────────────────────────
void loop() {
  server.handleClient();
}
