#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <ESP32Servo.h>

// ---- CHANGE THESE ----
const char* WIFI_SSID = "CodeXcel";
const char* WIFI_PASS = "bittu123";

// AI-Thinker ESP32-CAM camera pin mapping
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Direct GPIOs: GPIO3 is shared with UART RX during upload; disconnect programmer after upload.
// GPIO2 is a boot strap pin; TRIG is an input on the HC-SR04 and must not pull it high.
// GPIO4 also drives the onboard flash LED. Do not use the flash LED in this project.
const int SERVO_PIN = 3;
const int SONAR_TRIG_PIN = 2;
const int SONAR_ECHO_PIN = 4; // HC-SR04 ECHO MUST use a voltage divider to reduce 5V to ~3.3V

// PCF8574 I2C expander, using GPIO13/14 (camera does not use these pins; SD card must not be used)
const int I2C_SDA_PIN = 13;
const int I2C_SCL_PIN = 14;
const uint8_t PCF_ADDR = 0x20; // common address; adjust if A0/A1/A2 jumpers differ

// PCF8574 P0..P3 -> L298N IN1..IN4
const uint8_t IN1_BIT = 0;
const uint8_t IN2_BIT = 1;
const uint8_t IN3_BIT = 2;
const uint8_t IN4_BIT = 3;

const int OBSTACLE_CM = 25;
const unsigned long COMMAND_TIMEOUT_MS = 800;
const unsigned long TURN_PULSE_MS = 280;

WebServer server(80);
WiFiServer streamServer(81);
Servo scanServo;

volatile char requestedCommand = 'S';
volatile unsigned long lastCommandMs = 0;
uint8_t motorState = 0x00;
bool expanderOK = false;
unsigned long lastSafetyCheck = 0;

void pcfWrite(uint8_t value) {
  if (!expanderOK) return;
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(value);
  Wire.endTransmission();
  motorState = value;
}

void setMotorBits(bool in1, bool in2, bool in3, bool in4) {
  uint8_t value = 0;
  if (in1) value |= (1 << IN1_BIT);
  if (in2) value |= (1 << IN2_BIT);
  if (in3) value |= (1 << IN3_BIT);
  if (in4) value |= (1 << IN4_BIT);
  pcfWrite(value);
}

void motorsStop() { setMotorBits(false, false, false, false); }
void driveForward() { setMotorBits(true, false, true, false); }
void driveBackward(){ setMotorBits(false, true, false, true); }
void turnLeft()     { setMotorBits(false, true, true, false); }
void turnRight()    { setMotorBits(true, false, false, true); }

float readDistanceCm() {
  digitalWrite(SONAR_TRIG_PIN, LOW);
  delayMicroseconds(3);
  digitalWrite(SONAR_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_TRIG_PIN, LOW);
  unsigned long duration = pulseIn(SONAR_ECHO_PIN, HIGH, 25000UL);
  if (duration == 0) return 400.0f; // no echo = unknown/far; never treat as confirmed clear
  return duration * 0.0343f / 2.0f;
}

float scanAt(int angle) {
  scanServo.write(angle);
  delay(220);
  return readDistanceCm();
}

void avoidObstacleIfNeeded() {
  // Only perform automatic sonar intervention when a forward command is requested.
  if (requestedCommand != 'F') return;
  float front = readDistanceCm();
  // A missing echo is unknown, not proof that the path is clear.
  if (front >= 350.0f) {
    motorsStop();
    requestedCommand = 'S';
    return;
  }
  if (front >= OBSTACLE_CM) return;

  motorsStop();
  delay(100);
  float leftDistance = scanAt(145);
  float rightDistance = scanAt(35);
  scanServo.write(90);

  // If both readings are invalid/no echo, stop rather than guess.
  if (leftDistance > 350 && rightDistance > 350) {
    motorsStop();
    requestedCommand = 'S';
    return;
  }

  if (leftDistance >= rightDistance && leftDistance >= OBSTACLE_CM) {
    turnLeft();
    delay(TURN_PULSE_MS);
  } else if (rightDistance > leftDistance && rightDistance >= OBSTACLE_CM) {
    turnRight();
    delay(TURN_PULSE_MS);
  }
  motorsStop();
  requestedCommand = 'S'; // require a fresh AI command after avoidance
}

void applyCommand() {
  char cmd = requestedCommand;
  if (millis() - lastCommandMs > COMMAND_TIMEOUT_MS) cmd = 'S';

  switch (cmd) {
    case 'F': driveForward(); break;
    case 'B': driveBackward(); break;
    case 'L': turnLeft(); break;
    case 'R': turnRight(); break;
    default: motorsStop(); break;
  }
}

void handleRoot() {
  String html = R"HTML(
<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32-CAM Rover</title>
<style>body{font-family:system-ui;max-width:760px;margin:20px auto;padding:0 12px;background:#101820;color:#eef}
img{width:100%;max-width:640px;border-radius:12px}button{font-size:18px;padding:12px 18px;margin:4px}
</style></head><body><h2>AI Vision Rover</h2>
<p>Live camera stream. Buttons are manual test commands; stop the rover before lifting it.</p>
<img id="cam" alt="Camera stream">
<p>
<button onclick="cmd('F')">Forward</button><button onclick="cmd('B')">Reverse</button>
<button onclick="cmd('L')">Left</button><button onclick="cmd('R')">Right</button>
<button onclick="cmd('S')">STOP</button></p>
<p id="status">Connecting…</p>
<script>
const host=location.hostname;
document.getElementById('cam').src='http://'+host+':81/stream';
async function cmd(c){try{let r=await fetch('/drive?cmd='+c);document.getElementById('status').textContent=await r.text()}catch(e){document.getElementById('status').textContent='Command failed'}}
</script></body></html>)HTML";
  server.send(200, "text/html", html);
}

void handleDrive() {
  String s = server.arg("cmd");
  if (s.length() != 1 || String("FBLRS").indexOf(s[0]) < 0) {
    server.send(400, "text/plain", "Use cmd=F, B, L, R, or S");
    return;
  }
  requestedCommand = s[0];
  lastCommandMs = millis();
  if (requestedCommand == 'S') motorsStop();
  server.send(200, "text/plain", String("Command accepted: ") + requestedCommand);
}

void handleStatus() {
  String out = "{\"ip\":\"" + WiFi.localIP().toString() + "\",\"cmd\":\"";
  out += requestedCommand;
  out += "\",\"expander\":";
  out += (expanderOK ? "true" : "false");
  out += "}";
  server.send(200, "application/json", out);
}

void streamTask(void* parameter) {
  for (;;) {
    WiFiClient client = streamServer.available();
    if (!client) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

    String requestLine = client.readStringUntil('\r');
    client.readStringUntil('\n');
    while (client.connected()) {
      String line = client.readStringUntil('\n');
      if (line == "\r" || line.length() == 0) break;
    }

    if (requestLine.indexOf("GET /stream") < 0) {
      client.print("HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\n");
      client.stop();
      continue;
    }

    client.print("HTTP/1.1 200 OK\r\n");
    client.print("Content-Type: multipart/x-mixed-replace; boundary=frame\r\n");
    client.print("Access-Control-Allow-Origin: *\r\n");
    client.print("Cache-Control: no-cache\r\nConnection: close\r\n\r\n");

    while (client.connected()) {
      camera_fb_t* fb = esp_camera_fb_get();
      if (!fb) { vTaskDelay(pdMS_TO_TICKS(30)); continue; }
      client.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len);
      size_t written = client.write(fb->buf, fb->len);
      client.print("\r\n");
      esp_camera_fb_return(fb);
      if (written == 0) break;
      vTaskDelay(pdMS_TO_TICKS(35));
    }
    client.stop();
  }
}

bool startCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count = psramFound() ? 2 : 1;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  return esp_camera_init(&config) == ESP_OK;
}

void setup() {
  // Do not start Serial: GPIO3 is used for servo after programmer is disconnected.
  pinMode(SONAR_TRIG_PIN, OUTPUT);
  digitalWrite(SONAR_TRIG_PIN, LOW);
  pinMode(SONAR_ECHO_PIN, INPUT);

  scanServo.setPeriodHertz(50);
  scanServo.attach(SERVO_PIN, 500, 2400);
  scanServo.write(90);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 100000);
  Wire.beginTransmission(PCF_ADDR);
  expanderOK = (Wire.endTransmission() == 0);
  if (expanderOK) motorsStop();

  if (!startCamera()) {
    // Stop motors if camera setup fails; firmware remains available for diagnosis.
    motorsStop();
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 25000) {
    delay(300);
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/drive", HTTP_GET, handleDrive);
  server.on("/status", HTTP_GET, handleStatus);
  server.begin();

  streamServer.begin();
  xTaskCreatePinnedToCore(streamTask, "camera_stream", 8192, nullptr, 1, nullptr, 0);

  lastCommandMs = millis();
  requestedCommand = 'S';
}

void loop() {
  server.handleClient();

  if (!expanderOK) {
    motorsStop();
    delay(10);
    return;
  }

  if (millis() - lastSafetyCheck >= 100) {
    lastSafetyCheck = millis();
    if (requestedCommand == 'F') {
      avoidObstacleIfNeeded();
    }
    applyCommand();
  }
  delay(2);
}
