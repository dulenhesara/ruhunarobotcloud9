#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h> 
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h> 
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ======================= OLED LIBRARIES =======================
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h> 

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1 

Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ======================= WEB SERVER =======================
WebServer server(80);

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>8 Sensor Dashboard</title>
  <style>
    body { font-family: Arial; text-align: center; margin:0px auto; padding-top: 10px; background-color: #f4f4f4; }
    .card { background: white; padding: 20px; margin: 10px auto; width: 95%; max-width: 500px; box-shadow: 2px 2px 12px #aaa; border-radius: 8px; }
    input { padding: 5px; width: 60px; text-align: center; }
    table { width: 100%; }
    td { padding: 5px; }
    .btn { background-color: #008CBA; color: white; padding: 10px 20px; border: none; border-radius: 4px; cursor: pointer; font-size: 16px; }
    .btn:hover { background-color: #007399; }
    .sensor-box { display: inline-block; width: 40px; padding: 5px; margin: 2px; background: #ddd; border-radius: 5px; font-weight: bold; font-size: 12px;}
    .active { background-color: #ff4444; color: white; }
    .outer-sensor { background-color: #add8e6; } 
    .outer-active { background-color: #0000ff; color: white; }
    .row { margin-bottom: 10px; }
  </style>
</head>
<body>
  <h3>🤖 8-Sensor View</h3>
  <div class="card">
    <h4>Sensor Readings (ADC)</h4>
    <div class="row">
      <span style="margin-right:20px;">LEFT: <span id="sLeft" class="sensor-box outer-sensor">--</span></span>
      <span>FRONT: <span id="sFront" class="sensor-box outer-sensor">--</span></span>
      <span style="margin-left:20px;">RIGHT: <span id="sRight" class="sensor-box outer-sensor">--</span></span>
    </div>
    <div class="row">
      <span id="s1" class="sensor-box">S1</span>
      <span id="s2" class="sensor-box">S2</span>
      <span id="s3" class="sensor-box">S3</span>
      <span id="s4" class="sensor-box">S4</span>
      <span id="s5" class="sensor-box">S5</span>
    </div>
    <hr>
    <p>Ultrasonic: <span id="dist" style="font-weight:bold; color:blue;">--</span> cm</p>
    <p>Action: <span id="act" style="font-weight:bold; color:green;">--</span></p>
  </div>
  <div class="card">
    <h3>Tuning</h3>
    <form action="/update" target="hidden-frame">
      <table>
        <tr><td>Base Speed:</td><td><input type="number" name="baseSpeed" id="baseSpeed"></td></tr>
        <tr><td>Turn Speed:</td><td><input type="number" name="turnSpeed" id="turnSpeed"></td></tr>
        <tr><td>Target 90:</td><td><input type="number" name="target90" id="target90"></td></tr>
        <tr><td>Kp:</td><td><input type="number" step="0.1" name="Kp" id="Kp"></td></tr>
        <tr><td>Kd:</td><td><input type="number" step="0.1" name="Kd" id="Kd"></td></tr>
      </table>
      <br>
      <input type="submit" value="Update" class="btn">
    </form>
    <iframe name="hidden-frame" style="display:none;"></iframe>
  </div>
<script>
  fetch('/settings').then(response => response.json()).then(data => {
    document.getElementById('baseSpeed').value = data.baseSpeed;
    document.getElementById('turnSpeed').value = data.turnSpeed;
    document.getElementById('target90').value = data.target90;
    document.getElementById('Kp').value = data.Kp;
    document.getElementById('Kd').value = data.Kd;
  });
  setInterval(function() {
    fetch('/data').then(response => response.json()).then(data => {
      let thresh = 2000; 
      for(let i=1; i<=5; i++){
         let val = data["s"+i];
         let el = document.getElementById("s"+i);
         el.innerText = val;
         el.className = val < thresh ? "sensor-box active" : "sensor-box";
      }
      document.getElementById('sFront').innerText = data.sFront;
      document.getElementById('sLeft').innerText = data.sLeft;
      document.getElementById('sRight').innerText = data.sRight;
      document.getElementById('sFront').className = data.sFront > thresh ? "sensor-box outer-active" : "sensor-box outer-sensor";
      document.getElementById('sLeft').className = data.sLeft > thresh ? "sensor-box outer-active" : "sensor-box outer-sensor";
      document.getElementById('sRight').className = data.sRight > thresh ? "sensor-box outer-active" : "sensor-box outer-sensor";
      document.getElementById('dist').innerText = data.dist;
      document.getElementById('act').innerText = data.act;
    });
  }, 300);
</script>
</body>
</html>
)rawliteral";

// ======================= PCA9685 SERVO SETUP =======================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  150 
#define SERVOMAX  600 
#define SERVO_FREQ 50 

const int SRV_CH_ARM  = 13;
const int SRV_CH_TILT = 14;
const int SRV_CH_GRIP = 15;

int boxCount = 0;        
unsigned long lastDisplayUpdate = 0;

// Servo Positions
int LIFT_UP = 134;     
int LIFT_DOWN = 0;      
int TILT_FORWARD = 57;   
int TILT_BACK = 0;      
int GRIP_OPEN = 0;       
int GRIP_CLOSE = 167;    

int currentArmPos = LIFT_UP;
int currentTiltPos = TILT_BACK;
int currentGripPos = GRIP_OPEN;

// --- TCS230 Color Sensor ---
const int S2 = 17;
const int S3 = 27;
const int COLOR_OUT = 33;

// ======================= PIN SETUP =======================
const int LEFT_ENC_PIN = 25;
const int RIGHT_ENC_PIN = 26;
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// ======================= MUX SETUP =======================
const int MUX_S0 = 18;
const int MUX_S1 = 19;
const int MUX_S2 = 23;
const int MUX_SIG = 35; 

// Mux Channel Mapping
const int CH_S1 = 0;
const int CH_S2 = 1;
const int CH_S3 = 2;
const int CH_S4 = 3;
const int CH_S5 = 4;
const int CH_FRONT = 5;
const int CH_LEFT = 6;
const int CH_RIGHT = 7;

// --- Motors ---
const int IN1 = 14; const int IN2 = 12;
const int IN3 = 13; const int IN4 = 32;
const int ENA = 2;  const int ENB = 4;

int firstcolor;
int secondcolor;
bool boxHandled = false; 

// Ultrasonic & Limit Switch
const int trigPin = 5;            
const int echoPin = 34;
const int limitSwitchPin = 39; 

// ======================= VARIABLES =======================
float Kp = 300.0;
float Ki = 0.0;
float Kd = 200.0;
float setPoint = 2;
float previousError = 0;
float integral = 0;

int baseSpeed = 100;
int turnSpeed = 150;
int target90 = 150; 
int alignTime = 80; 

int sensorValues[8]; 
int sensorMin[8]; 
int sensorMax[8]; 

int threshold = 2000;
long currentUltrasonic = 0;
String currentAction = "Idle";
bool insideCircle = false;
bool startZoneCleared = false; // Added for start logic

// ======================= INTERRUPTS =======================
void IRAM_ATTR leftEncoderISR() { leftEncoderCount++; }
void IRAM_ATTR rightEncoderISR() { rightEncoderCount++; }

// ======================= PROTOTYPES =======================
void setupPins();
void setupServosFunc();
void setServoAngle(int channel, int angle); 
void moveServoSmooth(int channel, int &currentPos, int targetPos, int speedDelay);
void calibrateSensors();
void waitForStartButton(); // NEW PROTOTYPE
void readSensors(); 
void motor(int left, int right);
char follow_line();
char identifyJunction();
void turn_left();
void turn_right();
void turn_around();
void boxPickPlaceTask();
bool isBoxDetected();
void pickUpBox();
bool isBoxGripped();
long readUltrasonic();
void handleRoot();
void handleUpdate();
void handleData();
void handleSettings();
int readMux(int channel);
void updateOLED(); 
int detectBoxColor(); 
void GetOnRoboat();
void HoldArm();
void alinarm();
void adjustDistance();
void handleStartZone(); 

// ======================= SETUP =======================
void setup() {
  Serial.begin(115200);
  delay(100); 
  
  if(!display.begin(0x3C, true)) { 
    Serial.println(F("SH110X allocation failed"));
  } else {
    display.setRotation(2); 
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0,0);
    display.println("WIFI AP...");
    display.display();
  }
  
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 

  WiFi.softAP("ESP32_Robot", "12345678");
  server.on("/", handleRoot);
  server.on("/update", handleUpdate);
  server.on("/data", handleData);
  server.on("/settings", handleSettings);
  server.begin();

  setupPins();
  setupServosFunc();   
  
  // --- 1. SENSOR CALIBRATION (Stops on 1st Click) ---
  calibrateSensors(); 

  // --- 2. WAIT FOR START (Starts on 2nd Click) ---
  waitForStartButton();

  // --- 3. SERVO INIT ---
  setServoAngle(SRV_CH_ARM, 134);     
  setServoAngle(SRV_CH_TILT, 60);  
  setServoAngle(SRV_CH_GRIP, GRIP_OPEN);  
}

// ======================= MAIN LOOP =======================
void loop() {
  if (millis() - lastDisplayUpdate > 200) {
       updateOLED();
       lastDisplayUpdate = millis();
  }
  server.handleClient(); 

  // --- START ZONE CHECK ---
  if (!startZoneCleared) {
    handleStartZone();    
    startZoneCleared = true; 
    return;
  }
  // ------------------------

  char state = follow_line();
  currentAction = String(state); 
  currentUltrasonic = readUltrasonic(); 

  if (state == 'B') {
    currentAction = "BOX FOUND";
    motor(0, 0); delay(100);
    boxPickPlaceTask();
    return;
  }
  
  if (state == 'S') {
       currentAction = "CIRCLE";
       if (!insideCircle) {
           motor(baseSpeed, baseSpeed); delay(350); insideCircle = true; 
       } else {
           motor(baseSpeed, baseSpeed); delay(200); insideCircle = false; 
       }
       state = 'F'; 
  }

  if (state == 'F') return; 

  motor(0, 0); delay(100);
  if (state == 'D') { currentAction = "Turn Back"; turn_around(); }
  else if (state == 'L') { currentAction = "Left"; turn_left(); }
  else if (state == 'R') { currentAction = "Right"; turn_right(); }
  else if (state == 'C') { currentAction = "Cross"; turn_left(); }
  else if (state == 'Y') { currentAction = "Y-Junc"; turn_left(); }
  else if (state == 'T') { currentAction = "T-Junc"; turn_left(); }
  
  motor(0, 0); delay(100);
}

// ======================= CALIBRATION & WAIT =======================
void calibrateSensors() {
  display.clearDisplay(); display.setCursor(0,0);
  display.println("CALIBRATING"); 
  display.setTextSize(1);
  display.println("Press BOOT to Stop");
  display.display();

  for (int i = 0; i < 8; i++) { sensorMin[i] = 4095; sensorMax[i] = 0; }
  
  // Rotate to scan line
  motor(60, -60); 

  // Calibrate until button is pressed (LOW)
  while (digitalRead(0) == HIGH) { 
    readSensors();
    for (int i = 0; i < 8; i++) {
      sensorMin[i] = min(sensorMin[i], sensorValues[i]);
      sensorMax[i] = max(sensorMax[i], sensorValues[i]);
    }
    server.handleClient();
    delay(5);
  }

  // Button pressed! Stop motors.
  motor(0, 0); 

  // WAIT FOR BUTTON RELEASE (Debounce)
  while(digitalRead(0) == LOW) { delay(10); }

  // Calculate threshold
  threshold = 0;
  for (int i = 0; i < 5; i++) threshold += (sensorMin[i] + sensorMax[i]) / 2;
  threshold /= 5;

  display.clearDisplay(); display.setCursor(0,0);
  display.setTextSize(2);
  display.print("Thresh: "); display.println(threshold); display.display();
  delay(1000);
}

void waitForStartButton() {
  display.clearDisplay(); 
  display.setCursor(0,0);
  display.setTextSize(2);
  display.println("READY!");
  display.setTextSize(1);
  display.println("");
  display.println("Press BOOT to Start");
  display.display();

  // Wait until button is pressed (LOW)
  while (digitalRead(0) == HIGH) {
    server.handleClient(); // Keep WiFi working while waiting
    delay(10);
  }

  // Button pressed! 
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(2);
  display.println("GO!");
  display.display();
  delay(500); // Small delay before moving
}

// ======================= START ZONE LOGIC =======================
void handleStartZone() {
  Serial.println("Action: Crossing Start Zone (Sensor Mode)...");
  currentAction = "START ZONE";
  updateOLED();

  // 1. Start Moving Forward
  motor(baseSpeed, baseSpeed); 

  // 2. Loop until we see a "Normal Line"
  while (true) {
    readSensors();       
    server.handleClient(); 

    // Value < threshold means BLACK line
    bool s1_is_White = (sensorValues[0] > threshold);
    bool s3_is_Black = (sensorValues[2] < threshold); // Center
    bool s5_is_White = (sensorValues[4] > threshold);

    // Stop ONLY when Center is Black AND at least one side is White
    if (s3_is_Black && (s1_is_White || s5_is_White)) {
       Serial.println("Normal Line Detected!");
       break; 
    }
  }

  motor(0, 0);
  delay(100); 
  currentAction = "Start Cleared";
  updateOLED();
}

// ======================= OLED =======================
void updateOLED() {
  display.clearDisplay();
  display.setTextSize(2); 
  display.setTextColor(SH110X_WHITE); 
  display.setCursor(0, 0); 
  display.println(currentAction);
  display.setTextSize(1);
  display.print("IP: 192.168.4.1");
  display.display();
}

// ======================= SENSORS & WEB =======================
int readMux(int channel) {
  digitalWrite(MUX_S0, (channel & 1));      
  digitalWrite(MUX_S1, (channel & 2) >> 1); 
  digitalWrite(MUX_S2, (channel & 4) >> 2); 
  delayMicroseconds(10); 
  return analogRead(MUX_SIG);
}

void readSensors() {
  for(int i=0; i<8; i++) { sensorValues[i] = readMux(i); }
}

void handleRoot() { server.send(200, "text/html", index_html); }
void handleSettings() {
  String json = "{";
  json += "\"baseSpeed\":" + String(baseSpeed) + ",";
  json += "\"turnSpeed\":" + String(turnSpeed) + ",";
  json += "\"target90\":" + String(target90) + ",";
  json += "\"Kp\":" + String(Kp) + ",";
  json += "\"Kd\":" + String(Kd) + "}";
  server.send(200, "application/json", json);
}
void handleUpdate() {
  if (server.hasArg("baseSpeed")) baseSpeed = server.arg("baseSpeed").toInt();
  if (server.hasArg("turnSpeed")) turnSpeed = server.arg("turnSpeed").toInt();
  if (server.hasArg("target90")) target90 = server.arg("target90").toInt();
  if (server.hasArg("Kp")) Kp = server.arg("Kp").toFloat();
  if (server.hasArg("Kd")) Kd = server.arg("Kd").toFloat();
  server.send(200, "text/plain", "OK");
}
void handleData() {
  readSensors(); 
  String json = "{";
  for(int i=0; i<5; i++) json += "\"s" + String(i+1) + "\":" + String(sensorValues[i]) + ",";
  json += "\"sFront\":" + String(sensorValues[CH_FRONT]) + ",";
  json += "\"sLeft\":" + String(sensorValues[CH_LEFT]) + ",";
  json += "\"sRight\":" + String(sensorValues[CH_RIGHT]) + ",";
  json += "\"dist\":" + String(currentUltrasonic) + ",";
  json += "\"act\":\"" + currentAction + "\"}";
  server.send(200, "application/json", json);
}

// ======================= MOVEMENT & TURNS =======================
void motor(int left, int right) {
   if (left > 0) { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); } 
   else { left = -left; digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
   if (right > 0) { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); } 
   else { right = -right; digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
   analogWrite(ENB, constrain(left, 0, 255));
   analogWrite(ENA, constrain(right, 0, 255));
}

void turn_left() {
  leftEncoderCount = 0; rightEncoderCount = 0;
  motor(turnSpeed, -turnSpeed); 
  while (rightEncoderCount < (target90 * 0.7)) { server.handleClient(); }
  motor(turnSpeed * 0.8, -turnSpeed * 0.8); 
  unsigned long searchStart = millis();
  while (readMux(CH_S3) > threshold) {
     if(millis() - searchStart > 2000) break; 
     server.handleClient(); 
  }
  motor(0,0); delay(200); 
  previousError = 0; integral = 0;
}

void turn_right() {
  leftEncoderCount = 0; rightEncoderCount = 0;
  motor(-turnSpeed, turnSpeed); 
  while (leftEncoderCount < (target90 * 0.7)) { server.handleClient(); }
  motor(-turnSpeed * 0.8, turnSpeed * 0.8);
  unsigned long searchStart = millis();
  while (readMux(CH_S3) > threshold) {
     if(millis() - searchStart > 2000) break;
     server.handleClient();
  }
  motor(0,0); delay(200);
  previousError = 0; integral = 0;
}

void turn_around() {
  leftEncoderCount = 0; rightEncoderCount = 0;
  int blindTarget = (target90 * 2.2) * 0.85; 
  motor(-turnSpeed, turnSpeed); 
  while (rightEncoderCount < blindTarget) { server.handleClient(); }
  unsigned long searchStart = millis();
  while (readMux(CH_S3) > threshold) {
     if(millis() - searchStart > 3000) break;
     server.handleClient();
  }
  motor(0,0); delay(200);
  previousError = 0; integral = 0;
}

// ======================= LOGIC =======================
char follow_line() {
  readSensors();
  if (isBoxDetected()) { return 'B'; }

  char j = identifyJunction();
  if (j != 'F') return j;

  int digitalVal[5];
  int total = 0;
  long posSum = 0;

  for (int i = 0; i < 5; i++) {
    digitalVal[i] = (sensorValues[i] < threshold) ? 1 : 0;
    posSum += digitalVal[i] * i;
    total += digitalVal[i];
  }

  if (total == 0) return 'D';

  float avg = (float)posSum / total;
  float error = setPoint - avg;
  float P = Kp * error;
  float D = Kd * (error - previousError);
  previousError = error;
  float PID_value = P + D;

  motor(baseSpeed + PID_value, baseSpeed - PID_value);
  return 'F';
}

char identifyJunction() {
  bool m1 = (sensorValues[0] < threshold); 
  bool m2 = (sensorValues[1] < threshold); 
  bool m3 = (sensorValues[2] < threshold); 
  bool m4 = (sensorValues[3] < threshold); 
  bool m5 = (sensorValues[4] < threshold); 
  
  bool outerLeftIsBlack = (sensorValues[CH_LEFT] > threshold); 
  bool outerRightIsBlack = (sensorValues[CH_RIGHT] > threshold);
  bool outers_frontIsBlack = (sensorValues[CH_FRONT] > threshold);

  if (outerLeftIsBlack && outerRightIsBlack) {
      motor(baseSpeed, baseSpeed); delay(alignTime); motor(0,0);
      readSensors(); 
      if((sensorValues[2] < threshold)) return 'C'; 
      else return 'T';   
  }
  else if (m2 && m3 && m4 && !outerLeftIsBlack && !outerRightIsBlack) {
      motor(baseSpeed, baseSpeed); delay(alignTime); motor(0,0);
      readSensors(); 
      if (sensorValues[2] < threshold) return 'S'; 
      else {
          motor(-baseSpeed, -baseSpeed); delay(alignTime); motor(0,0);
          return 'F'; 
      }
  }
  else if (m1) {
     motor(baseSpeed, baseSpeed); delay(alignTime); motor(0,0);
     readSensors();
     if (sensorValues[2] < threshold || sensorValues[1] < threshold || sensorValues[3] < threshold) return 'Y'; 
     else return 'L';           
  } 
  else  if (m5 && outers_frontIsBlack) { 
      motor(baseSpeed, baseSpeed); delay(alignTime); motor(0,0); 
      readSensors();
      if((sensorValues[0] < threshold)){ return 'L'; }
      return 'F';
  }
  else  if (m5) {
     motor(baseSpeed, baseSpeed); delay(alignTime); motor(0,0); 
     readSensors();
     if((sensorValues[0] < threshold)) { return 'L'; }
     else  { return 'R';   }
  }  
  else if (!m1 && !m2 && !m3 && !m4 && !m5 && !outerLeftIsBlack && !outerRightIsBlack ) {
    return 'D';
  } else {
    return 'F'; 
  }
}

// ======================= ALIGNMENT & DISTANCE =======================

void adjustDistance() {
  const int TARGET_DIST = 11.5; 
  const int TOLERANCE = 1;    
  const int ADJUST_SPEED = 80;

  long currentDist = readUltrasonic();

  while (currentDist < (TARGET_DIST - TOLERANCE) || currentDist > (TARGET_DIST + TOLERANCE)) {
    if (currentDist > 30 || currentDist == 0) { 
        Serial.println("Adjust distance failed (lost box).");
        motor(0, 0);
        return; 
    }
    
    if (currentDist < (TARGET_DIST - TOLERANCE)) {
      currentAction = "ADJ BACK";
      motor(-ADJUST_SPEED, -ADJUST_SPEED); 
    } 
    else if (currentDist > (TARGET_DIST + TOLERANCE)) {
      currentAction = "ADJ FWD";
      motor(ADJUST_SPEED, ADJUST_SPEED); 
    }
    
    updateOLED(); 
    delay(50); 
    currentDist = readUltrasonic();
    server.handleClient();
  }

  motor(0, 0); 
  currentAction = "ALIGNED";
  updateOLED();
  delay(100);
}

void alinarm() {
  Serial.println("Action: Aligning with Box Center...");
  updateOLED(); 

  long dist = readUltrasonic();
  if (dist > 15 || dist == 0) return; 

  motor(-110,110); 
  
  unsigned long startTime = millis();
  while (true) {
    dist = readUltrasonic();
    if (dist > 20 || millis() - startTime > 2000) { break; }
    delay(10);
  }
  
  motor(0, 0); 
  delay(200);

  leftEncoderCount = 0;
  int centerOffsetTicks = 80; 
  motor(110, -110); 
  
  while (leftEncoderCount < centerOffsetTicks) {
  }
  motor(0, 0); 
  delay(300);
  
  adjustDistance(); 
}

// ======================= BOX TASKS =======================

void boxPickPlaceTask() {
  motor(0,0); delay(200);
  alinarm(); 
  pickUpBox();
  
  if(isBoxGripped()) {
    boxCount++; 
    currentAction = "Box: " + String(boxCount);
    updateOLED(); 
    if(boxCount == 1) { GetOnRoboat(); }
    else if (boxCount == 2) { HoldArm(); }
  } else {
    Serial.println("Failed to grip box. Resetting arm.");
    moveServoSmooth(SRV_CH_GRIP, currentGripPos, GRIP_OPEN, 10);
    moveServoSmooth(SRV_CH_ARM, currentArmPos, LIFT_UP, 10);
  }
  delay(500); 
}

void GetOnRoboat(){
  moveServoSmooth(SRV_CH_ARM, currentArmPos, 134, 20); 
  delay(200);
  moveServoSmooth(SRV_CH_TILT, currentTiltPos, 35, 15);
  delay(200);
  firstcolor = detectBoxColor(); 
  currentAction = (firstcolor == 1) ? "RED" : "BLUE";
  updateOLED(); delay(2000); 
  moveServoSmooth(SRV_CH_GRIP, currentGripPos, 0, 10);
}

void HoldArm(){
  secondcolor = detectBoxColor(); 
  currentAction = (secondcolor == 1) ? "RED" : "BLUE";
  updateOLED(); delay(2000); 
  moveServoSmooth(SRV_CH_ARM, currentArmPos, 90, 20); 
  delay(200);
}

void pickUpBox() {
    Serial.println("Action: Picking up box with feedback...");
    moveServoSmooth(SRV_CH_GRIP, currentGripPos, 0, 10); // GRIP_OPEN
    moveServoSmooth(SRV_CH_TILT, currentTiltPos, 57, 15); // TILT_FORWARD
    delay(200);
    moveServoSmooth(SRV_CH_ARM, currentArmPos, 0, 20); // LIFT_DOWN
    delay(500); 

    Serial.println("Closing grip until limit switch...");
    const int GRIP_CLOSE_MAX = 140; 
    for (int angle = 20; angle <= GRIP_CLOSE_MAX; angle++) {
        setServoAngle(SRV_CH_GRIP, angle);
        currentGripPos = angle; 
        delay(15); 
        if (digitalRead(limitSwitchPin) == LOW) { break; }
    }
    delay(500); 
}

// ======================= HELPER FUNCTIONS =======================
bool isBoxDetected() {
  if (boxCount >= 2) return false;
  long distance = readUltrasonic();
  return (distance > 1 && distance < 11);
}

bool isBoxGripped() { return digitalRead(limitSwitchPin) == LOW; }

int detectBoxColor() { 
  digitalWrite(S2, LOW); digitalWrite(S3, LOW);
  int redPW = pulseIn(COLOR_OUT, LOW); delay(10);
  digitalWrite(S2, LOW); digitalWrite(S3, HIGH);
  int bluePW = pulseIn(COLOR_OUT, LOW); delay(10);
  return (redPW < bluePW) ? 1 : 2; 
} 

long readUltrasonic() {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000); 
  if (duration == 0) return 999; 
  return duration * 0.034 / 2; 
}

void setupPins() {
  pinMode(0, INPUT_PULLUP); 
  pinMode(LEFT_ENC_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN), rightEncoderISR, RISING);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(MUX_S0, OUTPUT); pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT); pinMode(MUX_SIG, INPUT); 
  pinMode(trigPin, OUTPUT); pinMode(echoPin, INPUT);
  pinMode(limitSwitchPin, INPUT); 
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT); pinMode(COLOR_OUT, INPUT);
}

void setupServosFunc() {
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ); 
  delay(10);
}

void setServoAngle(int channel, int angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulse);
}

void moveServoSmooth(int channel, int &currentPos, int targetPos, int speedDelay) {
  if (currentPos < targetPos) {
    for (int i = currentPos; i <= targetPos; i++) {
      setServoAngle(channel, i); delay(speedDelay);
    }
  } else {
    for (int i = currentPos; i >= targetPos; i--) {
      setServoAngle(channel, i); delay(speedDelay);
    }
  }
  currentPos = targetPos; 
}