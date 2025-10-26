/*
 * Arduino Uno Firmware - LogisticsBot Controller
 * 
 * Handles:
 * - L298N Motor Control (PWM + Direction)
 * - 5x IR Line Sensors (Fallback mode when camera fails)
 * - HC-SR04 Ultrasonic Sensor
 * - UART Communication with Raspberry Pi
 * 
 * Communication: JSON protocol over Serial (115200 baud)
 */

#include <ArduinoJson.h>

// ===== MOTOR PIN DEFINITIONS =====
// Left Motor
#define ENA 9   // PWM Left Motor Speed
#define IN1 2   // Left Motor Direction 1
#define IN2 3   // Left Motor Direction 2

// Right Motor
#define ENB 10  // PWM Right Motor Speed
#define IN3 4   // Right Motor Direction 1
#define IN4 5   // Right Motor Direction 2

// ===== SENSOR PIN DEFINITIONS =====
// HC-SR04 Ultrasonic Sensor
#define ULTRASONIC_TRIG  12
#define ULTRASONIC_ECHO  11

// IR Line Sensors (5 sensors) - UPDATED
#define LINE_SENSOR_1   A0  // Far Left
#define LINE_SENSOR_2   A1  // Left
#define LINE_SENSOR_3   A2  // Center
#define LINE_SENSOR_4   A3  // Right
#define LINE_SENSOR_5   A4  // Far Right

// IR Sensor threshold (adjustable)
#define IR_THRESHOLD 512  // ADC value 0-1023 (black > threshold > white)

// ===== GLOBAL VARIABLES =====
int leftSpeed = 0;
int rightSpeed = 0;
bool lineSensors[5] = {0};  // Changed from 8 to 5
int linePosition = 0;
float distance = 0.0;

// Sensor mode
bool irSensorEnabled = false;  // Default: disabled (camera primary)

unsigned long lastSensorRead = 0;
unsigned long sensorReadInterval = 50; // Read sensors every 50ms

// ===== SETUP =====
void setup() {
  // Initialize Serial Communication
  Serial.begin(115200);
  
  // Motor pins - LEFT MOTOR
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  // Motor pins - RIGHT MOTOR
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Ultrasonic sensor pins
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
  
  // Line sensor pins (5 sensors only)
  pinMode(LINE_SENSOR_1, INPUT);
  pinMode(LINE_SENSOR_2, INPUT);
  pinMode(LINE_SENSOR_3, INPUT);
  pinMode(LINE_SENSOR_4, INPUT);
  pinMode(LINE_SENSOR_5, INPUT);
  
  // Stop motors on startup
  stopMotors();
  
  // Send ready signal
  Serial.println("{\"status\":\"ready\",\"device\":\"arduino_uno\",\"ir_sensors\":5}");
}

// ===== MAIN LOOP =====
void loop() {
  // Read sensors periodically
  if (millis() - lastSensorRead >= sensorReadInterval) {
    readSensors();
    sendSensorData();
    lastSensorRead = millis();
  }
  
  // Process incoming commands
  if (Serial.available() > 0) {
    processCommand();
  }
}

// ===== COMMAND PROCESSING =====
void processCommand() {
  String json = Serial.readStringUntil('\n');
  json.trim();
  
  if (json.length() == 0) return;
  
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, json);
  
  if (error) {
    sendError("JSON parse error");
    return;
  }
  
  const char* cmd = doc["cmd"];
  
  if (strcmp(cmd, "MOVE") == 0) {
    int left = doc["left"] | 0;
    int right = doc["right"] | 0;
    setMotors(left, right);
    sendAck("MOVE");
  }
  else if (strcmp(cmd, "STOP") == 0) {
    stopMotors();
    sendAck("STOP");
  }
  else if (strcmp(cmd, "SET_SPEED") == 0) {
    int speed = doc["value"] | 0;
    sendAck("SET_SPEED");
  }
  else if (strcmp(cmd, "ENABLE_IR") == 0) {
    // Enable IR sensors (fallback mode)
    irSensorEnabled = true;
    sendAck("IR_ENABLED");
  }
  else if (strcmp(cmd, "DISABLE_IR") == 0) {
    // Disable IR sensors (camera mode)
    irSensorEnabled = false;
    sendAck("IR_DISABLED");
  }
  else if (strcmp(cmd, "GET_SENSORS") == 0) {
    sendSensorData();
  }
  else if (strcmp(cmd, "PING") == 0) {
    sendAck("PONG");
  }
  else {
    sendError("Unknown command");
  }
}

// ===== MOTOR CONTROL =====
void setMotors(int left, int right) {
  leftSpeed = constrain(left, -255, 255);
  rightSpeed = constrain(right, -255, 255);
  
  // ===== LEFT MOTOR CONTROL =====
  if (leftSpeed > 0) {
    // Forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, leftSpeed);
  }
  else if (leftSpeed < 0) {
    // Backward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -leftSpeed);
  }
  else {
    // Stop
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
  
  // ===== RIGHT MOTOR CONTROL =====
  if (rightSpeed > 0) {
    // Forward
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, rightSpeed);
  }
  else if (rightSpeed < 0) {
    // Backward
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -rightSpeed);
  }
  else {
    // Stop
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

void stopMotors() {
  leftSpeed = 0;
  rightSpeed = 0;
  
  // Stop left motor
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  
  // Stop right motor
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

// ===== SENSOR READING =====
void readSensors() {
  // Read line sensors (5 sensors only)
  // Only read if IR sensors are enabled
  if (irSensorEnabled) {
    lineSensors[0] = (analogRead(LINE_SENSOR_1) > IR_THRESHOLD) ? 1 : 0;
    lineSensors[1] = (analogRead(LINE_SENSOR_2) > IR_THRESHOLD) ? 1 : 0;
    lineSensors[2] = (analogRead(LINE_SENSOR_3) > IR_THRESHOLD) ? 1 : 0;
    lineSensors[3] = (analogRead(LINE_SENSOR_4) > IR_THRESHOLD) ? 1 : 0;
    lineSensors[4] = (analogRead(LINE_SENSOR_5) > IR_THRESHOLD) ? 1 : 0;
    
    // Calculate line position (-2 to +2)
    // Weights: -2, -1, 0, 1, 2
    int sum = 0;
    int count = 0;
    for (int i = 0; i < 5; i++) {
      if (lineSensors[i]) {
        sum += (i - 2);  // Position: -2, -1, 0, 1, 2
        count++;
      }
    }
    linePosition = (count > 0) ? sum : 0;
  } else {
    // IR sensors disabled, set all to 0
    for (int i = 0; i < 5; i++) {
      lineSensors[i] = 0;
    }
    linePosition = 0;
  }
  
  // Read ultrasonic distance (always enabled)
  distance = readUltrasonic();
}

float readUltrasonic() {
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  
  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000); // Timeout 30ms
  if (duration == 0) {
    return 999.0; // Max distance or no echo
  }
  
  float dist = duration * 0.034 / 2.0; // Convert to cm
  return constrain(dist, 2.0, 400.0);
}

// ===== COMMUNICATION =====
void sendSensorData() {
  StaticJsonDocument<300> doc;
  
  // Line sensors array (5 sensors)
  JsonArray lineArray = doc.createNestedArray("line");
  for (int i = 0; i < 5; i++) {
    lineArray.add(lineSensors[i]);
  }
  
  doc["line_pos"] = linePosition;
  doc["distance"] = round(distance * 10) / 10.0; // Round to 1 decimal
  doc["left_speed"] = leftSpeed;
  doc["right_speed"] = rightSpeed;
  doc["uptime"] = millis();
  doc["ir_enabled"] = irSensorEnabled;  // NEW: Report IR sensor status
  
  serializeJson(doc, Serial);
  Serial.println();
}

void sendAck(const char* command) {
  StaticJsonDocument<100> doc;
  doc["status"] = "ok";
  doc["cmd"] = command;
  serializeJson(doc, Serial);
  Serial.println();
}

void sendError(const char* message) {
  StaticJsonDocument<100> doc;
  doc["status"] = "error";
  doc["message"] = message;
  serializeJson(doc, Serial);
  Serial.println();
}
