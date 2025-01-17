#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <Adafruit_AHTX0.h>

#define EEPROM_VERSION_ADDR 167 
#define CURRENT_VERSION 2 

#define EEPROM_SIZE 512
#define SSID_ADDR 0
#define PASS_ADDR 64
#define SSID_MAX_LENGTH 32
#define PASS_MAX_LENGTH 64

#define MQ2_PIN A0
#define SENSOR_UPDATE_INTERVAL 60000

#define BUZZER_PIN_POS D5
#define BUZZER_PIN_NEG D7
#define TEMP_THRESHOLD 50.0  // Celsius
#define GAS_THRESHOLD 700    // Analog value
Adafruit_AHTX0 aht;
WebSocketsClient webSocket;
WiFiUDP udp;
const unsigned int udpPort = 4210;
char incomingPacket[255];
WiFiClientSecure client;

String WS_HOST = "homeconnect-api-ws-3a22df0bbfef.herokuapp.com";
uint16_t WS_PORT = 443;
String deviceId = "6";
String WS_PATH = "/?deviceId=" + deviceId;

const char* apSSID = "FIRE ALARM - 6";
const char* apPassword = "12345678";

unsigned long lastSensorUpdate = 0;
unsigned long lastPongTime = 0;
const unsigned long PING_INTERVAL = 25000;
const unsigned long PONG_TIMEOUT = 35000;
const unsigned long WIFI_TIMEOUT = 20000;

void readWiFiCredentials(String &ssid, String &password) {
  for (int i = 0; i < SSID_MAX_LENGTH; i++) {
    char c = EEPROM.read(SSID_ADDR + i);
    if (c == '\0') break;
    ssid += c;
  }
  
  for (int i = 0; i < PASS_MAX_LENGTH; i++) {
    char c = EEPROM.read(PASS_ADDR + i);
    if (c == '\0') break;
    password += c;
  }
}

void saveWiFiCredentials(const String &ssid, const String &password) {
  for (int i = 0; i < SSID_MAX_LENGTH; i++) {
    EEPROM.write(SSID_ADDR + i, 0);
  }
  for (int i = 0; i < PASS_MAX_LENGTH; i++) {
    EEPROM.write(PASS_ADDR + i, 0);
  }

  for (int i = 0; i < ssid.length(); i++) {
    EEPROM.write(SSID_ADDR + i, ssid[i]);
  }
  for (int i = 0; i < password.length(); i++) {
    EEPROM.write(PASS_ADDR + i, password[i]);
  }
  
  EEPROM.commit();
}

void resetEEPROM() {
  for (int i = 0; i < EEPROM_SIZE; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.write(EEPROM_VERSION_ADDR, CURRENT_VERSION);
  EEPROM.commit();
}

void setupAccessPoint() {
  WiFi.softAP(apSSID, apPassword);
  Serial.println("[AP] Access Point created:");
  Serial.println("SSID: " + String(apSSID));
  Serial.println("Password: " + String(apPassword));
}

void setupUDP() {
  udp.begin(udpPort);
  Serial.println("[UDP] Listening on port " + String(udpPort));
}

void handleUDPMessage() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = '\0';
      String receivedData = String(incomingPacket);
      String ssid = parseValue(receivedData, "SSID");
      String password = parseValue(receivedData, "PASSWORD");
      if (!ssid.isEmpty() && !password.isEmpty()) {
        if (connectToWiFi(ssid, password)) {
          startWebSocket();
        }
      }
    }
  }
}

String parseValue(String data, String key) {
  int startIndex = data.indexOf(key + "=");
  if (startIndex == -1) return "";
  startIndex += key.length() + 1;
  int endIndex = data.indexOf(";", startIndex);
  return endIndex == -1 ? "" : data.substring(startIndex, endIndex);
}

void sendSensorData() {
  if (millis() - lastSensorUpdate < SENSOR_UPDATE_INTERVAL) return;
  
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);
  int gasValue = analogRead(MQ2_PIN);

  DynamicJsonDocument doc(256);
  doc["type"] = "sensorData";
  doc["gas"] = gasValue;
  doc["temperature"] = temp.temperature;
  doc["humidity"] = humidity.relative_humidity;

  String jsonString;
  serializeJson(doc, jsonString);
  webSocket.sendTXT(jsonString);
  
  lastSensorUpdate = millis();
}

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("[WS] Disconnected");
      break;

    case WStype_CONNECTED:
      Serial.println("[WS] Connected to server");
      lastPongTime = millis();
      break;

    case WStype_PING:
    case WStype_PONG:
      lastPongTime = millis();
      break;

    default:
      break;
  }
}

bool connectToWiFi(const String &ssid, const String &password) {
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.disconnect();
  }
  
  Serial.println("[WiFi] Connecting to: " + ssid);
  WiFi.begin(ssid.c_str(), password.c_str());
  
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - startTime > WIFI_TIMEOUT) {
      Serial.println("[WiFi] Connection failed");
      return false;
    }
    delay(500);
  }
  
  Serial.println("[WiFi] Connected successfully");
  saveWiFiCredentials(ssid, password);
  return true;
}

void startWebSocket() {
  webSocket.onEvent(webSocketEvent);
  webSocket.beginSSL(WS_HOST.c_str(), WS_PORT, WS_PATH.c_str());
  webSocket.enableHeartbeat(10000, 2000, 2);
  webSocket.setReconnectInterval(5000);

  unsigned long startTime = millis();
  while (!webSocket.isConnected() && millis() - startTime < 10000) {
    webSocket.loop();
    Serial.println("[WS] Attempting to connect...");
    delay(500);
  }

  if (webSocket.isConnected()) {
    Serial.println("[WS] WebSocket connected successfully!");
  } else {
    Serial.println("[WS] WebSocket connection failed!");
  }
}



void setupBuzzer() {
  pinMode(BUZZER_PIN_POS, OUTPUT);
  pinMode(BUZZER_PIN_NEG, OUTPUT);
  digitalWrite(BUZZER_PIN_NEG, LOW);
}

void checkAlarmConditions() {
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);
  int gasValue = analogRead(MQ2_PIN);
  
  if (temp.temperature > TEMP_THRESHOLD || gasValue > GAS_THRESHOLD) {
    digitalWrite(BUZZER_PIN_POS, HIGH);
    digitalWrite(BUZZER_PIN_NEG, LOW);
  } else {
    digitalWrite(BUZZER_PIN_POS, LOW);
    digitalWrite(BUZZER_PIN_NEG, LOW);
  }
}

void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  setupBuzzer();

  if (!aht.begin()) {
    Serial.println("Could not find AHT sensor!");
    while (1) delay(10);
  }
  
  pinMode(MQ2_PIN, INPUT);
  
  String storedSSID = "", storedPassword = "";
  readWiFiCredentials(storedSSID, storedPassword);
  
  if (!storedSSID.isEmpty() && !storedPassword.isEmpty()) {
    if (connectToWiFi(storedSSID, storedPassword)) {
      startWebSocket();
      return;
    }
  }
  setupAccessPoint();
  setupUDP();
}

void loop() {
  handleUDPMessage();

  checkAlarmConditions();

  if (WiFi.status() == WL_CONNECTED) {
    if (webSocket.isConnected()) {
      webSocket.loop();
      sendSensorData();
      if (millis() - lastPongTime > PONG_TIMEOUT) {
        Serial.println("[WS] No pong received - disconnecting");
        webSocket.disconnect();
        lastPongTime = millis();
        delay(1000);
        startWebSocket();
        return;
      }
    } else {
      startWebSocket();
      delay(5000);
    }
  }
}