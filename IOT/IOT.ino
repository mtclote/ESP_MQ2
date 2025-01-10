#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h> // Thư viện cho cảm biến AHT20

// Cấu hình Wi-Fi Access Point
const char* apSSID = "SENSOR_control";
const char* apPassword = "12345678";

// Thời gian timeout
const unsigned long WIFI_TIMEOUT = 20000;

// UDP
WiFiUDP udp;
const unsigned int udpPort = 4210;
char incomingPacket[255];

// WebSocket
String WS_HOST = "192.168.170.173";
uint16_t WS_PORT = 4000;
String WS_PATH = "/?deviceId=7";
WebSocketsClient webSocket;

// Cảm biến
Adafruit_AHTX0 aht; // Đối tượng cho cảm biến AHT20
#define MQ2_PIN A0   // Chân analog cho cảm biến MQ2

// Biến thời gian
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 10000; // Gửi dữ liệu mỗi 10 giây

void setup() {
  Serial.begin(115200);
  
  // Khởi tạo cảm biến AHT20
  if (!aht.begin()) {
    Serial.println("Could not find AHT20. Check wiring");
  }
  
  // Khởi tạo chân analog cho MQ2
  pinMode(MQ2_PIN, INPUT);
  
  setupAccessPoint();
  setupUDP();
}

void loop() {
  handleUDPMessage();
  
  if (WiFi.status() == WL_CONNECTED) {
    webSocket.loop();
    
    // Kiểm tra xem đã đến lúc gửi dữ liệu chưa
    if (millis() - lastSendTime >= SEND_INTERVAL) {
      sendSensorData();
      lastSendTime = millis();
    }
  }
}

void sendSensorData() {
  // Đọc dữ liệu từ cảm biến AHT20
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);
  
  // Đọc dữ liệu từ cảm biến MQ2
  int mq2Value = analogRead(MQ2_PIN);
  float gasPercentage = map(mq2Value, 0, 1023, 0, 100); // Chuyển đổi giá trị analog thành phần trăm
  
  // Tạo JSON object để gửi dữ liệu
  DynamicJsonDocument doc(256);
  doc["type"] = "sensorData";
  doc["temperature"] = temp.temperature;
  doc["humidity"] = humidity.relative_humidity;
  doc["gas"] = int(mq2Value); // Gửi giá trị thô từ cảm biến MQ2
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  // Gửi dữ liệu qua WebSocket
  if (webSocket.isConnected()) {
    webSocket.sendTXT(jsonString);
    Serial.println("Sent: " + jsonString);
  }
}

// Xử lý sự kiện WebSocket
void webSocketEvent(WStype_t type, uint8_t *payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println("[WS] Disconnected!");
      break;
    case WStype_CONNECTED:
      Serial.println("[WS] Connected!");
      break;
    case WStype_TEXT:
      Serial.printf("[WS] Received text: %s\n", payload);
      break;
  }
}

void setupAccessPoint() {
  WiFi.softAP(apSSID, apPassword);
  Serial.println("[AP] Access Point created:");
  Serial.print("SSID: ");
  Serial.println(apSSID);
  Serial.print("Password: ");
  Serial.println(apPassword);
}

void setupUDP() {
  udp.begin(udpPort);
  Serial.println("[UDP] Listening on port " + String(udpPort));
}

void handleUDPMessage() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, 255);
    if (len > 0) incomingPacket[len] = '\0';

    String receivedData = String(incomingPacket);
    String ssid = parseValue(receivedData, "SSID");
    String password = parseValue(receivedData, "PASSWORD");

    if (ssid.isEmpty() || password.isEmpty()) {
      Serial.println("[UDP] Invalid data, missing SSID or PASSWORD");
      return;
    }

    Serial.println("[UDP] SSID: " + ssid + ", PASSWORD: " + password);
    connectToWiFi(ssid, password);
  }
}

void connectToWiFi(String ssid, String password) {
  Serial.println("[WiFi] Starting connection...");
  WiFi.begin(ssid.c_str(), password.c_str());

  unsigned long startAttemptTime = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] Connected! IP: " + WiFi.localIP().toString());
    startWebSocket();
  } else {
    Serial.println("\n[WiFi] Failed to connect. Please check SSID/PASSWORD.");
  }
}

void startWebSocket() {
  webSocket.onEvent(webSocketEvent);
  webSocket.begin(WS_HOST.c_str(), WS_PORT, WS_PATH.c_str());
  webSocket.setReconnectInterval(5000);
}

String parseValue(String data, String key) {
  int startIndex = data.indexOf(key + "=");
  if (startIndex == -1) return "";
  startIndex += key.length() + 1;
  int endIndex = data.indexOf(";", startIndex);
  return endIndex == -1 ? "" : data.substring(startIndex, endIndex);
}
