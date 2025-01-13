#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>

// Thiết lập WiFi điểm phát
const char* apSSID = "SENSOR_control";      // Tên WiFi phát ra
const char* apPassword = "12345678";        // Mật khẩu WiFi phát ra

// Thời gian chờ kết nối tối đa
const unsigned long WIFI_TIMEOUT = 20000;    // 20 giây

// Thiết lập UDP để nhận thông tin WiFi
WiFiUDP udp;
const unsigned int udpPort = 4210;          // Cổng UDP
char incomingPacket[255];                   // Bộ nhớ đệm nhận dữ liệu UDP

// Thiết lập kết nối với máy chủ
String WS_HOST = "192.168.170.173";         // Địa chỉ IP máy chủ - Cần thay đổi
uint16_t WS_PORT = 4000;                    // Cổng máy chủ
String WS_PATH = "/?deviceId=7";            // ID của thiết bị - Cần thay đổi
WebSocketsClient webSocket;

// Khởi tạo cảm biến
Adafruit_AHTX0 aht;                         // Cảm biến nhiệt độ, độ ẩm AHT20
#define MQ2_PIN A0                          // Chân kết nối cảm biến khí gas MQ2

// Biến đếm thời gian
unsigned long lastSendTime = 0;              // Thời điểm gửi dữ liệu lần cuối
const unsigned long SEND_INTERVAL = 5000;    // Gửi dữ liệu mỗi 5 giây

void setup() {
  Serial.begin(115200);
  
  // Khởi động cảm biến AHT20
  if (!aht.begin()) {
    Serial.println("Không tìm thấy cảm biến AHT20. Kiểm tra lại dây");
  }
  
  // Cài đặt chân đọc cảm biến MQ2
  pinMode(MQ2_PIN, INPUT);
  
  setupAccessPoint();                        // Tạo điểm phát WiFi
  setupUDP();                               // Khởi động UDP
}

void loop() {
  handleUDPMessage();      // Xử lý tin nhắn UDP nếu có
  
  if (WiFi.status() == WL_CONNECTED) {
    webSocket.loop();      // Duy trì kết nối với máy chủ
    
    // Kiểm tra đã đến lúc gửi dữ liệu chưa
    if (millis() - lastSendTime >= SEND_INTERVAL) {
      sendSensorData();    // Gửi dữ liệu cảm biến
      lastSendTime = millis();
    }
  }
}

// Hàm gửi dữ liệu cảm biến
void sendSensorData() {
  // Đọc nhiệt độ và độ ẩm từ AHT20
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);
  
  // Đọc giá trị khí gas từ MQ2
  int gasValue = analogRead(MQ2_PIN);
  
  // Tạo dữ liệu JSON để gửi
  DynamicJsonDocument doc(256);
  doc["type"] = "sensorData";
  doc["gas"] = gasValue;
  doc["temperature"] = temp.temperature;
  doc["humidity"] = humidity.relative_humidity;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  // Gửi dữ liệu lên máy chủ
  if (webSocket.isConnected()) {
    webSocket.sendTXT(jsonString);
    Serial.println("Đã gửi: " + jsonString);
  }
}

// Xử lý sự kiện từ máy chủ
void webSocketEvent(WStype_t type, uint8_t *payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println("[WS] Đã ngắt kết nối!");
      break;
    case WStype_CONNECTED:
      Serial.println("[WS] Đã kết nối!");
      break;
    case WStype_TEXT:
      Serial.printf("[WS] Nhận được tin nhắn: %s\n", payload);
      break;
  }
}

// Tạo điểm phát WiFi
void setupAccessPoint() {
  WiFi.softAP(apSSID, apPassword);
  Serial.println("[AP] Đã tạo điểm phát WiFi:");
  Serial.print("Tên WiFi: ");
  Serial.println(apSSID);
  Serial.print("Mật khẩu: ");
  Serial.println(apPassword);
}

// Khởi động UDP
void setupUDP() {
  udp.begin(udpPort);
  Serial.println("[UDP] Đang lắng nghe tại cổng " + String(udpPort));
}

// Xử lý tin nhắn UDP
void handleUDPMessage() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, 255);
    if (len > 0) incomingPacket[len] = '\0';

    String receivedData = String(incomingPacket);
    String ssid = parseValue(receivedData, "SSID");
    String password = parseValue(receivedData, "PASSWORD");

    if (ssid.isEmpty() || password.isEmpty()) {
      Serial.println("[UDP] Dữ liệu không hợp lệ, thiếu SSID hoặc PASSWORD");
      return;
    }

    Serial.println("[UDP] Tên WiFi: " + ssid + ", Mật khẩu: " + password);
    connectToWiFi(ssid, password);
  }
}

// Kết nối WiFi
void connectToWiFi(String ssid, String password) {
  Serial.println("[WiFi] Bắt đầu kết nối...");
  WiFi.begin(ssid.c_str(), password.c_str());

  unsigned long startAttemptTime = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] Đã kết nối! IP: " + WiFi.localIP().toString());
    startWebSocket();
  } else {
    Serial.println("\n[WiFi] Kết nối thất bại. Kiểm tra lại tên và mật khẩu WiFi.");
  }
}

// Khởi động kết nối với máy chủ
void startWebSocket() {
  webSocket.onEvent(webSocketEvent);
  webSocket.begin(WS_HOST.c_str(), WS_PORT, WS_PATH.c_str());
  webSocket.setReconnectInterval(5000);  // Thử kết nối lại sau mỗi 5 giây nếu mất kết nối
}

// Hàm tách thông tin từ chuỗi UDP
String parseValue(String data, String key) {
  int startIndex = data.indexOf(key + "=");
  if (startIndex == -1) return "";
  startIndex += key.length() + 1;
  int endIndex = data.indexOf(";", startIndex);
  return endIndex == -1 ? "" : data.substring(startIndex, endIndex);
}
