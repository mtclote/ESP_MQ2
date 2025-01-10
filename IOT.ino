#include <ESP8266WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <Adafruit_AHTX0.h>

// Cấu hình WiFi
const char* WIFI_SSID = "";     // Thay SSID WiFi
const char* WIFI_PASSWORD = "";  // Thay password WiFi 

// Cấu hình WebSocket
WebSocketsClient webSocket;
const char* WS_HOST = "192.168.1.7";  // Thay IP server 
const int WS_PORT = 4000;
const char* DEVICE_ID = "1";          // ID của thiết bị
char WS_URL[20];                      // Buffer cho URL

// Cấu hình cảm biến
Adafruit_AHTX0 aht;
const int GAS_SENSOR = A0;
bool isConnected = false;

void setupWiFi() {
    Serial.println("\nĐang kết nối WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    if(WiFi.status() == WL_CONNECTED) {
        Serial.println("\nĐã kết nối WiFi thành công!");
        Serial.print("Địa chỉ IP: ");
        Serial.println(WiFi.localIP());
    }
}

void setupWebSocket() {
    // Tạo URL với deviceId
    sprintf(WS_URL, "/?deviceId=%s", DEVICE_ID);
    
    Serial.println("Thiết lập WebSocket...");
    Serial.print("URL kết nối: ws://");
    Serial.print(WS_HOST);
    Serial.print(":");
    Serial.print(WS_PORT);
    Serial.println(WS_URL);
    
    webSocket.begin(WS_HOST, WS_PORT, WS_URL);
    webSocket.onEvent(webSocketEvent);
    webSocket.setReconnectInterval(5000);
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.println("WebSocket ngắt kết nối!");
            isConnected = false;
            break;
            
        case WStype_CONNECTED:
            Serial.printf("WebSocket đã kết nối tới server tại %s:%d\n", WS_HOST, WS_PORT);
            isConnected = true;
            break;
            
        case WStype_TEXT:
            Serial.printf("Nhận được dữ liệu: %s\n", payload);
            break;
    }
}

void sendSensorData() {
    if(!isConnected) {
        Serial.println("Đang chờ kết nối...");
        return;
    }

    // Đọc dữ liệu từ cảm biến
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);
    int gasValue = analogRead(GAS_SENSOR);

    // Tạo JSON
    StaticJsonDocument<200> doc;
    doc["type"] = "sensorData";
    doc["gas"] = gasValue;
    doc["temperature"] = temp.temperature;
    doc["humidity"] = humidity.relative_humidity;

    // Chuyển đổi sang chuỗi
    String jsonString;
    serializeJson(doc, jsonString);

    // Gửi dữ liệu
    webSocket.sendTXT(jsonString);

    // In thông tin debug
    Serial.println("\n=== Dữ liệu cảm biến ===");
    Serial.print("Nhiệt độ: ");
    Serial.print(temp.temperature);
    Serial.println("°C");
    Serial.print("Độ ẩm: ");
    Serial.print(humidity.relative_humidity);
    Serial.println("%");
    Serial.print("Khí gas: ");
    Serial.println(gasValue);
    Serial.println("Đã gửi: " + jsonString);
    Serial.println("=====================");
}

void setup() {
    // Khởi tạo Serial để debug
    Serial.begin(115200);
    Serial.println("\n=== Khởi động hệ thống ===");
    
    // Khởi tạo cảm biến
    if (!aht.begin()) {
        Serial.println("Không tìm thấy cảm biến AHT! Vui lòng kiểm tra kết nối.");
        while (1) {
            delay(1000);
        }
    }
    Serial.println("Cảm biến AHT đã sẵn sàng");

    // Thiết lập kết nối
    setupWiFi();
    setupWebSocket();
    Serial.println("=== Khởi động hoàn tất ===\n");
}

void loop() {
    webSocket.loop();

    // Kiểm tra và kết nối lại WiFi nếu mất kết nối
    if(WiFi.status() != WL_CONNECTED) {
        Serial.println("Mất kết nối WiFi! Đang kết nối lại...");
        setupWiFi();
    }

    // Gửi dữ liệu định kỳ
    static unsigned long lastTime = 0;
    if (millis() - lastTime > 2000) { // Gửi dữ liệu mỗi 2 giây
        lastTime = millis();
        sendSensorData();
    }
}
