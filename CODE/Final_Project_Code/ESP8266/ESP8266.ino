#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

const char* ssid = "nihal";
const char* password = "nihal123";
const char* serverUrl = "http://192.168.164.59:5000/sensor";  // Replace with your actual PC IP

void setup() {
  Serial.begin(115200);  // STM32 to ESP8266 UART
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected.");
   Serial.println(WiFi.localIP());
}

void loop() {
  if (Serial.available()) {
    String jsonData = Serial.readStringUntil('\n');  // read JSON line
    jsonData.trim();

    Serial.println("Received JSON: " + jsonData);

    if (WiFi.status() == WL_CONNECTED) {
      WiFiClient client;
      HTTPClient http;
      http.begin(client, serverUrl);
      http.addHeader("Content-Type", "application/json");

      int responseCode = http.POST(jsonData);
      String response = http.getString();

      Serial.printf("Response [%d]: %s\n", responseCode, response.c_str());

      http.end();
    } else {
      Serial.println("WiFi not connected.");
    }
  }

  delay(100);  // delay to avoid flooding server
} 
