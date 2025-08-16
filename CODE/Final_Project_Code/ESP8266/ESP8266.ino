#define BLYNK_TEMPLATE_ID "TMPL3MfRvCDZG"
#define BLYNK_TEMPLATE_NAME "SENSORS DATA"
#define BLYNK_AUTH_TOKEN "ivqYTHmN5kKdvZ2wwoctPoVAIcclZy4F"

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <BlynkSimpleEsp8266.h>

// WiFi & Server
const char* ssid = "nihal";
const char* password = "nihal123";
const char* serverUrl = "http://192.168.164.59:5000/sensor";

// Blynk Auth Token
char auth[] = BLYNK_AUTH_TOKEN;

void setup() {
  Serial.begin(115200);  // UART to STM32
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi Connected.");
  Serial.println(WiFi.localIP());

  // Connect to Blynk server
  Blynk.begin(auth, ssid, password);
}

void loop() {
  Blynk.run();  // Required to keep Blynk connected

  // 1. Read JSON from STM32 (via UART)
  if (Serial.available()) {
    String jsonData = Serial.readStringUntil('\n');  // Expect JSON ending with \n
    jsonData.trim();  // Remove any leading/trailing whitespace
    Serial.println("Received JSON: " + jsonData);

    // 2. Parse JSON
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, jsonData);

    if (error) {
      Serial.print("JSON parse failed: ");
      Serial.println(error.c_str());
      return;
    }

    // 3. Extract fields
    int ldr  = doc["LDR_DATA"];
    int mq5  = doc["MQ5_DATA"];
    int temp = doc["TEMPERATURE"];
    int hum  = doc["HUMIDITY"];
    const char* location = doc["LOCATION"];

   
    // 4. Send to Flask Server
    if (WiFi.status() == WL_CONNECTED) 
    {
      WiFiClient client;
      HTTPClient http;
      http.begin(client, serverUrl);
      http.addHeader("Content-Type", "application/json");

      int responseCode = http.POST(jsonData);
      String response = http.getString();

      Serial.printf("Flask Response [%d]: %s\n", responseCode, response.c_str());
      http.end();
    }
    else 
      Serial.println("WiFi not connected (Flask part skipped).");
    

    // 5. Send to Blynk
    Blynk.virtualWrite(V0, ldr); Blynk.virtualWrite(V1, mq5); Blynk.virtualWrite(V2, temp); Blynk.virtualWrite(V3, hum);
    
    // 6. Conditional alerts
    if (ldr > 2000) Blynk.logEvent("ldr_alert", "⚠️ Darkness Detected!");
    if (mq5 > 1500) Blynk.logEvent("gas_alert", "⚠️ Gas Leakage Detected!");
    if (temp > 30)  Blynk.logEvent("temp_alert", "⚠️ High Temperature!");
  }

  delay(100);  // Small delay to prevent UART flooding
}
