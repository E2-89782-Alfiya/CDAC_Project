#define BLYNK_TEMPLATE_ID "TMPL3MfRvCDZG"
#define BLYNK_TEMPLATE_NAME "SENSORS DATA"
#define BLYNK_AUTH_TOKEN "ivqYTHmN5kKdvZ2wwoctPoVAIcclZy4F"

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <BlynkSimpleEsp8266.h>

// WiFi
const char* ssid = "GalaxyM31";
const char* password = "0987654321";

// Flask server URL
const char* serverUrl = "http://192.168.196.109:4000/sensordata";

// Blynk Auth Token
char auth[] = BLYNK_AUTH_TOKEN;  // Already defined above

void setup() {
  Serial.begin(115200);
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi Connected.");
  Serial.println(WiFi.localIP());

  // Connect to Blynk
  Blynk.begin(auth, ssid, password);
}

void loop() {
  Blynk.run();  // Required for Blynk to work

  // ------ Simulated received JSON from STM32 ------
  String jsonData = "{\"LDR_DATA\":987,\"MQ5_DATA\":1500,\"TEMPERATURE\":26,\"HUMIDITY\":72,\"LOCATION\":\"Lab1\"}";

  // Print full JSON
  Serial.println("Received JSON: " + jsonData);

  // Parse JSON
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, jsonData);

  if (error) {
    Serial.print("JSON parse failed: ");
    Serial.println(error.c_str());
    return;
  }

  // Extract data
  int ldr = doc["LDR_DATA"];
  int mq5 = doc["MQ5_DATA"];
  int temp = doc["TEMPERATURE"];
  int hum = doc["HUMIDITY"];
  const char* location = doc["LOCATION"];

  
  Serial.print("LDR: "); Serial.println(ldr);
  Serial.print("MQ5: "); Serial.println(mq5);
  Serial.print("Temperature: "); Serial.println(temp);
  Serial.print("Humidity: "); Serial.println(hum);
  Serial.print("Location: "); Serial.println(location);

  // -------- Send to Flask Server --------
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

  // -------- Send to Blynk --------
  Blynk.virtualWrite(V0, ldr);
  Blynk.virtualWrite(V1, mq5);
  Blynk.virtualWrite(V2, temp);
  Blynk.virtualWrite(V3, hum);
  //Blynk.virtualWrite(V4, location);

  delay(5000); 
}
