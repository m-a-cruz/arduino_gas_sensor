#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_AHTX0.h>
#include <DHT.h>

const char* ssid = "Infinix HOT 11S NFC";
const char* password = "12345678";
const char* serverGAS = "https://trashtalk-be-gas-production.up.railway.app/api/gas/data";

#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

const int gasSensorPin = 34;
const int buzzerPin = 25;
const int buttonPin = 33;  // NEW: Button to stop the buzzer

bool buzzerOverride = false;  // Flag to track if button was pressed
const int bin = 1;
String type = "";

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  dht.begin();

  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  pinMode(buttonPin, INPUT_PULLUP);  // NEW: Set button pin as input with pull-up

  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // Check if button is pressed
  if (digitalRead(buttonPin) == LOW) {
    buzzerOverride = true;              // Button was pressed
    digitalWrite(buzzerPin, LOW);       // Immediately stop the buzzer
    Serial.println("Buzzer manually stopped by button.");
  }

  int sensorValue = analogRead(gasSensorPin);
  float RS_AIR = 9.83;
  float RL = 10;
  float RS_gas = (4095.0 / (float)sensorValue) * RL;
  float ratio = RS_gas / RS_AIR;

  float LPG = pow(ratio, -2.552) * 100;
  float methane = pow(ratio, -2.271) * 100;
  float smoke = pow(ratio, -1.515) * 100;
  float CO = pow(ratio, -2.478) * 100;
  float hydrogen = pow(ratio, -2.154) * 100;

  float LEL_LPG = ((LPG / 10000) / 0.1) * 100;
  float LEL_methane = ((methane / 10000) / 0.05) * 100;
  float LEL_smoke = ((smoke / 10000) / 0.05) * 100;
  float LEL_CO = ((CO / 10000) / 0.125) * 100;
  float LEL_hydrogen = ((hydrogen / 10000) / 0.04) * 100;

  float temp = dht.readTemperature();
  float humidity = dht.readHumidity();

  if (isnan(temp) || isnan(humidity)) {
    Serial.println("Failed to read from DHT11 sensor!");
  } else {
    Serial.print("Temperature: ");
    Serial.print(temp);
    Serial.println(" °C");
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");
  }

  // Logic: Only turn on buzzer if not manually overridden
  if (!buzzerOverride) {
    if ((LEL_LPG > 10 && LEL_LPG <= 25) ||
        (LEL_methane > 10 && LEL_methane <= 25) ||
        (LEL_smoke > 10 && LEL_smoke <= 25) ||
        (LEL_CO > 10 && LEL_CO <= 25) ||
        (LEL_hydrogen > 10 && LEL_hydrogen <= 25)) {
      digitalWrite(buzzerPin, HIGH);
      type = "Critical";
      Serial.println("Warning: Gas levels rising.");
    } else if ((LEL_LPG > 25 && LEL_LPG < 100) ||
               (LEL_methane > 25 && LEL_methane < 100) ||
               (LEL_smoke > 25 && LEL_smoke < 100) ||
               (LEL_CO > 25 && LEL_CO < 100) ||
               (LEL_hydrogen > 25 && LEL_hydrogen < 100)) {
      digitalWrite(buzzerPin, HIGH);
      type = "Dangerous";
      Serial.println("Critical: High gas levels!");
    } else if ((LEL_LPG >= 100) ||
               (LEL_methane >= 100) ||
               (LEL_smoke >= 100) ||
               (LEL_CO >= 100) ||
               (LEL_hydrogen >= 100)) {
      digitalWrite(buzzerPin, HIGH);
      type = "Explosive";
      Serial.println("Explosive: Immediate evacuation!");
    } else {
      digitalWrite(buzzerPin, LOW);
      type = "Safe";
      Serial.println("Gas levels are safe.");
    }
  } else {
    Serial.println("Buzzer is manually disabled.");
  }

  String dangerousGases = "";

  // Check each gas
  if (LEL_LPG > 10) dangerousGases += "LPG,";
  if (LEL_methane > 10) dangerousGases += "Methane,";
  if (LEL_smoke > 10) dangerousGases += "Smoke,";
  if (LEL_CO > 10) dangerousGases += "CO,";
  if (LEL_hydrogen > 10) dangerousGases += "Hydrogen,";

  // Remove trailing comma
  if (dangerousGases.endsWith(",")) {
    dangerousGases.remove(dangerousGases.length() - 1);
  }

  // Prepare JSON payload
  String jsonPayload = "{";
  jsonPayload += "\"BIN\": " + String(bin) + ",";
  jsonPayload += "\"LEL_LPG\": " + String(LEL_LPG, 2) + ",";
  jsonPayload += "\"LEL_methane\": " + String(LEL_methane, 2) + ",";
  jsonPayload += "\"LEL_smoke\": " + String(LEL_smoke, 2) + ",";
  jsonPayload += "\"LEL_CO\": " + String(LEL_CO, 2) + ",";
  jsonPayload += "\"LEL_hydrogen\": " + String(LEL_hydrogen, 2) + ",";
  jsonPayload += "\"temperature\": " + String(temp, 2) + ",";
  jsonPayload += "\"humidity\": " + String(humidity, 2) + ",";
  jsonPayload += "\"type\": \"" + type + "\"" + ",";
  jsonPayload += "\"dangerous_gases\": \"" + dangerousGases + "\"";
  jsonPayload += "}";

  Serial.println("JSON Payload: " + jsonPayload);

  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverGAS);
    http.addHeader("Content-Type", "application/json");

    int httpResponseCode = http.POST(jsonPayload);
    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println("Server Response: " + response);
    } else {
      Serial.println("Error sending POST request: " + String(httpResponseCode));
    }

    http.end();
  } else {
    Serial.println("Wi-Fi not connected!");
    WiFi.begin(ssid, password);
  }

  delay(60000);  // Wait 1 minute
}
