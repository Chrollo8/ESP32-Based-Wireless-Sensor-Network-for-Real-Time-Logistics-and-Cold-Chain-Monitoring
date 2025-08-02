/*********
 * ESP32-S3 Master Code with BME280, GPS, MPU6050, Web Server and Dual GPIO SMS Alert
 * Sends HIGH signals on separate GPIO pins for temperature and humidity breaches.
 * COMMUNICATION METHOD: DUAL WIRED GPIO JUMPERS
 *
 * V3 UPDATE: Separate GPIO pins for temperature and humidity breach alerts
 * V4 UPDATE: Integrated HTML dashboard via LittleFS
 * V5 UPDATE: Added MPU6050 accelerometer on IO17 (SCL) and IO18 (SDA)
*********/

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <ArduinoJson.h>
#include <vector>

/* Definitions */
// GPIO pins for different alert types
#define TEMP_ALERT_PIN 10      // GPIO pin for temperature breach alerts
#define HUMIDITY_ALERT_PIN 11  // GPIO pin for humidity breach alerts
#define STARTUP_TEST_PULSE_MS 500 // Duration of the initial test pulse

// I2C pin definitions for ESP32-S3
// BME280 sensor
#define BME_I2C_SDA 4
#define BME_I2C_SCL 5

// MPU6050 sensor
#define MPU_I2C_SDA 18
#define MPU_I2C_SCL 17

// GPS UART pin definitions for ESP32-S3
#define GPS_RX_PIN 46 // This is actually RXD2
#define GPS_TX_PIN 9  // This is actually TXD2

// == THRESHOLD SETTINGS ==
#define MIN_TEMP 20.0
#define MAX_TEMP 28.0
#define MIN_HUMIDITY 30.0
#define MAX_HUMIDITY 80.0

// Create sensor objects
Adafruit_BME280 bme;
Adafruit_MPU6050 mpu;
TwoWire I2C_BME = TwoWire(0);  // I2C bus for BME280
TwoWire I2C_MPU = TwoWire(1);  // I2C bus for MPU6050

// GPS module connection using UART1
HardwareSerial gpsSerial(1); // Use UART1 for GPS

// Replace with your network credentials for the Web Server
const char* ssid = "C-137";
const char* password = "sadaf@12";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Structure to hold detailed GPS data
struct GPSRawData {
  double latitude;
  char latitudeDir;
  double longitude;
  char longitudeDir;
  int satellites;
  double altitude;
  int hours, minutes, seconds;
  int day, month, year;
  String timestamp;
};

// Structure to store simplified GPS data for display
struct GPSData {
  double latitude;
  double longitude;
  String timestamp;
  int satellites;
  double altitude;
};

// Structure to store MPU6050 acceleration data
struct AccelerationData {
  float x;
  float y;
  float z;
};

bool gpsDataValid = false;
GPSData latestGPSData;
GPSRawData latestGPSRawData;

// Global variables for sensor readings and breach status
bool g_temperatureBreach = false;
bool g_humidityBreach = false;
float g_currentTemperature = 0.0;
float g_currentHumidity = 0.0;
float g_currentPressure = 0.0;

// Global variables for MPU6050 acceleration data
AccelerationData g_currentAcceleration = {0.0, 0.0, 0.0};
bool g_mpuInitialized = false;

// SMS alert tracking - separate cooldowns for each alert type
unsigned long lastTempSMSAlert = 0;
unsigned long lastHumiditySMSAlert = 0;
const unsigned long SMS_COOLDOWN = 300000; // 5 minutes between alerts

// Function Prototypes
void readSensors();
void readMPU6050();
void checkThresholds();
void sendTemperatureAlert();
void sendHumidityAlert();
void processGPSData(String raw);
void parseGPGGA(String gpgga);
void parseGPRMC(String gprmc);
double nmeaToDecimal(String nmeaCoord);
void convertAndPrintLocalDateTime();
int daysInMonth(int month, int year);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32-S3 Sensor Hub - Dual GPIO Alert Master with MPU6050 Accelerometer");

  // Configure both alert pins as outputs and set them to LOW initially
  pinMode(TEMP_ALERT_PIN, OUTPUT);
  pinMode(HUMIDITY_ALERT_PIN, OUTPUT);
  digitalWrite(TEMP_ALERT_PIN, LOW);
  digitalWrite(HUMIDITY_ALERT_PIN, LOW);
  
  // ===================================================================
  // === STARTUP TEST: Send test signals on both pins ===
  // ===================================================================
  Serial.println("\nPERFORMING STARTUP TEST...");
  Serial.println("Testing TEMPERATURE alert line (GPIO " + String(TEMP_ALERT_PIN) + ")...");
  digitalWrite(TEMP_ALERT_PIN, HIGH);
  delay(STARTUP_TEST_PULSE_MS);
  digitalWrite(TEMP_ALERT_PIN, LOW);
  delay(500); // Small gap between tests
  
  Serial.println("Testing HUMIDITY alert line (GPIO " + String(HUMIDITY_ALERT_PIN) + ")...");
  digitalWrite(HUMIDITY_ALERT_PIN, HIGH);
  delay(STARTUP_TEST_PULSE_MS);
  digitalWrite(HUMIDITY_ALERT_PIN, LOW);
  
  Serial.println("Both test pulses sent. Master setup will now continue.");
  Serial.println("----------------------------------------------\n");
  delay(1000); // Give a moment before continuing

  // Initialize I2C buses with custom pins for ESP32-S3
  I2C_BME.begin(BME_I2C_SDA, BME_I2C_SCL, 100000);  // BME280 on SDA=4, SCL=5
  I2C_MPU.begin(MPU_I2C_SDA, MPU_I2C_SCL, 100000);  // MPU6050 on SDA=18, SCL=17
  Serial.println("I2C buses initialized:");
  Serial.println("  BME280 - SDA: IO4, SCL: IO5");
  Serial.println("  MPU6050 - SDA: IO18, SCL: IO17");

  // Initialize BME280 on its dedicated I2C bus
  if (!bme.begin(0x76, &I2C_BME)) {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
      while (true) { delay(1000); }
  }
  Serial.println("BME280 sensor initialized successfully!");

  // Initialize MPU6050 on its dedicated I2C bus
  if (!mpu.begin(0x68, &I2C_MPU)) {
    Serial.println("Failed to find MPU6050 chip on SDA=18, SCL=17");
    g_mpuInitialized = false;
  } else {
    Serial.println("MPU6050 Found!");
    
    // Configure MPU6050 settings
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    Serial.print("Accelerometer range set to: ");
    switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
    }
    
    g_mpuInitialized = true;
    Serial.println("MPU6050 sensor initialized successfully!");
  }

  // Initialize GPS module on UART1
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS Module Initialized - RX: IO46, TX: IO9");

  // Initialize LittleFS for the web page
  if (!LittleFS.begin(true)) {
    Serial.println("An Error has occurred while mounting LittleFS");
    Serial.println("Make sure you have uploaded the index.html file to LittleFS!");
    return;
  }
  Serial.println("LittleFS mounted successfully");

  // Check if index.html exists
  if (!LittleFS.exists("/index.html")) {
    Serial.println("WARNING: /index.html not found in LittleFS!");
    Serial.println("Please upload the HTML file using the ESP32 LittleFS Data Upload Tool");
  } else {
    Serial.println("Dashboard HTML file found in LittleFS");
  }

  // Connect to Wi-Fi for the web server
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi for web server");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("ESP32-S3 IP Address: ");
  Serial.println(WiFi.localIP());

  // --- Web Server Routes ---
  
  // Serve the main dashboard page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    if (LittleFS.exists("/index.html")) {
      request->send(LittleFS, "/index.html", "text/html");
    } else {
      // Fallback simple page if HTML file is missing
      String fallbackHTML = R"(
<!DOCTYPE HTML><html><head><title>ThermaTrack - System Status</title></head>
<body style="font-family:Arial;padding:20px;background:#f0f0f0;">
<div style="max-width:800px;margin:0 auto;background:white;padding:20px;border-radius:10px;">
<h1 style="color:#333;text-align:center;">ThermaTrack Dashboard</h1>
<p style="color:red;text-align:center;"><strong>HTML Dashboard file not found!</strong></p>
<p>Please upload the index.html file to LittleFS using the ESP32 Data Upload Tool.</p>
<h3>Current Readings:</h3>
<p>Temperature: <span id="temp">Loading...</span>°C</p>
<p>Humidity: <span id="hum">Loading...</span>%</p>
<p>Pressure: <span id="press">Loading...</span> hPa</p>
<script>
setInterval(function(){
  fetch('/temperature').then(r=>r.text()).then(d=>document.getElementById('temp').textContent=d);
  fetch('/humidity').then(r=>r.text()).then(d=>document.getElementById('hum').textContent=d);
  fetch('/pressure').then(r=>r.text()).then(d=>document.getElementById('press').textContent=d);
}, 2000);
</script>
</div></body></html>
      )";
      request->send(200, "text/html", fallbackHTML);
    }
  });

  // API endpoints for sensor data
  server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", String(g_currentTemperature, 2));
  });

  server.on("/humidity", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", String(g_currentHumidity, 2));
  });

  server.on("/pressure", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", String(g_currentPressure, 2));
  });

  // NEW: MPU6050 acceleration data endpoints
  server.on("/acceleration", HTTP_GET, [](AsyncWebServerRequest *request){
    StaticJsonDocument<200> doc;
    doc["x"] = g_currentAcceleration.x;
    doc["y"] = g_currentAcceleration.y;
    doc["z"] = g_currentAcceleration.z;
    doc["initialized"] = g_mpuInitialized;
    String jsonString;
    serializeJson(doc, jsonString);
    request->send(200, "application/json", jsonString);
  });

  server.on("/accel-x", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", String(g_currentAcceleration.x, 3));
  });

  server.on("/accel-y", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", String(g_currentAcceleration.y, 3));
  });

  server.on("/accel-z", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", String(g_currentAcceleration.z, 3));
  });

  // Breach status endpoint for dashboard
  server.on("/breach-status", HTTP_GET, [](AsyncWebServerRequest *request){
    StaticJsonDocument<256> doc;
    doc["temp_breach"] = g_temperatureBreach;
    doc["humidity_breach"] = g_humidityBreach;
    doc["temperature"] = g_currentTemperature;
    doc["humidity"] = g_currentHumidity;
    doc["pressure"] = g_currentPressure;
    doc["min_temp"] = MIN_TEMP;
    doc["max_temp"] = MAX_TEMP;
    doc["min_humidity"] = MIN_HUMIDITY;
    doc["max_humidity"] = MAX_HUMIDITY;
    String jsonString;
    serializeJson(doc, jsonString);
    request->send(200, "application/json", jsonString);
  });

  // GPS data endpoint for dashboard
  server.on("/gps", HTTP_GET, [](AsyncWebServerRequest *request){
    StaticJsonDocument<256> doc;
    doc["valid"] = gpsDataValid;
    if (gpsDataValid) {
      doc["latitude"] = latestGPSData.latitude;
      doc["longitude"] = latestGPSData.longitude;
      doc["satellites"] = latestGPSData.satellites;
      doc["altitude"] = latestGPSData.altitude;
      doc["timestamp"] = latestGPSData.timestamp;
    } else {
      doc["latitude"] = 0.0;
      doc["longitude"] = 0.0;
      doc["satellites"] = 0;
      doc["altitude"] = 0.0;
      doc["timestamp"] = "No GPS data";
    }
    String jsonString;
    serializeJson(doc, jsonString);
    request->send(200, "application/json", jsonString);
  });

  // Manual test endpoints for alert testing
  server.on("/test-temp-alert", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("Manual TEMPERATURE alert triggered via web interface");
    sendTemperatureAlert();
    request->send(200, "text/plain", "Temperature Alert signal sent on GPIO " + String(TEMP_ALERT_PIN));
  });

  server.on("/test-humidity-alert", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("Manual HUMIDITY alert triggered via web interface");
    sendHumidityAlert();
    request->send(200, "text/plain", "Humidity Alert signal sent on GPIO " + String(HUMIDITY_ALERT_PIN));
  });

  // System info endpoint
  server.on("/system-info", HTTP_GET, [](AsyncWebServerRequest *request){
    StaticJsonDocument<512> doc;
    doc["device"] = "ESP32-S3 ThermaTrack Master";
    doc["version"] = "v5.0";
    doc["wifi_ssid"] = ssid;
    doc["ip_address"] = WiFi.localIP().toString();
    doc["uptime"] = millis();
    doc["free_heap"] = ESP.getFreeHeap();
    doc["temp_alert_pin"] = TEMP_ALERT_PIN;
    doc["humidity_alert_pin"] = HUMIDITY_ALERT_PIN;
    doc["mpu6050_initialized"] = g_mpuInitialized;
    doc["littlefs_total"] = LittleFS.totalBytes();
    doc["littlefs_used"] = LittleFS.usedBytes();
    String jsonString;
    serializeJson(doc, jsonString);
    request->send(200, "application/json", jsonString);
  });

  // Handle 404 errors
  server.onNotFound([](AsyncWebServerRequest *request){
    request->send(404, "text/plain", "Page Not Found");
  });

  // Start server
  server.begin();
  Serial.println("Web server started successfully!");
  Serial.println("===========================================");
  Serial.println("ThermaTrack Dashboard Access Information:");
  Serial.println("URL: http://" + WiFi.localIP().toString());
  Serial.println("Temperature Alerts: GPIO " + String(TEMP_ALERT_PIN));
  Serial.println("Humidity Alerts: GPIO " + String(HUMIDITY_ALERT_PIN));
  Serial.println("MPU6050 Status: " + String(g_mpuInitialized ? "Initialized" : "Failed"));
  Serial.println("===========================================");
}

// Function to read environmental sensors (BME280)
void readSensors() {
  g_currentTemperature = bme.readTemperature();
  g_currentHumidity = bme.readHumidity();
  g_currentPressure = bme.readPressure() / 100.0F;
}

// Function to read MPU6050 accelerometer data
void readMPU6050() {
  if (!g_mpuInitialized) return;
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  g_currentAcceleration.x = a.acceleration.x;
  g_currentAcceleration.y = a.acceleration.y;
  g_currentAcceleration.z = a.acceleration.z;
}

// Function to check sensor readings against thresholds
void checkThresholds() {
  bool prevTempBreach = g_temperatureBreach;
  bool prevHumidityBreach = g_humidityBreach;
  
  g_temperatureBreach = (g_currentTemperature > MAX_TEMP || g_currentTemperature < MIN_TEMP);
  g_humidityBreach = (g_currentHumidity > MAX_HUMIDITY || g_currentHumidity < MIN_HUMIDITY);
  
  // Check for new temperature breach
  if (g_temperatureBreach && !prevTempBreach && (millis() - lastTempSMSAlert > SMS_COOLDOWN)) {
    Serial.println("*** NEW TEMPERATURE BREACH DETECTED - Triggering Temperature Alert ***");
    Serial.println("Current: " + String(g_currentTemperature, 1) + "°C (Range: " + String(MIN_TEMP) + "-" + String(MAX_TEMP) + "°C)");
    sendTemperatureAlert();
    lastTempSMSAlert = millis();
  }
  
  // Check for new humidity breach
  if (g_humidityBreach && !prevHumidityBreach && (millis() - lastHumiditySMSAlert > SMS_COOLDOWN)) {
    Serial.println("*** NEW HUMIDITY BREACH DETECTED - Triggering Humidity Alert ***");
    Serial.println("Current: " + String(g_currentHumidity, 1) + "% (Range: " + String(MIN_HUMIDITY) + "-" + String(MAX_HUMIDITY) + "%)");
    sendHumidityAlert();
    lastHumiditySMSAlert = millis();
  }
}

// Function to send temperature breach alert via GPIO
void sendTemperatureAlert() {
  Serial.println("Sending TEMPERATURE alert: HIGH pulse on GPIO " + String(TEMP_ALERT_PIN));

  // Send a 1-second pulse on the temperature alert pin
  digitalWrite(TEMP_ALERT_PIN, HIGH);
  delay(1000); 
  digitalWrite(TEMP_ALERT_PIN, LOW);
  
  Serial.println("Temperature alert pulse finished. Pin is back to LOW.");
}

// Function to send humidity breach alert via GPIO
void sendHumidityAlert() {
  Serial.println("Sending HUMIDITY alert: HIGH pulse on GPIO " + String(HUMIDITY_ALERT_PIN));

  // Send a 1-second pulse on the humidity alert pin
  digitalWrite(HUMIDITY_ALERT_PIN, HIGH);
  delay(1000); 
  digitalWrite(HUMIDITY_ALERT_PIN, LOW);
  
  Serial.println("Humidity alert pulse finished. Pin is back to LOW.");
}

void loop() {
  // Read and process GPS data from the module
  static String gpsDataBuffer = "";
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    gpsDataBuffer += c;
    if (c == '\n') {
      processGPSData(gpsDataBuffer);
      gpsDataBuffer = "";
    }
  }
  
  // Periodically read sensors and check thresholds
  static unsigned long lastSensorRead = 0;
  if (millis() - lastSensorRead > 2000) { // Read sensors every 2 seconds
    lastSensorRead = millis();
    readSensors();      // Read BME280 data
    readMPU6050();      // Read MPU6050 data
    checkThresholds();
  }
  
  // Enhanced debug output every 10 seconds
  static unsigned long lastReading = 0;
  if (millis() - lastReading > 10000) {
    Serial.println("=== System Status ===");
    Serial.print("Temperature: " + String(g_currentTemperature, 1) + "°C");
    if(g_temperatureBreach) Serial.println(" <-- BREACH!"); else Serial.println(" (OK)");
    Serial.print("Humidity: " + String(g_currentHumidity, 1) + "%");
    if(g_humidityBreach) Serial.println(" <-- BREACH!"); else Serial.println(" (OK)");
    Serial.println("Pressure: " + String(g_currentPressure, 1) + " hPa");
    
    if (g_mpuInitialized) {
      Serial.println("Acceleration - X: " + String(g_currentAcceleration.x, 2) + 
                    " Y: " + String(g_currentAcceleration.y, 2) + 
                    " Z: " + String(g_currentAcceleration.z, 2) + " m/s²");
    } else {
      Serial.println("Acceleration: MPU6050 not initialized");
    }
    
    if(gpsDataValid) {
      Serial.println("GPS: " + String(latestGPSData.latitude, 6) + ", " + String(latestGPSData.longitude, 6) + 
                    " (" + String(latestGPSData.satellites) + " sats)");
    } else {
      Serial.println("GPS: No valid data");
    }
    Serial.println("Dashboard: http://" + WiFi.localIP().toString());
    Serial.println("=====================");
    lastReading = millis();
  }
  
  delay(10);
}

// --- GPS Data Processing Functions ---
void processGPSData(String raw) {
  if (raw.startsWith("$GPGGA")) {
    parseGPGGA(raw);
    convertAndPrintLocalDateTime();
  } else if (raw.startsWith("$GPRMC")) {
    parseGPRMC(raw);
  }
}

void parseGPGGA(String gpgga) {
  String tokens[15];
  int tokenIndex = 0;
  int startIndex = 0;
  for (int i = 0; i < gpgga.length(); i++) {
    if (gpgga[i] == ',' || gpgga[i] == '*') {
      tokens[tokenIndex++] = gpgga.substring(startIndex, i);
      startIndex = i + 1;
    }
  }

  if (tokenIndex > 9) {
    String utcTime = tokens[1];
    if (utcTime.length() >= 6) {
      latestGPSRawData.hours = utcTime.substring(0, 2).toInt();
      latestGPSRawData.minutes = utcTime.substring(2, 4).toInt();
      latestGPSRawData.seconds = utcTime.substring(4, 6).toInt();
    }

    latestGPSData.latitude = nmeaToDecimal(tokens[2]);
    if (tokens[3].length() > 0 && tokens[3].charAt(0) == 'S') {
      latestGPSData.latitude = -latestGPSData.latitude;
    }
    
    latestGPSData.longitude = nmeaToDecimal(tokens[4]);
    if (tokens[5].length() > 0 && tokens[5].charAt(0) == 'W') {
      latestGPSData.longitude = -latestGPSData.longitude;
    }

    latestGPSData.satellites = tokens[7].toInt();
    latestGPSData.altitude = tokens[9].toDouble();

    if (latestGPSData.satellites >= 4 && latestGPSData.latitude != 0 && latestGPSData.longitude != 0) {
      gpsDataValid = true;
    } else {
      gpsDataValid = false;
    }
  }
}

void parseGPRMC(String gprmc) {
  String tokens[15];
  int tokenIndex = 0;
  int startIndex = 0;
  for (int i = 0; i < gprmc.length(); i++) {
    if (gprmc[i] == ',' || gprmc[i] == '*') {
      tokens[tokenIndex++] = gprmc.substring(startIndex, i);
      startIndex = i + 1;
    }
  }

  if (tokenIndex > 9 && tokens[9].length() >= 6) {
    String utcDate = tokens[9];
    latestGPSRawData.day = utcDate.substring(0, 2).toInt();
    latestGPSRawData.month = utcDate.substring(2, 4).toInt();
    latestGPSRawData.year = 2000 + utcDate.substring(4, 6).toInt();
  }
}

void convertAndPrintLocalDateTime() {
  int offsetHours = 5;
  int offsetMinutes = 30;

  int localHours = latestGPSRawData.hours + offsetHours;
  int localMinutes = latestGPSRawData.minutes + offsetMinutes;
  int localDay = latestGPSRawData.day;
  int localMonth = latestGPSRawData.month;
  int localYear = latestGPSRawData.year;

  if (localMinutes >= 60) {
    localMinutes -= 60;
    localHours++;
  }
  if (localHours >= 24) {
    localHours -= 24;
    localDay++;
    if (localDay > daysInMonth(localMonth, localYear)) {
      localDay = 1;
      localMonth++;
      if (localMonth > 12) {
        localMonth = 1;
        localYear++;
      }
    }
  }

  char timeBuffer[20];
  snprintf(timeBuffer, sizeof(timeBuffer), "%04d-%02d-%02d %02d:%02d:%02d",
           localYear, localMonth, localDay,
           localHours, localMinutes, latestGPSRawData.seconds);
  latestGPSData.timestamp = String(timeBuffer);
}

int daysInMonth(int month, int year) {
  if (month == 2) {
    return (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)) ? 29 : 28;
  }
  return (month == 4 || month == 6 || month == 9 || month == 11) ? 30 : 31;
}

double nmeaToDecimal(String nmeaCoord) {
  if (nmeaCoord == "" || nmeaCoord.toDouble() == 0.0) return 0.0;
  double raw = nmeaCoord.toDouble();
  int degrees = int(raw / 100);
  double minutes = raw - (degrees * 100);
  return degrees + (minutes / 60.0);
}