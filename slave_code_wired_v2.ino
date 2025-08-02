/******************************************************************
 * TTGO T-Call V1.3 ESP32 Slave Code
 * Receives triggers via separate GPIO pins for temperature and humidity alerts.
 * 
 * Communication Method: Dual Wired GPIO Triggers
 * - Listens for HIGH signals on TEMP_ALERT_PIN (GPIO 12) and HUMIDITY_ALERT_PIN (GPIO 13).
 * - Sends different SMS messages based on which pin triggered the alert.
 *
 * V3 UPDATE: Separate GPIO pins for temperature and humidity breach alerts
 ******************************************************************/

#include <HardwareSerial.h>
#include <Wire.h>

/* Definitions */
#define TEMP_ALERT_PIN 12     // GPIO pin to receive temperature alert from master
#define HUMIDITY_ALERT_PIN 13 // GPIO pin to receive humidity alert from master
#define STARTUP_TEST_TIMEOUT_MS 15000 // 15 seconds to wait for both test signals

// SIM800L Pin Configuration for TTGO T-Call V1.3
#define SIM800L_TX 27
#define SIM800L_RX 26
#define SIM800L_PWKEY 4
#define SIM800L_RST 5
#define SIM800L_POWER 23

// I2C pins for power management
#define I2C_SDA 21
#define I2C_SCL 22

// IP5306 power management chip registers
#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00

// Create SIM800L serial connection
HardwareSerial sim800l(1);

// Phone number to send SMS alerts (REPLACE with your phone number)
String phoneNumber = "+8801771192499";

// Global Variables
bool sim800lReady = false;
bool networkRegistered = false;
unsigned long lastTempSMSAttempt = 0;
unsigned long lastHumiditySMSAttempt = 0;
unsigned long lastStatusCheck = 0;
const unsigned long SMS_RETRY_INTERVAL = 30000; // 30 seconds
const unsigned long STATUS_CHECK_INTERVAL = 30000; // 30 seconds

// Function Prototypes
void initPowerManagement();
void initSIM800L();
bool sendSMS(String message, String number);
String waitForResponse(unsigned long timeout = 1000);
void checkSIM800LStatus();
bool setPowerBoostKeepOn(int en);
void handleTemperatureAlert();
void handleHumidityAlert();
bool runStartupTest();

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("TTGO T-Call SMS Gateway Starting (Dual GPIO Trigger Mode)...");

  // Configure both alert pins as inputs with pulldown resistors
  pinMode(TEMP_ALERT_PIN, INPUT_PULLDOWN);
  pinMode(HUMIDITY_ALERT_PIN, INPUT_PULLDOWN);

  // ===================================================================
  // === STARTUP TEST: Wait for signals from both GPIO pins ===
  // ===================================================================
  bool testOk = runStartupTest();
  if (!testOk) {
    Serial.println("HALTING. Please check wiring between master and slave, then reset both devices.");
    while(true) { delay(1000); } // Halt execution
  }
  
  // Test passed, now proceed with normal (and slow) setup.
  Serial.println("\nConnection test passed. Initializing Power and SIM Module...");

  // Initialize Power Management and SIM800L
  initPowerManagement();
  initSIM800L();

  Serial.println("\nSetup Complete.");
  Serial.println("Waiting for alert signals:");
  Serial.println("  - Temperature alerts on GPIO " + String(TEMP_ALERT_PIN));
  Serial.println("  - Humidity alerts on GPIO " + String(HUMIDITY_ALERT_PIN));
}

bool runStartupTest() {
  Serial.println("----------------------------------------------");
  Serial.println("PERFORMING STARTUP TEST...");
  Serial.println("Waiting for test signals from master...");
  Serial.println("  - Temperature test on GPIO " + String(TEMP_ALERT_PIN));
  Serial.println("  - Humidity test on GPIO " + String(HUMIDITY_ALERT_PIN));
  
  bool tempTestReceived = false;
  bool humidityTestReceived = false;
  
  unsigned long startTime = millis();
  while (millis() - startTime < STARTUP_TEST_TIMEOUT_MS) {
    // Check temperature alert pin
    if (!tempTestReceived && digitalRead(TEMP_ALERT_PIN) == HIGH) {
      Serial.println("SUCCESS: Temperature test signal received!");
      tempTestReceived = true;
      // Wait for the pulse to end
      while(digitalRead(TEMP_ALERT_PIN) == HIGH) {
        delay(50);
      }
    }
    
    // Check humidity alert pin
    if (!humidityTestReceived && digitalRead(HUMIDITY_ALERT_PIN) == HIGH) {
      Serial.println("SUCCESS: Humidity test signal received!");
      humidityTestReceived = true;
      // Wait for the pulse to end
      while(digitalRead(HUMIDITY_ALERT_PIN) == HIGH) {
        delay(50);
      }
    }
    
    // If both tests received, we're good
    if (tempTestReceived && humidityTestReceived) {
      Serial.println("Both test signals received successfully!");
      return true;
    }
    
    delay(10);
  }
  
  Serial.println("ERROR: Timed out waiting for test signals.");
  if (!tempTestReceived) {
    Serial.println("Missing: Temperature test signal on GPIO " + String(TEMP_ALERT_PIN));
    Serial.println("Check connection: Master GPIO 10 to Slave GPIO " + String(TEMP_ALERT_PIN));
  }
  if (!humidityTestReceived) {
    Serial.println("Missing: Humidity test signal on GPIO " + String(HUMIDITY_ALERT_PIN));
    Serial.println("Check connection: Master GPIO 11 to Slave GPIO " + String(HUMIDITY_ALERT_PIN));
  }
  Serial.println("Ensure a common GND wire is connected between both boards.");
  return false;
}

void loop() {
  // === CORE LOGIC: Check for temperature alert signal ===
  if (digitalRead(TEMP_ALERT_PIN) == HIGH) {
    Serial.println("!!! TEMPERATURE alert signal received from master device on GPIO " + String(TEMP_ALERT_PIN) + " !!!");
    handleTemperatureAlert();
    
    // Wait for the signal to go away to prevent multiple SMS for one event
    Serial.println("Waiting for TEMPERATURE signal to return to LOW...");
    while(digitalRead(TEMP_ALERT_PIN) == HIGH) {
      delay(100);
    }
    Serial.println("TEMPERATURE signal is LOW. Ready for next alert.");
  }
  
  // === CORE LOGIC: Check for humidity alert signal ===
  if (digitalRead(HUMIDITY_ALERT_PIN) == HIGH) {
    Serial.println("!!! HUMIDITY alert signal received from master device on GPIO " + String(HUMIDITY_ALERT_PIN) + " !!!");
    handleHumidityAlert();
    
    // Wait for the signal to go away to prevent multiple SMS for one event
    Serial.println("Waiting for HUMIDITY signal to return to LOW...");
    while(digitalRead(HUMIDITY_ALERT_PIN) == HIGH) {
      delay(100);
    }
    Serial.println("HUMIDITY signal is LOW. Ready for next alert.");
  }
  
  // Periodically check SIM800L status
  if (millis() - lastStatusCheck > STATUS_CHECK_INTERVAL) {
    lastStatusCheck = millis();
    checkSIM800LStatus();
  }

  // A small delay to keep the loop from running too fast
  delay(100);
}

// Handles the temperature SMS alert process
void handleTemperatureAlert() {
  // Check cooldown period for temperature alerts
  if (millis() - lastTempSMSAttempt < SMS_RETRY_INTERVAL) {
    Serial.println("TEMPERATURE SMS cooldown active, skipping SMS send.");
    return;
  }
  
  Serial.println("Processing TEMPERATURE SMS alert...");
  
  // Create temperature-specific SMS message
  String smsMessage = "TEMPERATURE ALERT! The temperature sensor on your monitoring system has breached the set thresholds. Please check the system immediately.";
  
  // Attempt to send SMS
  bool smsSuccess = sendSMS(smsMessage, phoneNumber);
  
  if (smsSuccess) {
    Serial.println("TEMPERATURE SMS alert sent successfully!");
  } else {
    Serial.println("Failed to send TEMPERATURE SMS alert.");
  }
  
  lastTempSMSAttempt = millis(); // Update the timestamp
}

// Handles the humidity SMS alert process
void handleHumidityAlert() {
  // Check cooldown period for humidity alerts
  if (millis() - lastHumiditySMSAttempt < SMS_RETRY_INTERVAL) {
    Serial.println("HUMIDITY SMS cooldown active, skipping SMS send.");
    return;
  }
  
  Serial.println("Processing HUMIDITY SMS alert...");
  
  // Create humidity-specific SMS message
  String smsMessage = "HUMIDITY ALERT! The humidity sensor on your monitoring system has breached the set thresholds. Please check the system immediately.";
  
  // Attempt to send SMS
  bool smsSuccess = sendSMS(smsMessage, phoneNumber);
  
  if (smsSuccess) {
    Serial.println("HUMIDITY SMS alert sent successfully!");
  } else {
    Serial.println("Failed to send HUMIDITY SMS alert.");
  }
  
  lastHumiditySMSAttempt = millis(); // Update the timestamp
}

// --- SIM800L and Power Helper Functions (No changes needed below this line) ---

void initPowerManagement() {
  Serial.println("Initializing power management...");
  
  // Initialize I2C for power management chip
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);
  
  // Keep power boost on when running from battery
  bool isOk = setPowerBoostKeepOn(1);
  Serial.println("IP5306 KeepOn: " + String(isOk ? "OK" : "FAIL"));
  
  if (!isOk) {
    Serial.println("Warning: Power management initialization failed");
  }
}

bool setPowerBoostKeepOn(int en) {
  Wire.beginTransmission(IP5306_ADDR);
  Wire.write(IP5306_REG_SYS_CTL0);
  if (en) {
    Wire.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
  } else {
    Wire.write(0x35); // 0x37 is default reg value
  }
  return Wire.endTransmission() == 0;
}

void initSIM800L() {
  Serial.println("Starting SIM800L initialization...");

  // Configure SIM800L power and control pins
  pinMode(SIM800L_POWER, OUTPUT);
  pinMode(SIM800L_PWKEY, OUTPUT);
  pinMode(SIM800L_RST, OUTPUT);
  
  // Power cycle sequence for SIM800L
  digitalWrite(SIM800L_PWKEY, LOW);
  digitalWrite(SIM800L_RST, LOW);
  digitalWrite(SIM800L_POWER, HIGH);
  delay(1000);
  digitalWrite(SIM800L_RST, HIGH);
  delay(1000);
  
  // Power on pulse
  digitalWrite(SIM800L_PWKEY, HIGH);
  delay(1000);
  digitalWrite(SIM800L_PWKEY, LOW);
  delay(3000);
  
  // Initialize SIM800L serial communication
  sim800l.begin(115200, SERIAL_8N1, SIM800L_RX, SIM800L_TX);
  delay(3000);
  
  // Send AT command to check if module is responding
  int retries = 10;
  bool moduleReady = false;
  while (retries > 0 && !moduleReady) {
    sim800l.println("AT");
    String response = waitForResponse(1000);
    if (response.indexOf("OK") != -1) {
      Serial.println("SIM800L is responding.");
      moduleReady = true;
    } else {
      Serial.println("SIM800L not responding, retrying...");
      retries--;
      delay(1000);
    }
  }

  if (!moduleReady) {
    Serial.println("CRITICAL: Failed to initialize SIM800L module!");
    sim800lReady = false;
    return;
  }
  
  sim800l.println("ATE0"); // Disable echo
  waitForResponse(1000);
  
  // Check SIM card status
  sim800l.println("AT+CPIN?");
  String response = waitForResponse(5000);
  if (response.indexOf("READY") == -1) {
    Serial.println("ERROR: SIM card not ready or not inserted.");
    sim800lReady = false;
    return;
  }
  Serial.println("SIM card is ready.");

  // Wait for network registration
  Serial.println("Waiting for network registration...");
  retries = 30; // Wait up to 30 seconds
  while (retries > 0) {
    sim800l.println("AT+CREG?");
    response = waitForResponse(1000);
    if (response.indexOf(",1") != -1 || response.indexOf(",5") != -1) {
      networkRegistered = true;
      break;
    }
    retries--;
    delay(1000);
  }
  
  if (!networkRegistered) {
    Serial.println("CRITICAL: Failed to register on network!");
    sim800lReady = false;
    return;
  }
  Serial.println("Network registration successful.");

  // Set SMS text mode
  sim800l.println("AT+CMGF=1");
  response = waitForResponse(1000);
  if (response.indexOf("OK") == -1) {
    Serial.println("ERROR: Failed to set SMS text mode.");
    sim800lReady = false;
    return;
  }
  Serial.println("SMS text mode set successfully.");
  
  sim800lReady = true;
  Serial.println("SIM800L initialization complete!");
}

void checkSIM800LStatus() {
  if (!sim800lReady) {
    Serial.println("SIM800L not ready, skipping status check.");
    return;
  }
  
  sim800l.println("AT+CREG?");
  String response = waitForResponse(2000);
  
  if (response.indexOf(",1") != -1 || response.indexOf(",5") != -1) {
    if (!networkRegistered) Serial.println("Network re-acquired.");
    networkRegistered = true;
  } else {
    if (networkRegistered) Serial.println("WARNING: Network registration lost.");
    networkRegistered = false;
  }
  sim800lReady = networkRegistered;
}

String waitForResponse(unsigned long timeout) {
  String response = "";
  unsigned long startTime = millis();
  while (millis() - startTime < timeout) {
    if (sim800l.available()) {
      response += sim800l.readString();
    }
  }
  response.trim();
  return response;
}

bool sendSMS(String message, String number) {
  if (!sim800lReady || !networkRegistered) {
    Serial.println("ERROR: Cannot send SMS, SIM module not ready or not registered.");
    return false;
  }
  
  Serial.println("=== Sending SMS ===");
  Serial.println("To: " + number);
  
  sim800l.print("AT+CMGS=\"");
  sim800l.print(number);
  sim800l.println("\"");
  
  String response = waitForResponse(5000);
  if (response.indexOf(">") == -1) {
    Serial.println("ERROR: Failed to get SMS prompt '>'");
    Serial.println("Response: " + response);
    return false;
  }
  
  Serial.println("Got prompt, sending message...");
  sim800l.print(message);
  delay(100);
  sim800l.write(26); // Send Ctrl+Z to end message
  
  response = waitForResponse(20000); // Wait up to 20 seconds for send confirmation
  if (response.indexOf("+CMGS:") != -1 && response.indexOf("OK") != -1) {
    return true;
  } else {
    Serial.println("ERROR: SMS sending failed or timed out.");
    Serial.println("Response: " + response);
    return false;
  }
}