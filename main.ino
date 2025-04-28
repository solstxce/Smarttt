#include <Arduino.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <PulseSensorPlayground.h>     // Includes the PulseSensorPlayground Library

// WiFi credentials
const char* ssid = "Madani";
const char* password = "123456789";
const char* serverIP = "144.126.254.154";
const int serverPort = 8085;

// Pin definitions for Arduino Nano
#define BUZZER_VCC 8
#define BUZZER_SIGNAL 7
#define GPS_RX 4
#define GPS_TX 3
#define ESP_RX 5  // Connect to ESP TX
#define ESP_TX 6  // Connect to ESP RX
#define PULSE_SENSOR A0  // Analog pin for pulse sensor
#define EMERGENCY_BUTTON 2  // Digital pin for emergency button

// Constants for pulse sensor
const int THRESHOLD = 550;  // Adjust this value based on your sensor readings

// Button debounce variables
bool lastButtonState = HIGH;  // Assuming pull-up resistor (button open = HIGH)
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;  // Debounce time in milliseconds
bool emergencyTriggered = false;
unsigned long emergencyCooldown = 10000;  // 30 seconds cooldown between alerts
unsigned long lastEmergencyTime = 0;

// Objects
TinyGPSPlus gps;
SoftwareSerial serialAlt(ESP_TX, ESP_RX); // We'll switch between GPS and ESP
PulseSensorPlayground pulseSensor;  // Creates a PulseSensorPlayground object

// Variables
float latitude = 9.574687;
float longitude = 77.679812;
unsigned long lastReadTime = 0;
const long readInterval = 1000;
bool wifiConnected = false;
float heartRate = 0;

void switchToGPS() {
    serialAlt.end();
    serialAlt = SoftwareSerial(GPS_TX, GPS_RX);
    serialAlt.begin(9600);
}

void switchToESP() {
    serialAlt.end();
    serialAlt = SoftwareSerial(ESP_TX, ESP_RX);
    serialAlt.begin(115200);
}

void clearSerialAlt() {
    while (serialAlt.available()) {
        serialAlt.read();
    }
}

void setupESP8266() {
    Serial.println("Initializing ESP8266...");
    switchToESP();
    
    // Reset ESP8266
    serialAlt.println("AT+RST");
    delay(2000);
    clearSerialAlt();
    
    // Set ESP8266 mode to station
    serialAlt.println("AT+CWMODE=1");
    delay(1000);
    clearSerialAlt();
    
    // Connect to WiFi
    String connectCmd = "AT+CWJAP=\"";
    connectCmd += ssid;
    connectCmd += "\",\"";
    connectCmd += password;
    connectCmd += "\"";
    
    Serial.println("Connecting to WiFi...");
    serialAlt.println(connectCmd);
    
    // Wait for connection
    int timeout = 20; // 20 seconds timeout
    while (timeout > 0) {
        if (serialAlt.find("OK")) {
            wifiConnected = true;
            Serial.println("WiFi Connected!");
            return;
        }
        delay(1000);
        timeout--;
    }
    
    Serial.println("Failed to connect to WiFi");
    wifiConnected = false;
}

void sendDataToServer(bool isRetry = false) {
    if (!wifiConnected) return;
    
    switchToESP();
    
    const int maxRetries = 3;  // Maximum number of retry attempts
    int currentRetry = 0;
    bool sendSuccess = false;

    while (!sendSuccess && currentRetry < maxRetries) {
        String cmd = "AT+CIPSTART=\"TCP\",\"";
        cmd += serverIP;
        cmd += "\",";
        cmd += serverPort;
        serialAlt.println(cmd);
        delay(500);
        
        if (serialAlt.find("ERROR")) {
            Serial.print("TCP Connection Error. Attempt ");
            Serial.print(currentRetry + 1);
            Serial.print(" of ");
            Serial.println(maxRetries);
            currentRetry++;
            
            if (currentRetry < maxRetries) {
                delay(1000 * currentRetry);  // Exponential backoff
                continue;
            } else {
                Serial.println("Failed to establish TCP connection after all retries");
                return;
            }
        } else {
            sendSuccess = true;
        }
    }
    
    // Prepare HTTP GET request
    String url = "GET /update?HR=";
    url += String(heartRate, 1);
    url += "&lat=";
    url += String(latitude, 6);
    url += "&lon=";
    url += String(longitude, 6);
    
    // Add emergency flag if triggered
    if (emergencyTriggered) {
        url += "&emergency=true";
        // Only clear emergency flag if send is successful
        if (!isRetry) {
            emergencyTriggered = false;
        }
    }
    
    url += " HTTP/1.1\r\nHost: ";
    url += serverIP;
    url += "\r\n\r\n";
    
    // Send data length
    cmd = "AT+CIPSEND=";
    cmd += url.length();
    serialAlt.println(cmd);
    delay(100);
    
    // Send data
    serialAlt.print(url);
    delay(1000);
    
    // Check if data was sent successfully
    if (serialAlt.find("SEND OK")) {
        Serial.println("Data sent successfully");
        if (emergencyTriggered && isRetry) {
            emergencyTriggered = false;
        }
    } else if (!isRetry) {
        Serial.println("Failed to send data, retrying...");
        sendDataToServer(true);  // Retry once
    }
    
    clearSerialAlt();
}

void checkEmergencyButton() {
    // Read current button state
    int reading = digitalRead(EMERGENCY_BUTTON);
    
    // Check if the button state has changed
    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }
    
    // If the button state has been stable for longer than the debounce delay
    if ((millis() - lastDebounceTime) > debounceDelay) {
        // If the button is pressed (LOW with pull-up resistor)
        if (reading == LOW && !emergencyTriggered) {
            // Check cooldown period
            if (millis() - lastEmergencyTime > emergencyCooldown) {
                Serial.println("EMERGENCY BUTTON PRESSED!");
                
                // Provide feedback with buzzer
                for (int i = 0; i < 3; i++) {
                    digitalWrite(BUZZER_SIGNAL, HIGH);
                    delay(200);
                    digitalWrite(BUZZER_SIGNAL, LOW);
                    delay(200);
                }
                
                emergencyTriggered = true;
                lastEmergencyTime = millis();
                
                // Send emergency alert immediately with retry capability
                sendDataToServer(false);
            } else {
                Serial.println("Emergency button in cooldown period");
            }
        }
    }
    
    lastButtonState = reading;
}

void setup() {
    // Initialize serial communications
    Serial.begin(57600);
    Serial.println("\nStarting setup...");
    
    // Initialize pins
    pinMode(BUZZER_VCC, OUTPUT);
    pinMode(BUZZER_SIGNAL, OUTPUT);
    digitalWrite(BUZZER_VCC, HIGH);
    pinMode(PULSE_SENSOR, INPUT);
    pinMode(EMERGENCY_BUTTON, INPUT_PULLUP);

    // Configure and initialize the PulseSensor object
    pulseSensor.analogInput(PULSE_SENSOR);   
    pulseSensor.setThreshold(THRESHOLD);   
    if (pulseSensor.begin()) {
        Serial.println("PulseSensor object created!");
    } else {
        Serial.println("Failed to create PulseSensor object!");
        // Handle error, perhaps loop forever or reset
        while(1); 
    }
    
    // Setup ESP8266 first
    setupESP8266();
    
    // Then switch to GPS mode
    switchToGPS();
    
    Serial.println("Setup complete. Monitoring started.");
}

void loop() {
    // Update heart rate using the library
    heartRate = pulseSensor.getBeatsPerMinute();

    // Check emergency button
    checkEmergencyButton();
    
    if (millis() - lastReadTime > readInterval) {
        // Get GPS data
        switchToGPS();
        while (serialAlt.available()) {
            if (gps.encode(serialAlt.read())) {
                if (gps.location.isValid()) {
                    latitude = gps.location.lat();
                    longitude = gps.location.lng();
                }
            }
        }
        
        // Send data to server if not already sent by emergency
        if (!emergencyTriggered) {
            sendDataToServer();
        }
        
        // Switch back to GPS
        switchToGPS();
        
        // Print data for debugging
        Serial.print("Heart rate: ");
        Serial.print(heartRate, 1);
        Serial.print(" bpm / Location: ");
        Serial.print(latitude, 6);
        Serial.print(", ");
        Serial.println(longitude, 6);
        
        lastReadTime = millis();
    }
}
