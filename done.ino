#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <FirebaseESP8266.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <SPI.h>
#include <MFRC522.h>

// WiFi credentials
const char* wifiSSID = "eng_afandyna";
const char* wifiPassword = "M#201677MM#m";

// Firebase details
#define API_KEY "AIzaSyA8F8EODsT2p26ofkxHQCAA2emUxUKpA9A"
#define DATABASE_URL "https://special-a6146-default-rtdb.firebaseio.com/"

// GPS Pins
static const int RXPin = 16;  // D0 (GPIO16)
static const int TXPin = 0;   // D3 (GPIO0)
static const uint32_t GPSBaud = 9600;

// RFID Pins
#define SS_PIN 15  // D8 (GPIO15)
#define RST_PIN 5  // D1 (GPIO5)

// Push button pin
const int buttonPin = A0;  // Analog pin ADC0

// Built-in LED pin
const int ledPin = 2;  // D4 (GPIO2, active-low)

// Buzzer pin
const int buzzerPin = 4;  // D2 (GPIO4)

// NTP for accurate time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000);  // UTC, update every 60s

SoftwareSerial gpsSerial(RXPin, TXPin);
TinyGPSPlus gps;
FirebaseData fbdo;
FirebaseData stream;
FirebaseAuth auth;
FirebaseConfig config;
MFRC522 mfrc522(SS_PIN, RST_PIN);  // RFID object

unsigned long lastUploadTime = 0;
unsigned long lastBuzzerCheck = 0;
const unsigned long checkInterval = 2000;  // Check every 2 seconds
String macAddress;
bool buttonPressed = false;
unsigned long lastButtonCheck = 0;
const unsigned long buttonDebounce = 50;  // Fast debounce (50ms)
unsigned long ledOnTime = 0;
bool ledActive = false;
unsigned long lastRFIDScanTime = 0;
bool rfidScanned = false;
const unsigned long rfidTimeout = 3000;  // 3 seconds timeout for RFID
unsigned long buttonTrueTime = 0;  // Track when button was pressed
bool buttonTrueSent = false;  // Track if true was sent
String lastBuzzerCommand = "";  // Track last buzzer command

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPSBaud);
  pinMode(ledPin, OUTPUT);           // Built-in LED
  digitalWrite(ledPin, HIGH);        // LED off (active-low)
  pinMode(buzzerPin, OUTPUT);        // Buzzer
  digitalWrite(buzzerPin, LOW);      // Buzzer off

  // Test buzzer: single 1000Hz tone for 500ms
  tone(buzzerPin, 1000, 500);
  delay(1000);  // Wait 1 second
  Serial.println("Buzzer test completed");

  // Initialize SPI and RFID
  SPI.begin();
  mfrc522.PCD_Init();
  Serial.println("Ready to read RFID card. Present card...");

  // Get MAC address as unique ID
  macAddress = WiFi.macAddress();
  macAddress.replace(":", "");  // Remove colons for Firebase path
  Serial.println("MAC Address: " + macAddress);

  // Connect to WiFi
  connectToWiFi();

  // Initialize NTP
  timeClient.begin();

  // Firebase setup
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  Firebase.signUp(&config, &auth, "", "");  // Anonymous auth
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  // Initialize Firebase stream for buzzer commands
  String streamPath = "/devices/" + macAddress + "/buzzer";
  if (Firebase.beginStream(stream, streamPath)) {
    Serial.println("Buzzer stream started on path: " + streamPath);
  } else {
    Serial.println("Failed to start buzzer stream: " + stream.errorReason());
  }
}

void loop() {
  // Update GPS data
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      Serial.println("GPS data received: " + String(gps.location.isValid() ? "Valid" : "Invalid"));
      if (gps.location.isValid()) {
        Serial.println("Latitude: " + String(gps.location.lat(), 6));
        Serial.println("Longitude: " + String(gps.location.lng(), 6));
      }
    }
  }

  // Update time
  timeClient.update();

  // Check button with fast debounce
  if (millis() - lastButtonCheck > buttonDebounce) {
    lastButtonCheck = millis();
    int buttonValue = analogRead(buttonPin);
    Serial.println("Button value: " + String(buttonValue));
    if (buttonValue < 100) {  // Button pressed (active-low, pulled to GND)
      if (!buttonPressed) {  // New press
        buttonPressed = true;
        buttonTrueTime = millis();
        buttonTrueSent = false;
        uploadButtonPress(true);  // Send true immediately
        playButtonSound();       // Play button sound
      }
    } else if (buttonPressed) {  // Button released
      buttonPressed = false;
      if (buttonTrueSent && millis() - buttonTrueTime >= 5000) {
        uploadButtonPress(false);  // Send false after 5 seconds
        buttonTrueSent = false;
      }
    }
  }

  // Check if 5 seconds have passed since button press to send false
  if (buttonTrueSent && !buttonPressed && millis() - buttonTrueTime >= 5000) {
    uploadButtonPress(false);  // Send false after 5 seconds
    buttonTrueSent = false;
  }

  // Upload GPS data and check buzzer commands every 2 seconds
  if (millis() - lastUploadTime > checkInterval) {
    lastUploadTime = millis();
    uploadGPSData();
    checkBuzzerCommands();
  }

  // Check for RFID card
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    String uid = "";
    for (byte i = 0; i < mfrc522.uid.size; i++) {
      uid += (mfrc522.uid.uidByte[i] < 0x10 ? "0" : "");
      uid += String(mfrc522.uid.uidByte[i], HEX);
    }
    uid.toUpperCase();
    Serial.print("RFID Card UID: ");
    Serial.println(uid);
    uploadRFIDData(uid);  // Upload UID to Firebase
    playRFIDSound();      // Play RFID sound
    lastRFIDScanTime = millis();
    rfidScanned = true;
    mfrc522.PICC_HaltA();  // Stop reading card
  }

  // Send "none scanned" after 3 seconds
  if (rfidScanned && millis() - lastRFIDScanTime > rfidTimeout) {
    uploadRFIDData("none scanned");
    rfidScanned = false;
  }

  // Turn off LED after 2 seconds
  if (ledActive && millis() - ledOnTime > 2000) {
    digitalWrite(ledPin, HIGH);  // LED off
    ledActive = false;
  }
}

void connectToWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(wifiSSID, wifiPassword);
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 20000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected.");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect to WiFi.");
  }
}

void uploadGPSData() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, skipping GPS upload");
    return;
  }

  String path = "/devices/" + macAddress + "/gps";
  bool success = true;
  if (gps.location.isValid()) {
    success &= Firebase.setFloat(fbdo, path + "/latitude", gps.location.lat());
    success &= Firebase.setFloat(fbdo, path + "/longitude", gps.location.lng());
  } else {
    success &= Firebase.setFloat(fbdo, path + "/latitude", 0.0);
    success &= Firebase.setFloat(fbdo, path + "/longitude", 0.0);
    Serial.println("Invalid GPS data, uploading 0.0");
  }
  success &= Firebase.setString(fbdo, path + "/mac", macAddress);
  success &= Firebase.setString(fbdo, path + "/datetime", timeClient.getFormattedTime() + " " + String(timeClient.getEpochTime() / 86400));

  if (!success) {
    Serial.println("GPS upload failed: " + fbdo.errorReason());
  } else {
    Serial.println("GPS data uploaded successfully");
  }
}

void uploadButtonPress(bool state) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, skipping button upload");
    return;
  }

  String path = "/devices/" + macAddress + "/button";
  bool success = true;

  success &= Firebase.setBool(fbdo, path + "/state", state);
  if (gps.location.isValid()) {
    success &= Firebase.setFloat(fbdo, path + "/press_latitude", gps.location.lat());
    success &= Firebase.setFloat(fbdo, path + "/press_longitude", gps.location.lng());
  } else {
    success &= Firebase.setFloat(fbdo, path + "/press_latitude", 0.0);
    success &= Firebase.setFloat(fbdo, path + "/press_longitude", 0.0);
  }
  success &= Firebase.setString(fbdo, path + "/datetime", timeClient.getFormattedTime() + " " + String(timeClient.getEpochTime() / 86400));

  if (success && fbdo.errorReason() == "") {
    digitalWrite(ledPin, LOW);  // LED on (active-low)
    ledOnTime = millis();
    ledActive = true;
    if (state) {
      buttonTrueSent = true;  // Mark true as sent
    }
    Serial.println("Button state uploaded: " + String(state ? "true" : "false"));
  } else {
    Serial.println("Button upload failed: " + fbdo.errorReason());
  }
}

void uploadRFIDData(String uid) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, skipping RFID upload");
    return;
  }

  String path = "/devices/" + macAddress + "/rfid";
  bool success = true;

  success &= Firebase.setString(fbdo, path + "/uid", uid);
  success &= Firebase.setString(fbdo, path + "/datetime", timeClient.getFormattedTime() + " " + String(timeClient.getEpochTime() / 86400));
  if (gps.location.isValid()) {
    success &= Firebase.setFloat(fbdo, path + "/latitude", gps.location.lat());
    success &= Firebase.setFloat(fbdo, path + "/longitude", gps.location.lng());
  } else {
    success &= Firebase.setFloat(fbdo, path + "/latitude", 0.0);
    success &= Firebase.setFloat(fbdo, path + "/longitude", 0.0);
  }

  if (success && fbdo.errorReason() == "") {
    digitalWrite(ledPin, LOW);  // LED on (active-low)
    ledOnTime = millis();
    ledActive = true;
    Serial.println("RFID data uploaded: " + uid);
  } else {
    Serial.println("RFID upload failed: " + fbdo.errorReason());
  }
}

void checkBuzzerCommands() {
  if (!Firebase.ready()) {
    Serial.println("Firebase not ready, skipping buzzer check");
    return;
  }

  // Read current buzzer command
  String path = "/devices/" + macAddress + "/buzzer";
  if (Firebase.getString(fbdo, path)) {
    String command = fbdo.stringData();
    Serial.println("Received buzzer command: " + command);
    if (command == "true") {
      playTrueSound();
    } else if (command == "false") {
      playFalseSound();
    } else if (command == "error") {
      playErrorSound();
    } else if (command == "stop") {
      playStopSound();
    } else {
      Serial.println("Unknown command: " + command);
    }
    lastBuzzerCommand = command;  // Update last command
  } else {
    Serial.println("Failed to read buzzer command: " + fbdo.errorReason());
  }
}

void playRFIDSound() {
  tone(buzzerPin, 500, 200);  // 500Hz for 200ms
}

void playButtonSound() {
  tone(buzzerPin, 800, 100);  // 800Hz for 100ms
  delay(150);
  tone(buzzerPin, 800, 100);  // 800Hz for 100ms again
}

void playTrueSound() {
  tone(buzzerPin, 1000, 500);  // 1000Hz for 500ms
  Serial.println("Playing True sound");
}

void playFalseSound() {
  for (int i = 0; i < 3; i++) {
    tone(buzzerPin, 600, 100);  // 600Hz for 100ms
    delay(200);
  }
  Serial.println("Playing False sound");
}

void playErrorSound() {
  tone(buzzerPin, 300, 1000);  // 300Hz for 1000ms
  Serial.println("Playing Error sound");
}

void playStopSound() {
  digitalWrite(buzzerPin, LOW);  // Stop buzzer
  Serial.println("Buzzer stopped");
}