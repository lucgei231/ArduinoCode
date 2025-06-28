#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <NetworkUdp.h>
#include <ArduinoOTA.h>
#include <FastLED.h>

// Wi-Fi credentials
const char* ssid = "mypotato";
const char* password = "mypotato";

// Create a WebServer object on port 80
WebServer server(80);

// General Variables
bool ultrasonicAutoStopping = true;
float currentDistance = 0;
String serialLog = "";

// Speed Variables
const int minSpeed = 50;
const int maxSpeed = 255;

// Ultrasonic Distance Variables
const int maxDistance = 100;
const int stopDistance = 5;

// --- New Feature Variables ---
int lightPin = 34; // Use onboard LED for lights
bool lightsOn = true;
int buzzerPin = 19;
int lastDistances[10] = {0};
int distanceIndex = 0;
int manualSpeed = 150;
bool manualMode = false;
unsigned long bootMillis = 0; // Feature 3: Uptime tracking

// LED Variables
#define LED_PIN 34
#define NUM_LEDS 12
CRGB leds[NUM_LEDS];
uint8_t runningPos = 0;
CRGB currentColor = CRGB::Red;

// Feature 9: Function to clear distance log
void clearDistanceLog() {
  for (int i = 0; i < 10; i++) lastDistances[i] = 0;
  distanceIndex = 0;
}

// Ultrasonic distance function
float ultraDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2.0;
  delay(10);
  return distance;
}

// Motor control functions
void stopMotors() {
  digitalWrite(25, LOW);
  digitalWrite(26, LOW);
  analogWrite(33, 0);
  digitalWrite(12, LOW);
  digitalWrite(14, LOW);
  analogWrite(13, 0);
}

void startMotors(int gospeed) {
  digitalWrite(26, LOW);
  digitalWrite(25, HIGH);
  analogWrite(33, gospeed);
  digitalWrite(12, HIGH);
  digitalWrite(14, LOW);
  analogWrite(13, gospeed);
}

void moveBackwards(int backspeed) {
  digitalWrite(25, LOW);
  digitalWrite(26, HIGH);
  analogWrite(33, backspeed);
  digitalWrite(12, LOW);
  digitalWrite(14, HIGH);
  analogWrite(13, backspeed - 20);

}

void turnLeft() {
  digitalWrite(26, HIGH);
  digitalWrite(25, LOW);
  analogWrite(33, 200);
  digitalWrite(12, HIGH);
  digitalWrite(14, LOW);
  analogWrite(13, 200);
  delay(500);
  stopMotors();
}

void turnRight() {
  digitalWrite(26, LOW);
  digitalWrite(25, HIGH);
  analogWrite(33, 200);
  digitalWrite(12, LOW);
  digitalWrite(14, HIGH);
  analogWrite(13, 200);
  delay(500);
  stopMotors();
}

// Remove BitFlash config, callback, and object definition

// --- New Feature Functions ---
void toggleLights() {
  lightsOn = !lightsOn;
  digitalWrite(lightPin, lightsOn ? HIGH : LOW);
}

void beepBuzzer(int ms = 200) {
  digitalWrite(buzzerPin, HIGH);
  delay(ms);
  digitalWrite(buzzerPin, LOW);
}

void logDistance(int d) {
  lastDistances[distanceIndex] = d;
  distanceIndex = (distanceIndex + 1) % 10;
}

void printStatus() {
  Serial.print("Status: ");
  Serial.print(lightsOn ? "Lights ON, " : "Lights OFF, ");
  Serial.print("Speed: ");
  Serial.print(manualSpeed);
  Serial.print(", AutoStop: ");
  Serial.print(ultrasonicAutoStopping ? "ON" : "OFF");
  Serial.print(", ManualMode: ");
  Serial.println(manualMode ? "ON" : "OFF");
}

void testMotors() {
  Serial.println("Testing motors: Forward");
  startMotors(manualSpeed); delay(500); stopMotors(); delay(200);
  Serial.println("Backward");
  moveBackwards(manualSpeed); delay(500); stopMotors(); delay(200);
  Serial.println("Left");
  turnLeft(); delay(200);
  Serial.println("Right");
  turnRight(); delay(200);
  stopMotors();
  Serial.println("Motors test complete.");
}

void runningLed() {
  for (int i = 0; i < NUM_LEDS; i++) leds[i] = CRGB::Black;
  leds[runningPos] = currentColor;
  FastLED.show();
  runningPos = (runningPos + 1) % NUM_LEDS;
}

void setAllLeds(CRGB color) {
  for (int i = 0; i < NUM_LEDS; i++) leds[i] = color;
  FastLED.show();
}

// --- Web Server Handlers ---
void handleRoot() {
  String html = "<html><head><title>ESP32 Robot</title></head><body>";
  html += "<h1>ESP32 Robot Control</h1>";
  html += "<p>Distance: " + String(currentDistance) + " cm</p>";
  html += "<p>Speed: " + String(manualSpeed) + "</p>"; // Feature 4
  html += "<p>WiFi RSSI: " + String(WiFi.RSSI()) + " dBm</p>"; // Feature 2
  html += "<p>Uptime: " + String((millis() - bootMillis) / 1000) + " seconds</p>"; // Feature 3
  html += "<p>AutoStop: " + String(ultrasonicAutoStopping ? "ON" : "OFF") + "</p>"; // Feature 5
  html += "<p>Mode: " + String(manualMode ? "Manual" : "Auto") + "</p>"; // Feature 7
  html += "<form method='POST' action='/cmd'><button name='cmd' value='forward'>Forward</button> <button name='cmd' value='back'>Back</button> <button name='cmd' value='left'>Left</button> <button name='cmd' value='right'>Right</button> <button name='cmd' value='stop'>Stop</button> <button name='cmd' value='spin'>Spin</button></form>";
  html += "<form method='POST' action='/autostop'><button>";
  html += (ultrasonicAutoStopping ? "Disable AutoStop" : "Enable AutoStop");
  html += "</button></form>";
  html += "<form method='POST' action='/testmotors'><button>Test Motors</button></form>";
  html += "<form method='POST' action='/status'><button>Status</button></form>";
  html += "<form method='POST' action='/reset'><button>Reset</button></form>";
  html += "<form method='POST' action='/setspeed'><input name='speed' type='number' min='50' max='255' value='" + String(manualSpeed) + "'><button>Set Speed</button></form>";
  html += "<form method='POST' action='/togglemode'><button>";
  html += (manualMode ? "Switch to Auto Mode" : "Switch to Manual Mode"); // Feature 8
  html += "</button></form>";
  html += "<form method='POST' action='/cleardistances'><button>Clear Distance Log</button></form>"; // Feature 10
  html += "<a href='/distances'>Show Last 10 Distances</a> | ";
  html += "<a href='/leds'>LED Control</a>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleCmd() {
  if (server.method() == HTTP_POST) {
    String cmd = server.arg("cmd");
    manualMode = true;
    if (cmd == "forward") startMotors(manualSpeed);
    else if (cmd == "back") moveBackwards(manualSpeed);
    else if (cmd == "left") turnLeft();
    else if (cmd == "right") turnRight();
    else if (cmd == "stop") stopMotors();
    else if (cmd == "spin") {
      digitalWrite(25, HIGH); digitalWrite(26, LOW); analogWrite(33, 255);
      digitalWrite(12, LOW); digitalWrite(14, HIGH); analogWrite(13, 255);
      delay(1500); stopMotors();
    }
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleLights() {
  toggleLights();
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleAutoStop() {
  ultrasonicAutoStopping = !ultrasonicAutoStopping;
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleBeep() {
  beepBuzzer();
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleTestMotors() {
  testMotors();
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleStatus() {
  String s = "Status: ";
  s += (lightsOn ? "Lights ON, " : "Lights OFF, ");
  s += "Speed: ";
  s += String(manualSpeed);
  s += ", AutoStop: ";
  s += (ultrasonicAutoStopping ? "ON" : "OFF");
  s += ", ManualMode: ";
  s += (manualMode ? "ON" : "OFF");
  server.send(200, "text/plain", s);
}

void handleReset() {
  server.send(200, "text/plain", "Rebooting...");
  delay(1000);
  ESP.restart();
}

void handleSetSpeed() {
  if (server.method() == HTTP_POST) {
    int s = server.arg("speed").toInt();
    if (s >= 50 && s <= 255) manualSpeed = s;
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleToggleMode() {
  manualMode = !manualMode;
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleClearDistances() {
  clearDistanceLog();
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleDistances() {
  String html = "<html><body><h2>Last 10 Distances (cm)</h2><ul>";
  for (int i = 0; i < 10; i++) {
    int idx = (distanceIndex + i) % 10;
    html += "<li>" + String(lastDistances[idx]) + "</li>";
  }
  html += "</ul><a href='/'>Back</a></body></html>";
  server.send(200, "text/html", html);
}

void handleLeds() {
  if (server.method() == HTTP_POST) {
    String color = server.arg("color");
    long number = strtol(color.c_str()+1, NULL, 16); // skip '#'
    currentColor = CRGB((number >> 16) & 0xFF, (number >> 8) & 0xFF, number & 0xFF);
    setAllLeds(currentColor);
  }
  String html = "<html><body>";
  html += "<h2>LED Control</h2>";
  html += "<form method='POST'><input type='color' name='color' value='#";
  char buf[7];
  sprintf(buf, "%02X%02X%02X", currentColor.r, currentColor.g, currentColor.b);
  html += buf;
  html += "'><button>Set Color</button></form>";
  html += "<form method='POST' action='/ledrun'><button>Start Running Light</button></form>";
  html += "<a href='/'>Back</a></body></html>";
  server.send(200, "text/html", html);
}

void handleLedRun() {
  unsigned long start = millis();
  while (millis() - start < 5000) { // Run for 5 seconds
    runningLed();
    delay(100);
    server.handleClient();
  }
  setAllLeds(currentColor);
  server.sendHeader("Location", "/leds");
  server.send(303);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");
  
  // Pin modes
  pinMode(0, INPUT_PULLUP);
  pinMode(2, OUTPUT);
  pinMode(15, INPUT);
  pinMode(33, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(19, OUTPUT);
  pinMode(21, INPUT);
  pinMode(32, OUTPUT);
  pinMode(35, INPUT);
  pinMode(lightPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(lightPin, LOW);
  digitalWrite(buzzerPin, LOW);

  digitalWrite(27, HIGH);

  // Connect to Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("Connected to Wi-Fi: ");
  Serial.println(ssid);
  Serial.println("Wi-Fi connected!");
  Serial.print("Web server is running at: http://");
  Serial.println(WiFi.localIP());

  // Setup LED strip
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  FastLED.clear();
  FastLED.show();

  // Register new routes for features
  server.on("/togglemode", HTTP_POST, handleToggleMode); // Feature 8
  server.on("/cleardistances", HTTP_POST, handleClearDistances); // Feature 10
  server.on("/distances", HTTP_GET, handleDistances); // Register /distances endpoint
  server.on("/leds", HTTP_GET, handleLeds);
  server.on("/leds", HTTP_POST, handleLeds);
  server.on("/ledrun", HTTP_POST, handleLedRun);
  server.on("/", HTTP_GET, handleRoot); // Ensure root handler is registered
  server.on("/lights", HTTP_POST, handleLights);
  server.on("/autostop", HTTP_POST, handleAutoStop);
  server.on("/beep", HTTP_POST, handleBeep);
  server.on("/testmotors", HTTP_POST, handleTestMotors);
  server.on("/status", HTTP_POST, handleStatus);
  server.on("/reset", HTTP_POST, handleReset);
  server.on("/setspeed", HTTP_POST, handleSetSpeed);
  server.on("/cmd", HTTP_POST, handleCmd);

  server.begin(); // Start server after all routes are registered

  // Setup Arduino OTA
  ArduinoOTA
    .onStart([]() {
      Serial.println("Start updating...");
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
    });

  ArduinoOTA.begin();
  
  Serial.println("Ready for OTA updates!");

  bootMillis = millis(); // Feature 3: Track uptime
}

void loop() {
  server.handleClient();
  ArduinoOTA.handle();
  runningLed(); // Always animate running LED

  // --- Serial command handling ---
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.equalsIgnoreCase("stop")) {
      Serial.println("Received 'stop' command. Stopping motors for 15 seconds.");
      stopMotors();
      delay(15000);
      Serial.println("Resuming motors.");
    } else if (cmd.equalsIgnoreCase("spin")) {
      Serial.println("Received 'spin' command. Spinning for 15 seconds at max speed.");
      digitalWrite(25, HIGH); digitalWrite(26, LOW); analogWrite(33, 255);
      digitalWrite(12, LOW); digitalWrite(14, HIGH); analogWrite(13, 255);
      delay(15000); stopMotors();
      Serial.println("Spin complete.");
    } else if (cmd.equalsIgnoreCase("lights")) {
      toggleLights();
      Serial.println(lightsOn ? "Lights ON" : "Lights OFF");
    } else if (cmd.equalsIgnoreCase("beep")) {
      beepBuzzer();
      Serial.println("Beep!");
    } else if (cmd.equalsIgnoreCase("status")) {
      printStatus();
    } else if (cmd.equalsIgnoreCase("setspeed")) {
      Serial.println("Enter speed (50-255):");
      while (!Serial.available());
      int s = Serial.readStringUntil('\n').toInt();
      if (s >= 50 && s <= 255) {
        manualSpeed = s;
        Serial.print("Speed set to "); Serial.println(manualSpeed);
      } else {
        Serial.println("Invalid speed.");
      }
    } else if (cmd.equalsIgnoreCase("reset")) {
      Serial.println("Rebooting...");
      delay(1000);
      ESP.restart();
    } else if (cmd.equalsIgnoreCase("autostop")) {
      ultrasonicAutoStopping = !ultrasonicAutoStopping;
      Serial.print("AutoStop: ");
      Serial.println(ultrasonicAutoStopping ? "ON" : "OFF");
    } else if (cmd.equalsIgnoreCase("testmotors")) {
      testMotors();
    }
  }
  // --- End serial command handling ---

  float frontDistance = ultraDistance(2, 15);
  currentDistance = frontDistance;
  logDistance((int)frontDistance);

  if (frontDistance > stopDistance) {
    moveBackwards(200);
  } else {
    stopMotors();
    Serial.println("Blocked! Checking for open paths...");
    float leftDistance = ultraDistance(19, 21);
    float rightDistance = ultraDistance(32, 35);
    
    if (leftDistance > rightDistance && leftDistance > stopDistance) {
      turnLeft();
    } else if (rightDistance > leftDistance && rightDistance > stopDistance) {
      turnRight();
    } else {
      startMotors(150);
      delay(500);
    }
  }
}