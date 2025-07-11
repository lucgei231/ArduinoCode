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
int lightPin = 23; // Use onboard LED for lights
bool lightsOn = true;
int buzzerPin = 19;
int lastDistances[10] = {0};
int distanceIndex = 0;
int manualSpeed = 150;
bool manualMode = false;
unsigned long bootMillis = 0; // Feature 3: Uptime tracking

// LED Variables
#define LED_PIN 23
#define NUM_LEDS 12
CRGB leds[NUM_LEDS];
uint8_t runningPos = 0;
CRGB currentColor = CRGB::Red;
CRGB backgroundColor = CRGB::Blue; // New: background color

// --- LED SIGNAL HELPERS ---
void flashLeftSignal() {
  // Now flashes RIGHT half for left turn
  for (int t = 0; t < 3; t++) {
    for (int i = 0; i < NUM_LEDS / 2; i++) leds[i] = CRGB::Black;
    for (int i = NUM_LEDS / 2; i < NUM_LEDS; i++) leds[i] = CRGB::Yellow;
    FastLED.show();
    delay(200);
    for (int i = 0; i < NUM_LEDS; i++) leds[i] = CRGB::Black;
    FastLED.show();
    delay(200);
  }
}

void flashRightSignal() {
  // Now flashes LEFT half for right turn
  for (int t = 0; t < 3; t++) {
    for (int i = 0; i < NUM_LEDS / 2; i++) leds[i] = CRGB::Yellow;
    for (int i = NUM_LEDS / 2; i < NUM_LEDS; i++) leds[i] = CRGB::Black;
    FastLED.show();
    delay(200);
    for (int i = 0; i < NUM_LEDS; i++) leds[i] = CRGB::Black;
    FastLED.show();
    delay(200);
  }
}

void showStopSignal() {
  for (int i = 0; i < NUM_LEDS; i++) leds[i] = CRGB::Red;
  FastLED.show();
}

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
  showStopSignal(); // <-- Add this line
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
  flashLeftSignal(); // <-- Add this line
  delay(250);
  stopMotors();
}

void turnRight() {
  digitalWrite(26, LOW);
  digitalWrite(25, HIGH);
  analogWrite(33, 200);
  digitalWrite(12, LOW);
  digitalWrite(14, HIGH);
  analogWrite(13, 200);
  flashRightSignal(); // <-- Add this line
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
  for (int i = 0; i < NUM_LEDS; i++)
    leds[i] = backgroundColor; // Use background color for all
  leds[runningPos] = currentColor; // Running LED is main color
  FastLED.show();
  runningPos = (runningPos + 1) % NUM_LEDS;
}

void runningGreenLed() {
  static uint8_t pos = 0;
  for (int i = 0; i < NUM_LEDS; i++) leds[i] = CRGB::Black;
  leds[pos] = CRGB::Green;
  FastLED.show();
  pos = (pos + 1) % NUM_LEDS;
  delay(80); // Adjust speed as desired
}

void setAllLeds(CRGB color, CRGB bgColor) {
  for (int i = 0; i < NUM_LEDS; i++)
    leds[i] = bgColor;
  FastLED.show();
}

// --- Add this function near your other LED functions ---
void setRainbow() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CHSV((i * 255) / NUM_LEDS, 255, 255);
  }
  FastLED.show();
}

// --- Web Server Handlers ---
void handleRoot() {
  String html = R"rawliteral(
  <html>
  <head>
    <title>ESP32 Robot</title>
    <meta http-equiv="refresh" content="0.5">
    <style>
      body { background: #f7f7f7; color: #222; font-family: Arial, sans-serif; margin: 0; padding: 0; }
      .container { max-width: 600px; margin: 40px auto; background: #fff; border-radius: 12px; padding: 32px; box-shadow: 0 4px 24px #0002; }
      h1, h2 { color: #1976d2; }
      button, input[type="submit"] { background: #1976d2; color: #fff; border: none; border-radius: 6px; padding: 10px 18px; font-size: 1em; margin: 6px 0; cursor: pointer; transition: background 0.2s; }
      button:hover, input[type="submit"]:hover { background: #1565c0; }
      input[type="color"], input[type="number"] { margin: 0 8px 0 0; }
      .row { margin-bottom: 16px; }
      a { color: #1976d2; text-decoration: none; }
      a:hover { text-decoration: underline; }
      .btn-row { display: flex; flex-wrap: wrap; gap: 8px; }
    </style>
  </head>
  <body>
    <div class="container">
      <h1>ESP32 Robot Control</h1>
      <div class="row">Distance: <b>)rawliteral" + String(currentDistance) + R"rawliteral( cm</b></div>
      <div class="row">Speed: <b>)rawliteral" + String(manualSpeed) + R"rawliteral(</b></div>
      <div class="row">WiFi RSSI: <b>)rawliteral" + String(WiFi.RSSI()) + R"rawliteral( dBm</b></div>
      <div class="row">Uptime: <b>)rawliteral" + String((millis() - bootMillis) / 1000) + R"rawliteral( seconds</b></div>
      <div class="row">AutoStop: <b>)rawliteral" + String(ultrasonicAutoStopping ? "ON" : "OFF") + R"rawliteral(</b></div>
      <div class="row">Mode: <b>)rawliteral" + String(manualMode ? "Manual" : "Auto") + R"rawliteral(</b></div>
      <div class="btn-row">
        <form method='POST' action='/cmd'><button name='cmd' value='forward'>Forward</button></form>
        <form method='POST' action='/cmd'><button name='cmd' value='back'>Back</button></form>
        <form method='POST' action='/cmd'><button name='cmd' value='left'>Left</button></form>
        <form method='POST' action='/cmd'><button name='cmd' value='right'>Right</button></form>
        <form method='POST' action='/cmd'><button name='cmd' value='stop'>Stop</button></form>
        <form method='POST' action='/cmd'><button name='cmd' value='spin'>Spin</button></form>
      </div>
      <div class="btn-row">
        <form method='POST' action='/autostop'><button>)rawliteral" + String(ultrasonicAutoStopping ? "Disable AutoStop" : "Enable AutoStop") + R"rawliteral(</button></form>
        <form method='POST' action='/testmotors'><button>Test Motors</button></form>
        <form method='POST' action='/status'><button>Status</button></form>
        <form method='POST' action='/reset'><button>Reset</button></form>
      </div>
      <div class="row">
        <form method='POST' action='/setspeed'>
          <label>Set Speed:</label>
          <input name='speed' type='number' min='50' max='255' value=')" + String(manualSpeed) + R"rawliteral('>
          <input type='submit' value='Set Speed'>
        </form>
      </div>
      <div class="btn-row">
        <form method='POST' action='/togglemode'><button>)rawliteral" + String(manualMode ? "Switch to Auto Mode" : "Switch to Manual Mode") + R"rawliteral(</button></form>
        <form method='POST' action='/cleardistances'><button>Clear Distance Log</button></form>
      </div>
      <div class="row">
        <a href='/distances'>Show Last 10 Distances</a> | <a href='/leds'>LED Control</a>
      </div>
    </div>
  </body>
  </html>
  )rawliteral";
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
  String html = R"rawliteral(
  <html>
  <head>
    <title>Last 10 Distances</title>
    <meta http-equiv="refresh" content="0.5">
    <style>
      body { background: #f7f7f7; color: #222; font-family: Arial, sans-serif; margin: 0; padding: 0; }
      .container { max-width: 400px; margin: 40px auto; background: #fff; border-radius: 12px; padding: 32px; box-shadow: 0 4px 24px #0002; }
      h2 { color: #1976d2; }
      ul { padding-left: 24px; }
      li { margin-bottom: 8px; }
      a { color: #1976d2; text-decoration: none; }
      a:hover { text-decoration: underline; }
    </style>
  </head>
  <body>
    <div class="container">
      <h2>Last 10 Distances (cm)</h2>
      <ul>
  )rawliteral";
  for (int i = 0; i < 10; i++) {
    int idx = (distanceIndex + i) % 10;
    html += "<li>" + String(lastDistances[idx]) + "</li>";
  }
  html += R"rawliteral(
      </ul>
      <a href='/'>Back</a>
    </div>
    <script>
      // Optional: Use JS for smoother refresh (uncomment to use instead of meta refresh)
      // setTimeout(() => { location.reload(); }, 500);
    </script>
  </body>
  </html>
  )rawliteral";
  server.send(200, "text/html", html);
}

void handleLeds() {
  if (server.method() == HTTP_POST) {
    if (server.hasArg("color")) {
      String color = server.arg("color");
      long number = strtol(color.c_str()+1, NULL, 16); // skip '#'
      currentColor = CRGB((number >> 16) & 0xFF, (number >> 8) & 0xFF, number & 0xFF);
    }
    if (server.hasArg("bgcolor")) {
      String bgcolor = server.arg("bgcolor");
      long number = strtol(bgcolor.c_str()+1, NULL, 16); // skip '#'
      backgroundColor = CRGB((number >> 16) & 0xFF, (number >> 8) & 0xFF, number & 0xFF);
    }
    if (server.hasArg("rainbow")) {
      setRainbow();
      server.sendHeader("Location", "/leds");
      server.send(303);
      return;
    }
    setAllLeds(currentColor, backgroundColor);
  }
  String html = R"rawliteral(
  <html>
  <head>
    <title>LED Control</title>
    <style>
      body { background: #181c20; color: #f0f0f0; font-family: Arial, sans-serif; margin: 0; padding: 0; }
      .container { max-width: 500px; margin: 40px auto; background: #23272b; border-radius: 12px; padding: 32px; box-shadow: 0 4px 24px #0008; }
      h2 { color: #ffb300; }
      button, input[type="submit"] { background: #ffb300; color: #23272b; border: none; border-radius: 6px; padding: 10px 18px; font-size: 1em; margin: 6px 0; cursor: pointer; transition: background 0.2s; }
      button:hover, input[type="submit"]:hover { background: #ffa000; }
      input[type="color"], input[type="number"] { margin: 0 8px 0 0; }
      .row { margin-bottom: 16px; }
      a { color: #4fc3f7; text-decoration: none; }
      a:hover { text-decoration: underline; }
      .btn-row { display: flex; flex-wrap: wrap; gap: 8px; }
    </style>
  </head>
  <body>
    <div class="container">
      <h2>LED Control</h2>
      <form method='POST'>
        <div class="row">
          <label>Running LED: </label>
          <input type='color' name='color' value='#)rawliteral";
  char buf[7];
  sprintf(buf, "%02X%02X%02X", currentColor.r, currentColor.g, currentColor.b);
  html += buf;
  html += R"rawliteral('>
          <label>Background: </label>
          <input type='color' name='bgcolor' value='#)";
  sprintf(buf, "%02X%02X%02X", backgroundColor.r, backgroundColor.g, backgroundColor.b);
  html += buf;
  html += R"rawliteral('>
          <input type='submit' value='Set Colors'>
        </div>
      </form>
      <div class="btn-row">
        <form method='POST' action='/ledrun'><button>Start Running Light</button></form>
        <form method='POST'><button name='rainbow' value='1'>Set Rainbow</button></form>
      </div>
      <div class="row">
        <a href='/'>Back</a>
      </div>
    </div>
  </body>
  </html>
  )rawliteral";
  server.send(200, "text/html", html);
}

void handleLedRun() {
  unsigned long start = millis();
  while (millis() - start < 5000) { // Run for 5 seconds
    runningLed();
    delay(100);
    server.handleClient();
  }
  setAllLeds(currentColor, backgroundColor); // <-- Fix: pass both arguments
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

  Serial.print("Connecting to Wi-Fi");
  unsigned long startAttemptTime = millis();
  const unsigned long wifiTimeout = 20000; // 20 seconds timeout

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < wifiTimeout) {
    Serial.print(".");
    runningGreenLed();
    delay(100);
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(3000);
    ESP.restart();
  }

  Serial.print("Connected to Wi-Fi: ");
  Serial.println(ssid);
  Serial.print("IP address: ");
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