#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <NetworkUdp.h>
#include <ArduinoOTA.h>

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

// Ultrasonic distance function
int ultraDistance(int trigPin, int echoPin) {
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

  server.begin();

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
}

void loop() {
  server.handleClient();
  ArduinoOTA.handle();
  // Remove BitFlash handle

  // --- Serial command handling ---
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.equalsIgnoreCase("stop")) {
      Serial.println("Received 'stop' command. Stopping motors for 15 seconds.");
      stopMotors();
      delay(15000);
      Serial.println("Resuming motors.");
    }
  }
  // --- End serial command handling ---

  if (digitalRead(0) == LOW) {
    Serial.println("Boot triggered");
    stopMotors();
    delay(500000);
  }

  float frontDistance = ultraDistance(2, 15);
  
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