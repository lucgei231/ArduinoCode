# Moving Robot with Web Server and OTA

This project is for an ESP32-based robot that can move, avoid obstacles using ultrasonic sensors, and be controlled via serial commands. It also runs a web server and supports OTA (Over-The-Air) updates.

## Features

- **Wi-Fi Access Point or Client**: Connects to Wi-Fi or can be set up as an access point.
- **Web Server**: Runs a web server on port 80.
- **OTA Updates**: Supports Arduino OTA for wireless firmware updates.
- **Motor Control**: Functions to move forward, backward, turn left/right, and stop.
- **Obstacle Avoidance**: Uses ultrasonic sensors to detect and avoid obstacles.
- **Serial Commands**: 
  - `stop`: Stops the motors for 15 seconds.
  - `spin`: Spins the robot in place at max speed for 15 seconds.

## Pinout

- Motor and sensor pins are defined in the code. Adjust as needed for your hardware.

## Usage

1. **Upload the code to your ESP32.**
2. **Connect to the same Wi-Fi network as the ESP32 (or connect to its AP if configured).**
3. **Access the web server at** `http://<ESP32_IP_ADDRESS>/`
4. **Use the Arduino Serial Monitor** to send commands like `stop` or `spin`.

## Requirements

- ESP32 board
- Arduino IDE with ESP32 board support
- Required libraries: `WiFi`, `WebServer`, `ESPmDNS`, `ArduinoOTA`

## Notes

- Make sure to adjust the SSID and password in the code for your Wi-Fi network.
- OTA updates can be performed using the Arduino IDE's "Network Ports" menu.

