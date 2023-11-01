# Line Follower Bot with ESP32 and Camera

This project is designed to create a line-following robot using an ESP32 microcontroller and a camera for path detection.

## Code Overview

The provided code initializes the ESP32 and camera to capture images and process them for line detection. It includes functionalities to start and stop the robot's movement via a web-based control interface. The bot's movement is based on the detected path through the camera.

### Code Functionality

- **ESP32 and Camera Initialization:** 
  - Validates and sets up the ESP32 board with the required camera settings.
- **Web Control Interface:**
  - Initiates a web server on the ESP32, allowing control through a webpage.
  - Provides start/stop buttons for controlling the movement of the bot.
- **Image Processing for Line Detection:**
  - Captures images through the camera.
  - Processes images to detect the line path based on predefined thresholds.
  - Controls the movement of the bot in response to the detected path.

### Web Interface Functionality

- The web-based interface allows the user to control the start and stop of the line-follower bot.
- Additionally, it displays an elapsed timer showing the bot's running time.

### Requirements

- **Hardware:**
  - ESP32 board
  - Camera compatible with ESP32
  - WiFi connection for web-based control
- **Software Libraries:**
  - `esp_camera.h`: Library for interfacing with the camera.
  - `driver/ledc.h`: LED control library.
  - `soc/soc.h` and `soc/rtc_cntl_reg.h`: ESP32 system control libraries.
  - `WiFi.h` and `WebServer.h`: Libraries for WiFi connection and web server functionality.

## 3D Printed Mounts

The folder '3D_Printed_Mounts' contains the necessary .stl file for 3D printing the camera mount designed explicitly for this project. This mount is crucial for securely affixing the camera onto the line-follower robot.

### Included Mount

- **Camera Mount:** Specifically designed to securely attach and position the camera onto the line-follower robot.
