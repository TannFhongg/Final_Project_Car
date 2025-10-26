# 🚀 LogisticsBot - Complete Setup & Installation Guide

## 📋 Table of Contents
1. [Hardware Requirements](#hardware-requirements)
2. [Software Requirements](#software-requirements)
3. [Raspberry Pi Setup](#raspberry-pi-setup)
4. [Arduino Setup](#arduino-setup)
5. [Hardware Wiring](#hardware-wiring)
6. [Software Installation](#software-installation)
7. [Configuration](#configuration)
8. [Testing](#testing)
9. [Running the System](#running-the-system)
10. [Troubleshooting](#troubleshooting)

---

## 🛠️ Hardware Requirements

### **Required Components:**
- [ ] **Raspberry Pi 5** (4GB+ RAM recommended)
- [ ] **Arduino Uno** (ATmega328P)
- [ ] **L298N Motor Driver Module**
- [ ] **2x DC Geared Motors** (6V-12V, with wheels)
- [ ] **HC-SR04 Ultrasonic Sensor**
- [ ] **5x IR Line Sensors** (TCRT5000)
- [ ] **Raspberry Pi Camera V2**
- [ ] **7.4V-12V LiPo Battery** (for motors, 2200mAh+)
- [ ] **5V Power Bank** (for Raspberry Pi, 3A+)
- [ ] **USB Cable** (Type-C for Pi, Type-B for Arduino)
- [ ] **Jumper Wires** (Male-Male, Male-Female)
- [ ] **Breadboard** (optional, for prototyping)
- [ ] **Robot Chassis** (with mounting holes)

### **Optional Components:**
- [ ] Motor Encoders
- [ ] Additional sensors (temperature, IMU)
- [ ] LED indicators
- [ ] Buzzer for alerts

### **Tools Needed:**
- Screwdriver set
- Wire stripper
- Multimeter
- Soldering iron (optional)
- Hot glue gun
- Zip ties

---

## 💻 Software Requirements

### **On Your Computer:**
- [ ] **Arduino IDE** (version 2.0+)
- [ ] **SSH Client** (PuTTY for Windows, built-in for Mac/Linux)
- [ ] **Raspberry Pi Imager**
- [ ] **Text Editor** (VS Code recommended)

### **On Raspberry Pi:**
- [ ] **Raspberry Pi OS** (Bookworm or later)
- [ ] **Python 3.11+**
- [ ] **Git**

---

## 🍓 Part 1: Raspberry Pi Setup

### **Step 1.1: Install Raspberry Pi OS**

```bash
# On your computer:
# 1. Download Raspberry Pi Imager: https://www.raspberrypi.com/software/
# 2. Insert SD card (32GB+ recommended)
# 3. Open Raspberry Pi Imager
# 4. Choose OS: Raspberry Pi OS (64-bit, Bookworm)
# 5. Choose SD card
# 6. Click Settings (gear icon):
#    - Set hostname: logisticsbot
#    - Enable SSH
#    - Set username: pi
#    - Set password: raspberry (or your choice)
#    - Configure WiFi (optional)
# 7. Click Write
```

### **Step 1.2: Boot Raspberry Pi**

```bash
# 1. Insert SD card into Raspberry Pi
# 2. Connect power (USB-C, 5V/3A)
# 3. Wait 2-3 minutes for first boot

# 4. Find Raspberry Pi IP address:
#    Method 1: Check your router's connected devices
#    Method 2: Use network scanner (e.g., Angry IP Scanner)
#    Method 3: Connect monitor and keyboard, run: hostname -I
```

### **Step 1.3: Connect via SSH**

```bash
# From your computer:
ssh pi@192.168.1.100  # Replace with your Pi's IP
# Password: raspberry (or what you set)

# You should see:
# pi@logisticsbot:~ $
```

### **Step 1.4: Update System**

```bash
# Update package lists
sudo apt update

# Upgrade installed packages
sudo apt upgrade -y

# Install essential tools
sudo apt install -y git python3-pip python3-venv vim nano htop

# Reboot (recommended)
sudo reboot

# Wait 1 minute, then reconnect via SSH
```

### **Step 1.5: Enable Camera & UART**

```bash
# Enable camera and serial port
sudo raspi-config

# Navigate:
# 3. Interface Options
#    - P1 Camera → Enable
#    - P6 Serial Port
#      → Login shell over serial: No
#      → Serial port hardware: Yes

# Alternatively, edit config directly:
sudo nano /boot/firmware/config.txt

# Add these lines:
start_x=1
enable_uart=1
dtoverlay=disable-bt

# Save: Ctrl+X, Y, Enter
# Reboot
sudo reboot
```

### **Step 1.6: Test Camera**

```bash
# Reconnect via SSH after reboot
ssh pi@192.168.1.100

# Test camera
libcamera-hello --list-cameras

# Should show camera detected

# Take test photo
libcamera-jpeg -o test.jpg

# If successful, camera is working!
```

---

## 🔧 Part 2: Arduino Setup

### **Step 2.1: Install Arduino IDE** (On Your Computer)

```bash
# Download from: https://www.arduino.cc/en/software

# Windows: Run installer
# Mac: Drag to Applications
# Linux:
cd ~/Downloads
tar -xvf arduino-ide_*_Linux_64bit.tar.gz
sudo mv arduino-ide_* /opt/arduino-ide
sudo /opt/arduino-ide/arduino-ide
```

### **Step 2.2: Install ArduinoJson Library**

```bash
# In Arduino IDE:
# 1. Tools → Manage Libraries
# 2. Search: "ArduinoJson"
# 3. Install: ArduinoJson by Benoit Blanchon (version 6.21+)
```

### **Step 2.3: Upload Firmware**

```bash
# 1. Connect Arduino Uno to your computer via USB
# 2. In Arduino IDE:
#    - Tools → Board → Arduino Uno
#    - Tools → Port → /dev/ttyACM0 (Linux/Mac) or COM3 (Windows)
# 3. Copy firmware code from artifact "arduino_uno_5_sensors"
# 4. Paste into Arduino IDE
# 5. Click Upload (→ button)
# 6. Wait for "Done uploading"
```

### **Step 2.4: Test Arduino**

```bash
# In Arduino IDE:
# Tools → Serial Monitor
# Set baud rate: 115200

# You should see:
{"status":"ready","device":"arduino_uno","ir_sensors":5}

# Test motor command:
# Type in Serial Monitor:
{"cmd":"MOVE","left":100,"right":100}

# Motors should spin (if connected)

# Stop motors:
{"cmd":"STOP"}
```

---

## 🔌 Part 3: Hardware Wiring

### **Step 3.1: Power Connections**

```
Battery (7.4V-12V)
├─ (+) → L298N 12V input
└─ (-) → L298N GND

L298N 5V output
└─ Arduino Uno VIN (if not powered by USB)

Power Bank (5V/3A)
└─ Raspberry Pi USB-C
```

⚠️ **IMPORTANT:** 
- Arduino and Raspberry Pi have **separate power supplies**
- Never connect Arduino 5V to Raspberry Pi GPIO pins
- **Common GND only** for UART communication

### **Step 3.2: Motor Connections**

```
L298N → Arduino Uno:
├─ ENA → D9 (PWM)
├─ IN1 → D2
├─ IN2 → D3
├─ ENB → D10 (PWM)
├─ IN3 → D4
├─ IN4 → D5
└─ GND → GND

L298N → Motors:
├─ OUT1, OUT2 → Left Motor
└─ OUT3, OUT4 → Right Motor

⚠️ Remove ENA/ENB jumpers on L298N!
```

### **Step 3.3: Sensor Connections**

```
HC-SR04 → Arduino Uno:
├─ VCC → 5V
├─ TRIG → D12
├─ ECHO → D11
└─ GND → GND

IR Sensors (5x) → Arduino Uno:
├─ Sensor 1 → A0
├─ Sensor 2 → A1
├─ Sensor 3 → A2
├─ Sensor 4 → A3
├─ Sensor 5 → A4
└─ All: VCC → 5V, GND → GND
```

### **Step 3.4: UART Communication**

```
Arduino Uno → Raspberry Pi:
├─ TX (D1) → GPIO 15 (RXD) Pin 10
├─ RX (D0) → GPIO 14 (TXD) Pin 8
└─ GND → GND (Pin 6)

⚠️ CRITICAL: Cross connections (TX to RX)
⚠️ Common GND is essential!
```

### **Step 3.5: Camera Connection**

```
Pi Camera V2 → Raspberry Pi:
└─ Ribbon cable to Camera port (near HDMI)

# Make sure camera is enabled in raspi-config
```

### **Wiring Checklist:**
- [ ] L298N has power (LED on)
- [ ] ENA/ENB jumpers removed
- [ ] Motors connected to L298N
- [ ] Arduino powered (LED on)
- [ ] HC-SR04 connected
- [ ] 5 IR sensors connected
- [ ] UART TX/RX crossed
- [ ] Common GND between Arduino and Pi
- [ ] Camera connected
- [ ] Raspberry Pi powered

---

## 📦 Part 4: Software Installation on Raspberry Pi

### **Step 4.1: Create Project Directory**

```bash
# SSH into Raspberry Pi
ssh pi@192.168.1.100

# Create project directory
mkdir -p ~/logisticsbot
cd ~/logisticsbot

# Create directory structure
mkdir -p config templates static/{css,js,images} drivers/motor control
mkdir -p perception behaviors sensors utils arduino_firmware
mkdir -p data/{logs,models,recordings,calibration}
mkdir -p tests scripts docs

# Create __init__.py files
touch drivers/__init__.py drivers/motor/__init__.py
touch control/__init__.py perception/__init__.py
touch behaviors/__init__.py sensors/__init__.py utils/__init__.py
```

### **Step 4.2: Create Virtual Environment**

```bash
cd ~/logisticsbot

# Create virtual environment
python3 -m venv venv

# Activate virtual environment
source venv/bin/activate

# You should see (venv) in prompt
```

### **Step 4.3: Create requirements.txt**

```bash
nano requirements.txt
```

**Paste this content:**
```
# Web Framework
flask==3.0.0
flask-socketio==5.3.5
python-socketio==5.10.0

# Hardware Communication
pyserial==3.5
gpiozero==2.0.1
pigpio==1.78

# Computer Vision & AI
opencv-python==4.8.1.78
numpy==1.24.3
Pillow==10.1.0

# Configuration
PyYAML==6.0.1

# Utilities
python-dateutil==2.8.2
```

**Save:** Ctrl+X, Y, Enter

### **Step 4.4: Install Python Packages**

```bash
# Make sure virtual environment is activated
source venv/bin/activate

# Upgrade pip
pip install --upgrade pip

# Install packages (takes 5-10 minutes)
pip install -r requirements.txt

# Verify installation
pip list | grep -E 'flask|serial|opencv|yaml'
```

### **Step 4.5: Add User to dialout Group**

```bash
# Add current user to dialout group (for serial port access)
sudo usermod -a -G dialout $USER

# Verify
groups $USER

# Should show: ... dialout ...

# IMPORTANT: Logout and login again for this to take effect
exit

# Reconnect
ssh pi@192.168.1.100
```

---

## 📝 Part 5: Copy Project Files

### **Step 5.1: Copy Core Python Files**

```bash
cd ~/logisticsbot

# Create main.py
nano main.py
# Paste content from artifact "main_with_arduino"
# Save: Ctrl+X, Y, Enter

# Create test script
nano test_arduino.py
# Paste content from artifact "test_arduino_script"
# Save

# Create utils
nano utils/logger.py
# Paste content from document index 19
# Save

nano utils/config_loader.py
# Paste content from document index 18
# Save

# Create drivers
nano drivers/motor/arduino_driver.py
# Paste content from artifact "arduino_uart_driver" (updated)
# Save

nano drivers/motor/l298n_driver.py
# Paste content from document index 12
# Save

# Create control
nano control/robot_controller.py
# Paste content from document index 9 (or artifact "updated_robot_controller")
# Save
```

### **Step 5.2: Copy Web Interface Files**

```bash
# Create HTML template
nano templates/index.html
# Paste content from artifact "updated_index_html"
# Save

# Create CSS
nano static/css/style.css
# Paste content from artifact "updated_style_css"
# Save

# Create JavaScript
nano static/js/app.js
# Paste content from artifact "updated_app_js"
# Save
```

### **Step 5.3: Create Configuration**

```bash
nano config/hardware_config.yaml
# Paste content from artifact "updated_hardware_config"
# Save

# ⚠️ IMPORTANT: Update serial port
# Change line:
#   port: '/dev/ttyACM0'
# To your actual Arduino port (check with: ls /dev/ttyACM* /dev/ttyUSB*)
```

### **Step 5.4: Create Log File**

```bash
# Create initial log
mkdir -p data/logs
echo "[$(date '+%Y-%m-%d %H:%M:%S')] INFO: LogisticsBot initialized" > data/logs/robot.log
```

---

## 🔍 Part 6: Find Arduino Port

```bash
# Before connecting Arduino
ls /dev/tty* > before.txt

# Connect Arduino Uno to Raspberry Pi via USB

# After connecting
ls /dev/tty* > after.txt

# Find difference
diff before.txt after.txt

# Usually shows:
# /dev/ttyACM0  (Arduino Uno)
# or
# /dev/ttyUSB0  (with USB-Serial adapter)

# Verify
ls -l /dev/ttyACM0

# Should show: crw-rw---- 1 root dialout ...

# Test connection
screen /dev/ttyACM0 115200
# Should see JSON messages from Arduino
# Exit: Ctrl+A, then K, then Y
```

---

## ⚙️ Part 7: Configuration

### **Step 7.1: Update Hardware Config**

```bash
nano config/hardware_config.yaml

# Update these settings:
arduino:
  port: '/dev/ttyACM0'  # Your Arduino port

sensors:
  camera:
    device: 0  # Camera index (usually 0)

# Save: Ctrl+X, Y, Enter
```

### **Step 7.2: Test Configuration Loading**

```bash
cd ~/logisticsbot
source venv/bin/activate

python3 -c "
from utils.config_loader import load_config
config = load_config('config/hardware_config.yaml')
print('✓ Config loaded successfully')
print(f'Arduino port: {config[\"arduino\"][\"port\"]}')
"
```

---

## 🧪 Part 8: Testing

### **Step 8.1: Test Arduino Connection**

```bash
cd ~/logisticsbot
source venv/bin/activate

python3 test_arduino.py
```

**Expected Output:**
```
╔══════════════════════════════════════════════════════════╗
║      Arduino UART Communication Test Suite              ║
╚══════════════════════════════════════════════════════════╝

TEST 1: Arduino Connection
Trying port: /dev/ttyACM0
✅ Connected successfully on /dev/ttyACM0

TEST 2: Motor Control
Forward... ✅ OK (Left: 150, Right: 150)
...

✅ All tests passed! Arduino is working correctly.
```

### **Step 8.2: Test Camera**

```bash
# Test OpenCV camera
python3 -c "
import cv2
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
print(f'Camera OK: {ret}')
if ret:
    print(f'Frame shape: {frame.shape}')
cap.release()
"

# Expected:
# Camera OK: True
# Frame shape: (480, 640, 3)
```

### **Step 8.3: Test Imports**

```bash
python3 -c "
import flask
import serial
import cv2
import yaml
from flask_socketio import SocketIO
print('✓ All imports successful')
"
```

---

## 🚀 Part 9: Running the System

### **Step 9.1: First Run**

```bash
cd ~/logisticsbot
source venv/bin/activate

# Run main application
python3 main.py
```

**Expected Output:**
```
============================================================
LogisticsBot Control System Starting...
============================================================
Configuration loaded successfully
L298N Motor Driver initialized
Robot Controller initialized
Auto Mode Controller initialized
Follow Mode Controller initialized
Hardware initialized successfully
Starting web server on http://0.0.0.0:5000
Access dashboard at: http://192.168.1.100:5000
```

### **Step 9.2: Access Web Dashboard**

```
1. Open web browser on your computer/phone
2. Go to: http://192.168.1.100:5000
   (Replace with your Raspberry Pi's IP)
3. You should see the LogisticsBot Control Panel
```

### **Step 9.3: Test Controls**

```
In Web Dashboard:
1. Check "Connected" status (green badge)
2. Test Manual Control:
   - Click Forward button → Motors should move forward
   - Click Stop → Motors should stop
   - Test Left/Right buttons
3. Check sensor readings updating
4. Test speed slider
5. Try Emergency Stop
```

---

## 🔧 Part 10: Troubleshooting

### **Problem: "Module not found"**

```bash
# Make sure virtual environment is activated
source ~/logisticsbot/venv/bin/activate

# Reinstall packages
pip install -r requirements.txt
```

### **Problem: "Permission denied: /dev/ttyACM0"**

```bash
# Check groups
groups $USER

# Should include 'dialout'
# If not:
sudo usermod -a -G dialout $USER

# Logout and login again
exit
ssh pi@192.168.1.100
```

### **Problem: "Arduino not responding"**

```bash
# Check USB connection
ls -l /dev/ttyACM*

# Test with screen
screen /dev/ttyACM0 115200

# Should see JSON messages
# If not, re-upload firmware

# Try different port
ls /dev/ttyUSB*
```

### **Problem: "Camera not found"**

```bash
# Check camera is enabled
sudo raspi-config
# Interface Options → Camera → Enable

# Test camera
libcamera-hello --list-cameras

# Check /dev/video*
ls /dev/video*

# Should show: /dev/video0
```

### **Problem: "Cannot access web dashboard"**

```bash
# Check Flask is running
ps aux | grep python

# Check port 5000
sudo netstat -tlnp | grep 5000

# Check firewall (if enabled)
sudo ufw allow 5000

# Try localhost first
curl http://localhost:5000

# Check Raspberry Pi IP
hostname -I
```

### **Problem: "Motors not moving"**

```bash
# Checklist:
1. L298N power LED is ON?
2. ENA/ENB jumpers removed?
3. Battery voltage > 6V?
4. Motor wires connected?
5. Arduino D9 → ENA, D10 → ENB?
6. Check with multimeter: OUT1-4 should show voltage when moving
```

---

## 📊 Part 11: System Health Check

### **Create Health Check Script:**

```bash
nano ~/logisticsbot/scripts/health_check.sh
```

**Paste:**
```bash
#!/bin/bash
echo "=== LogisticsBot Health Check ==="
echo ""
echo "1. Arduino Connection:"
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "❌ No Arduino found"
echo ""
echo "2. Camera:"
ls /dev/video* 2>/dev/null || echo "❌ No camera found"
echo ""
echo "3. Python Packages:"
source ~/logisticsbot/venv/bin/activate
pip list | grep -E 'flask|serial|opencv|yaml' || echo "❌ Packages missing"
echo ""
echo "4. Process:"
ps aux | grep "python3 main.py" | grep -v grep || echo "❌ Not running"
echo ""
echo "5. Port 5000:"
sudo netstat -tlnp | grep 5000 || echo "❌ Port not open"
```

**Make executable:**
```bash
chmod +x ~/logisticsbot/scripts/health_check.sh

# Run it
~/logisticsbot/scripts/health_check.sh
```

---

## 🎯 Part 12: Auto-Start on Boot (Optional)

### **Create systemd Service:**

```bash
sudo nano /etc/systemd/system/logisticsbot.service
```

**Paste:**
```ini
[Unit]
Description=LogisticsBot Control System
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/logisticsbot
Environment="PATH=/home/pi/logisticsbot/venv/bin"
ExecStart=/home/pi/logisticsbot/venv/bin/python3 main.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**Enable and start:**
```bash
# Reload systemd
sudo systemctl daemon-reload

# Enable service (start on boot)
sudo systemctl enable logisticsbot.service

# Start service now
sudo systemctl start logisticsbot.service

# Check status
sudo systemctl status logisticsbot.service

# View logs
sudo journalctl -u logisticsbot.service -f
```

---

## ✅ Final Verification Checklist

### **Hardware:**
- [ ] Raspberry Pi 5 powered and accessible via SSH
- [ ] Arduino Uno connected and responding
- [ ] L298N powered (LED on)
- [ ] Motors connected and working
- [ ] HC-SR04 ultrasonic connected
- [ ] 5x IR sensors connected
- [ ] Camera connected and detected
- [ ] All GND connections secure

### **Software:**
- [ ] Raspberry Pi OS updated
- [ ] Python 3.11+ installed
- [ ] Virtual environment created
- [ ] All packages installed
- [ ] User in dialout group
- [ ] All project files copied
- [ ] Configuration updated
- [ ] Arduino firmware uploaded

### **Testing:**
- [ ] `test_arduino.py` passes
- [ ] Camera detected
- [ ] `python3 main.py` starts without errors
- [ ] Web dashboard accessible
- [ ] Motors respond to commands
- [ ] Sensors reading correctly
- [ ] All three modes work (Manual/Auto/Follow)

---

## 🎓 Quick Start Commands Reference

```bash
# SSH to Raspberry Pi
ssh pi@192.168.1.100

# Activate virtual environment
cd ~/logisticsbot
source venv/bin/activate

# Run application
python3 main.py

# Test Arduino
python3 test_arduino.py

# Check logs
tail -f data/logs/robot.log

# System health
~/logisticsbot/scripts/health_check.sh

# Stop application
Ctrl+C

# Restart service (if using systemd)
sudo systemctl restart logisticsbot

# View service logs
sudo journalctl -u logisticsbot -f
```

---

## 🎉 Success!

Your LogisticsBot is now fully set up and ready to go!

**Next Steps:**
1. ✅ Calibrate IR sensors
2. ✅ Test line following
3. ✅ Implement AI features
4. ✅ Enjoy your robot!

**Access Dashboard:** `http://<your-pi-ip>:5000`

**Happy Building! 🤖🚀**
