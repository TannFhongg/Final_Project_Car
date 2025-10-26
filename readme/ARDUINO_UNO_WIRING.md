# 🔌 Arduino Uno Wiring Guide

## Hardware Components
- **Arduino Uno** (ATmega328P)
- **L298N Motor Driver**
- **2x DC Motors** (with gearbox)
- **HC-SR04 Ultrasonic Sensor**
- **8x IR Line Sensors** (TCRT5000 or similar)
- **Power Supply** (7.4V-12V LiPo battery recommended)

---

## 📟 Pin Connections

### **Arduino Uno ↔ L298N Motor Driver**

| Arduino Pin | L298N Pin | Function | Motor |
|------------|-----------|----------|-------|
| **D9** (PWM) | **ENA** | Left Motor Speed Control | Left |
| **D2** | **IN1** | Left Motor Direction 1 | Left |
| **D3** | **IN2** | Left Motor Direction 2 | Left |
| **D10** (PWM) | **ENB** | Right Motor Speed Control | Right |
| **D4** | **IN3** | Right Motor Direction 1 | Right |
| **D5** | **IN4** | Right Motor Direction 2 | Right |
| **GND** | **GND** | Common Ground | - |

**L298N ↔ Motors & Power:**
- **OUT1, OUT2** → Left Motor
- **OUT3, OUT4** → Right Motor
- **12V** → Battery (+) 7.4V-12V
- **GND** → Battery (-)
- **5V Out** → Can power Arduino if using external power

⚠️ **Important:** 
- ENA and ENB jumpers must be **REMOVED** (we control speed via PWM)
- Connect Arduino D9 to ENA, D10 to ENB

---

### **Arduino Uno ↔ HC-SR04 Ultrasonic**

| Arduino Pin | HC-SR04 Pin | Function |
|------------|-------------|----------|
| **D12** | **TRIG** | Trigger Pin |
| **D11** | **ECHO** | Echo Pin |
| **5V** | **VCC** | Power |
| **GND** | **GND** | Ground |

---

### **Arduino Uno ↔ IR Line Sensors (8x)**

| Arduino Pin | Sensor # | Position | Type |
|------------|----------|----------|------|
| **A0** | Sensor 1 | Leftmost | Analog |
| **A1** | Sensor 2 | Left | Analog |
| **A2** | Sensor 3 | Center-Left | Analog |
| **A3** | Sensor 4 | Center-Left | Analog |
| **A4** | Sensor 5 | Center-Right | Analog |
| **A5** | Sensor 6 | Center-Right | Analog |
| **D6** | Sensor 7 | Right | Digital |
| **D7** | Sensor 8 | Rightmost | Digital |

**Each IR Sensor:**
- **VCC** → 5V (Arduino)
- **GND** → GND
- **OUT** → Arduino pin (above)

---

### **Arduino Uno ↔ Raspberry Pi 5**

| Arduino Pin | Raspberry Pi Pin | Function |
|------------|------------------|----------|
| **TX (D1)** | **GPIO 15 (RXD)** | Arduino TX → Pi RX |
| **RX (D0)** | **GPIO 14 (TXD)** | Arduino RX → Pi TX |
| **GND** | **GND (Pin 6)** | Common Ground |

**⚠️ CRITICAL:**
- **DO NOT connect Arduino 5V to Raspberry Pi pins!**
- Use **common GND only**
- Arduino and Raspberry Pi have **separate power supplies**

---

## 🎨 Pin Mapping Diagram

```
Arduino Uno
┌─────────────────────────────────┐
│                                 │
│  D0(RX) ────────────────────→ Pi TX
│  D1(TX) ────────────────────→ Pi RX
│  D2 ──────────→ L298N IN1      │
│  D3 ──────────→ L298N IN2      │
│  D4 ──────────→ L298N IN3      │
│  D5 ──────────→ L298N IN4      │
│  D6 ──────────→ Line Sensor 7  │
│  D7 ──────────→ Line Sensor 8  │
│  D9 (PWM) ────→ L298N ENA      │
│  D10(PWM) ────→ L298N ENB      │
│  D11 ─────────→ HC-SR04 ECHO   │
│  D12 ─────────→ HC-SR04 TRIG   │
│                                 │
│  A0 ──────────→ Line Sensor 1  │
│  A1 ──────────→ Line Sensor 2  │
│  A2 ──────────→ Line Sensor 3  │
│  A3 ──────────→ Line Sensor 4  │
│  A4 ──────────→ Line Sensor 5  │
│  A5 ──────────→ Line Sensor 6  │
│                                 │
│  5V ──────────→ Sensors VCC    │
│  GND ─────────→ Common GND     │
└─────────────────────────────────┘
```

---

## 🔋 Power Supply Configuration

### **Option 1: Separate Power (Recommended)**
```
Battery (7.4V-12V) ──→ L298N (12V input)
                    ├─→ L298N 5V Out → Arduino Uno VIN
                    
USB Power Bank ─────→ Raspberry Pi 5 (USB-C, 5V/3A)
```

### **Option 2: Single Battery with Buck Converter**
```
Battery (7.4V-12V) ──→ L298N (12V input)
                    ├─→ Buck Converter (7.4V→5V) → Raspberry Pi
                    └─→ L298N 5V Out → Arduino Uno VIN
```

---

## 📝 Complete Wiring Checklist

### Power Connections:
- [ ] ✅ L298N connected to battery (7.4V-12V)
- [ ] ✅ Arduino Uno powered via USB or VIN
- [ ] ✅ Raspberry Pi powered separately (USB-C 5V/3A)
- [ ] ✅ Common GND between all devices

### Motor Connections:
- [ ] ✅ Left motor → L298N OUT1, OUT2
- [ ] ✅ Right motor → L298N OUT3, OUT4
- [ ] ✅ ENA jumper REMOVED, D9 → ENA
- [ ] ✅ ENB jumper REMOVED, D10 → ENB
- [ ] ✅ D2 → IN1, D3 → IN2
- [ ] ✅ D4 → IN3, D5 → IN4

### Sensor Connections:
- [ ] ✅ HC-SR04: D12→TRIG, D11→ECHO, 5V, GND
- [ ] ✅ 8x IR sensors: A0-A5, D6-D7, 5V, GND
- [ ] ✅ All sensors have 5V power
- [ ] ✅ All sensors share common GND

### Communication:
- [ ] ✅ Arduino TX (D1) → Pi RX (GPIO15)
- [ ] ✅ Arduino RX (D0) → Pi TX (GPIO14)
- [ ] ✅ Common GND between Arduino and Pi
- [ ] ✅ NO direct 5V connection between Arduino and Pi

---

## 🧪 Testing Procedure

### **Step 1: Visual Inspection**
```bash
# Check all connections match the diagram above
# Verify no loose wires
# Check polarity of power connections
```

### **Step 2: Power Test**
```bash
# 1. Connect battery to L298N (motors disconnected)
# 2. Check L298N LED lights up
# 3. Measure L298N 5V output: should be 5V
# 4. Connect Arduino to L298N 5V out
# 5. Arduino power LED should light up
```

### **Step 3: Upload Firmware**
```bash
# Using Arduino IDE:
# 1. Open arduino_firmware.ino
# 2. Tools → Board → Arduino Uno
# 3. Tools → Port → /dev/ttyACM0 (or /dev/ttyUSB0)
# 4. Install ArduinoJson library
# 5. Click Upload

# Using arduino-cli:
arduino-cli lib install ArduinoJson
arduino-cli compile --fqbn arduino:avr:uno arduino_firmware/
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno arduino_firmware/
```

### **Step 4: Serial Monitor Test**
```bash
# Open serial monitor (115200 baud)
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200

# Expected output:
{"status":"ready","device":"arduino_uno"}
```

### **Step 5: Motor Test (No Load)**
```bash
# Send commands via serial monitor:
{"cmd":"MOVE","left":100,"right":100}
# Motors should spin forward slowly

{"cmd":"STOP"}
# Motors should stop

{"cmd":"MOVE","left":-100,"right":-100}
# Motors should spin backward

{"cmd":"MOVE","left":100,"right":-100}
# Robot should turn right
```

### **Step 6: Sensor Test**
```bash
# Place objects in front of sensors
# Watch for sensor data:
{"line":[0,0,1,1,1,1,0,0],"line_pos":0,"distance":25.5,"left_speed":0,"right_speed":0,"uptime":5432}
```

---

## 🔧 Troubleshooting

### ❌ **Motors not moving**

**Check:**
1. ✅ L298N power LED is ON (battery connected)
2. ✅ ENA/ENB jumpers are REMOVED
3. ✅ Arduino D9 connected to ENA, D10 to ENB
4. ✅ Motor wires properly connected to OUT1-4
5. ✅ Battery voltage > 6V (measure with multimeter)

**Test:**
```bash
# Test motors directly with battery
# If motors work, issue is in wiring or code
```

### ❌ **Arduino not detected on Raspberry Pi**

**Check:**
```bash
# List USB devices
ls /dev/ttyACM* /dev/ttyUSB*

# Common Arduino Uno ports:
# /dev/ttyACM0 - Most common for Uno
# /dev/ttyUSB0 - If using USB-Serial adapter

# Check permissions
sudo usermod -a -G dialout $USER
# Logout and login again
```

### ❌ **Sensors not reading**

**Check:**
1. ✅ All sensors have 5V power (measure with multimeter)
2. ✅ Common GND connected
3. ✅ Sensor outputs go to correct Arduino pins
4. ✅ For IR sensors: adjust threshold potentiometer

**Test individual sensor:**
```bash
# In serial monitor, watch for sensor updates
# Wave hand over sensors, values should change
```

### ❌ **"JSON parse error" in logs**

**Causes:**
- Loose TX/RX connections
- Wrong baud rate
- Missing common GND

**Fix:**
```bash
# 1. Check TX/RX are crossed (Arduino TX → Pi RX)
# 2. Verify baud rate is 115200 in both Arduino and Pi
# 3. Ensure common GND connected
# 4. Try different USB cable
```

---

## 📊 Pin Summary Table

| Function | Arduino Uno Pin | Connection |
|----------|----------------|------------|
| **Motors** | | |
| Left Motor Speed | D9 (PWM) | L298N ENA |
| Left Motor Dir 1 | D2 | L298N IN1 |
| Left Motor Dir 2 | D3 | L298N IN2 |
| Right Motor Speed | D10 (PWM) | L298N ENB |
| Right Motor Dir 1 | D4 | L298N IN3 |
| Right Motor Dir 2 | D5 | L298N IN4 |
| **Ultrasonic** | | |
| Trigger | D12 | HC-SR04 TRIG |
| Echo | D11 | HC-SR04 ECHO |
| **Line Sensors** | | |
| Sensor 1 | A0 | IR Sensor 1 |
| Sensor 2 | A1 | IR Sensor 2 |
| Sensor 3 | A2 | IR Sensor 3 |
| Sensor 4 | A3 | IR Sensor 4 |
| Sensor 5 | A4 | IR Sensor 5 |
| Sensor 6 | A5 | IR Sensor 6 |
| Sensor 7 | D6 | IR Sensor 7 |
| Sensor 8 | D7 | IR Sensor 8 |
| **Communication** | | |
| UART TX | D1 | Raspberry Pi RX |
| UART RX | D0 | Raspberry Pi TX |

---

## ⚡ Important Notes

1. **PWM Pins:** Arduino Uno has PWM on pins 3, 5, 6, 9, 10, 11
   - We use **D9** for left motor (ENA)
   - We use **D10** for right motor (ENB)

2. **Serial Port:** Arduino Uno typically appears as `/dev/ttyACM0` on Raspberry Pi

3. **Power:** Never connect Arduino 5V pin to Raspberry Pi GPIO pins (Pi uses 3.3V logic)

4. **L298N Jumpers:** MUST remove ENA/ENB jumpers to enable PWM speed control

5. **Ground:** All devices must share common ground

---

## 🎯 Next Steps

After wiring complete:

1. ✅ Upload firmware to Arduino Uno
2. ✅ Update `hardware_config.yaml` with port `/dev/ttyACM0`
3. ✅ Run test: `python3 test_arduino.py`
4. ✅ Calibrate IR sensors (adjust potentiometers)
5. ✅ Run main application: `python3 main.py`
6. ✅ Access dashboard: `http://<pi-ip>:5000`

---

**Good luck with your wiring! 🤖⚡**