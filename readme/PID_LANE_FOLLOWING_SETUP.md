# 🎯 PID Lane Following - Complete Setup Guide

## 📋 Overview

Hệ thống lane following hoàn chỉnh với:
- ✅ **PID Controller** - Advanced control với anti-windup, smoothing
- ✅ **Lane Detection** - Computer vision để phát hiện midline
- ✅ **Auto Mode** - Tự động theo làn đường
- ✅ **Real-time Tuning** - Điều chỉnh PID trong khi chạy

---

## 📦 Files Created

### **1. control/pid_controller.py** ⭐ NEW
- `PIDController` class với full features
- Anti-windup (chống tích phân quá mức)
- Output clamping (giới hạn đầu ra)
- Derivative smoothing (làm mịn đạo hàm)
- `AdaptivePIDController` tự điều chỉnh theo error

### **2. perception/lane_detector.py** ⭐ NEW
- `detect_line()` - Phát hiện lane và tính midline
- Hough Transform cho detect đường kẻ
- ROI (Region of Interest) để tối ưu
- Debug frame với visualization
- `detect_line_simple()` - Version đơn giản cho testing

### **3. control/robot_controller.py** ⭐ UPDATED
- `AutoModeController` hoàn toàn mới
- Tích hợp PID + lane detection
- Camera initialization
- Lane loss handling
- Real-time PID status

---

## 🚀 Installation Steps

### **Step 1: Create Directory Structure**

```bash
cd ~/logisticsbot

# Create directories if not exist
mkdir -p control perception

# Create __init__.py
touch control/__init__.py
touch perception/__init__.py
```

### **Step 2: Copy Files**

```bash
# PID Controller
nano control/pid_controller.py
# Paste from artifact: "pid_controller_class"
# Save: Ctrl+X, Y, Enter

# Lane Detector
nano perception/lane_detector.py
# Paste from artifact: "lane_detector_module"
# Save

# Update Robot Controller
nano control/robot_controller.py
# Paste from artifact: "updated_robot_controller" (updated section)
# Save
```

### **Step 3: Update Configuration**

```bash
nano config/hardware_config.yaml
```

**Add these sections:**

```yaml
# Lane Following Settings (PID-based)
lane_following:
  enabled: true
  
  # PID Controller parameters
  pid:
    kp: 0.8              # Proportional gain (start here)
    ki: 0.0              # Integral gain (start at 0)
    kd: 0.3              # Derivative gain
    
    # PID limits
    max_output: 150      # Max steering correction
    min_output: -150
  
  # Speed settings
  base_speed: 120        # Base forward speed
  max_speed: 180
  min_speed: 60
  
  # Steering
  max_steering_angle: 45  # degrees
  steering_sensitivity: 1.0

# AI/Computer Vision Settings
ai:
  # Lane Detection (Camera-based)
  lane_detection:
    enabled: true
    method: 'opencv'
    
    # OpenCV lane detection settings
    roi_top_ratio: 0.5          # Start ROI at 50% from top
    roi_bottom_ratio: 1.0
    canny_low: 50
    canny_high: 150
    hough_threshold: 30
    min_line_length: 40
    max_line_gap: 100
    blur_kernel: 5
```

**Save:** Ctrl+X, Y, Enter

### **Step 4: Install OpenCV (if not installed)**

```bash
source ~/logisticsbot/venv/bin/activate

# Check if OpenCV installed
python3 -c "import cv2; print(cv2.__version__)"

# If not installed:
pip install opencv-python==4.8.1.78
```

---

## 🧪 Testing

### **Test 1: PID Controller**

```bash
cd ~/logisticsbot
source venv/bin/activate

python3 << 'EOF'
from control.pid_controller import PIDController

# Create PID
pid = PIDController(kp=1.0, ki=0.0, kd=0.1)

# Simulate errors
errors = [50, 40, 30, 20, 10, 0, -10, -20]

print("Testing PID Controller:")
print("-" * 60)

for error in errors:
    correction = pid.compute(error, dt=0.1)
    components = pid.get_components()
    
    print(f"Error: {error:+4d} → "
          f"P: {components['p']:+6.1f}, "
          f"D: {components['d']:+6.1f}, "
          f"Output: {correction:+6.1f}")

print("-" * 60)
print("✓ PID Controller working!")
EOF
```

**Expected Output:**
```
Testing PID Controller:
------------------------------------------------------------
Error:  +50 → P:  +50.0, D:   +0.0, Output:  +50.0
Error:  +40 → P:  +40.0, D:   -1.0, Output:  +39.0
...
------------------------------------------------------------
✓ PID Controller working!
```

### **Test 2: Lane Detection (Static Image)**

```bash
# Test with camera
python3 << 'EOF'
import cv2
from perception.lane_detector import detect_line

# Initialize camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("Capturing test frame...")
ret, frame = cap.read()

if ret:
    print("✓ Frame captured")
    
    # Detect lane
    error, x_line, center_x, frame_debug = detect_line(frame)
    
    print(f"Lane Detection Results:")
    print(f"  Error: {error:+d} pixels")
    print(f"  Lane X: {x_line} px")
    print(f"  Center X: {center_x} px")
    
    # Save debug frame
    cv2.imwrite('lane_detection_test.jpg', frame_debug)
    print("✓ Debug frame saved: lane_detection_test.jpg")
else:
    print("✗ Failed to capture frame")

cap.release()
EOF
```

### **Test 3: Full Auto Mode (Dry Run)**

```bash
python3 << 'EOF'
import time
from drivers.motor.arduino_driver import ArduinoDriver
from control.robot_controller import RobotController, AutoModeController
from utils.config_loader import load_config

print("Testing Auto Mode...")

# Load config
config = load_config('config/hardware_config.yaml')

# Initialize driver
driver = ArduinoDriver(config['arduino']['port'])

if not driver.connected:
    print("✗ Arduino not connected!")
    exit(1)

print("✓ Arduino connected")

# Initialize controllers
robot = RobotController(driver, config)
auto_mode = AutoModeController(robot)

# Switch to auto mode
robot.set_mode('auto')
print("✓ Switched to AUTO mode")

# Start auto mode
if auto_mode.start():
    print("✓ Auto mode started")
    print("Running for 10 seconds... (Place robot on track)")
    
    time.sleep(10)
    
    # Get PID status
    pid_status = auto_mode.get_pid_status()
    print(f"\nPID Status:")
    print(f"  Error: {pid_status['error']:+d} px")
    print(f"  Correction: {pid_status['correction']:+.1f}")
    print(f"  P: {pid_status['p']:+.1f}")
    print(f"  I: {pid_status['i']:+.1f}")
    print(f"  D: {pid_status['d']:+.1f}")
    
    # Stop
    auto_mode.stop()
    print("\n✓ Auto mode stopped")
else:
    print("✗ Failed to start auto mode")

# Cleanup
robot.cleanup()
EOF
```

---

## ⚙️ PID Tuning Guide

### **Step 1: Start with P-only**

```yaml
pid:
  kp: 0.5    # Start conservative
  ki: 0.0    # Disable
  kd: 0.0    # Disable
```

**Test:** Robot should follow line but may oscillate

### **Step 2: Increase P until oscillation**

```yaml
kp: 0.8    # Increase gradually
kp: 1.0
kp: 1.2    # Stop when it oscillates
```

**Then reduce P by 20%:**
```yaml
kp: 0.8    # If oscillated at 1.0
```

### **Step 3: Add D to reduce oscillation**

```yaml
kp: 0.8
kd: 0.1    # Start small
```

**Increase D if still oscillating:**
```yaml
kd: 0.3
kd: 0.5
```

### **Step 4: Add I if needed (steady-state error)**

If robot consistently off-center:
```yaml
ki: 0.01   # Start very small
```

**Monitor integral term** - should not grow unbounded

### **Final Example Values:**

```yaml
# Smooth, stable lane following
pid:
  kp: 0.8
  ki: 0.0
  kd: 0.3

# Aggressive, fast response
pid:
  kp: 1.2
  ki: 0.0
  kd: 0.5

# Conservative, safe
pid:
  kp: 0.5
  ki: 0.0
  kd: 0.2
```

---

## 🎮 Running the System

### **Start Application:**

```bash
cd ~/logisticsbot
source venv/bin/activate

python3 main.py
```

### **Access Web Dashboard:**

```
http://<raspberry-pi-ip>:5000
```

### **Select Auto Mode:**

1. Click "Auto" radio button
2. Robot should start following lane
3. Watch Activity Log for PID values
4. Emergency Stop if needed

---

## 📊 Monitoring & Debugging

### **View Logs:**

```bash
# Real-time log monitoring
tail -f data/logs/robot.log

# Filter PID messages
tail -f data/logs/robot.log | grep PID

# Example output:
# PID: error=+045px, P=+36.0, I=+0.0, D=-3.2, correction=+32.8, motors=L87/R153
```

### **Debug Lane Detection:**

```python
# Add to auto_loop in robot_controller.py
if frame_count % 30 == 0:
    cv2.imwrite(f'debug_frame_{frame_count}.jpg', frame_debug)
```

### **Live PID Tuning:**

```python
# While running, adjust PID:
auto_mode.pid.set_gains(kp=1.0, kd=0.4)
```

---

## 🔧 Troubleshooting

### **Problem: Robot oscillates (zigzag)**

```yaml
# Solution: Reduce P, increase D
pid:
  kp: 0.6    # Reduce from 0.8
  kd: 0.5    # Increase from 0.3
```

### **Problem: Robot too slow to respond**

```yaml
# Solution: Increase P
pid:
  kp: 1.2    # Increase from 0.8
```

### **Problem: Robot overshoots curves**

```yaml
# Solution: Increase D (damping)
pid:
  kd: 0.6    # Increase from 0.3
```

### **Problem: Consistent offset from center**

```yaml
# Solution: Add small I term
pid:
  ki: 0.01   # Add integral
```

### **Problem: No lane detected**

```bash
# Check:
1. Camera working: ls /dev/video*
2. Good lighting
3. High contrast line (black on white or white on black)
4. Line width 2-5cm
5. Adjust detection parameters in config
```

### **Problem: Lane detection too sensitive**

```yaml
# Adjust thresholds
lane_detection:
  hough_threshold: 50      # Increase (was 30)
  min_line_length: 60      # Increase (was 40)
```

---

## 📈 Performance Optimization

### **Increase FPS:**

```yaml
camera:
  resolution: [320, 240]   # Lower resolution
  framerate: 60            # Higher FPS
```

### **Reduce CPU Usage:**

```yaml
lane_detection:
  roi_top_ratio: 0.6       # Smaller ROI (was 0.5)
```

### **Balance Speed vs Accuracy:**

```yaml
# Fast but less accurate
hough_threshold: 40
min_line_length: 30

# Slow but more accurate
hough_threshold: 20
min_line_length: 60
```

---

## 🎓 Understanding PID Terms

### **P (Proportional):**
- **What:** Response proportional to error
- **Effect:** Larger error = larger correction
- **Problem:** Can cause oscillation
- **Typical:** 0.5 - 1.5

### **I (Integral):**
- **What:** Sum of all past errors
- **Effect:** Eliminates steady-state error
- **Problem:** Can cause windup and overshoot
- **Typical:** 0.0 - 0.05 (often not needed)

### **D (Derivative):**
- **What:** Rate of change of error
- **Effect:** Dampens oscillations, predicts future error
- **Problem:** Sensitive to noise
- **Typical:** 0.1 - 0.5

---

## ✅ Checklist

### **Before Running:**
- [ ] Arduino connected and firmware uploaded
- [ ] Camera working (`ls /dev/video0`)
- [ ] All Python files copied
- [ ] Configuration updated with PID values
- [ ] Track prepared (high contrast, good lighting)

### **During Run:**
- [ ] Monitor logs for PID values
- [ ] Watch for lane loss warnings
- [ ] Check motor speeds are reasonable
- [ ] Emergency stop accessible

### **After Run:**
- [ ] Review logs for issues
- [ ] Adjust PID values if needed
- [ ] Check camera debug frames
- [ ] Note any improvements needed

---

## 🎯 Next Steps

1. ✅ Test PID controller standalone
2. ✅ Test lane detection with static image
3. ✅ Run dry test with auto mode
4. ✅ Place robot on track and test
5. ✅ Tune PID parameters
6. ✅ Optimize for your specific track
7. ✅ Add features (speed control based on curvature, etc.)

---

**Your lane following system is ready! 🎯🤖🚀**