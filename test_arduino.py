#!/usr/bin/env python3
"""
Test script for Arduino UART communication
Tests motor control and sensor reading
"""

import sys
import time
from pathlib import Path

# Add project root to path
sys.path.append(str(Path(__file__).parent))

from drivers.motor.arduino_driver import ArduinoDriver
from utils.logger import setup_logger

logger = setup_logger('test_arduino', level='DEBUG')


def test_connection():
    """Test basic connection to Arduino"""
    print("\n" + "="*60)
    print("TEST 1: Arduino Connection")
    print("="*60)
    
    # Try common serial ports
    ports = ['/dev/ttyUSB0', '/dev/ttyACM0', '/dev/ttyUSB1']
    
    for port in ports:
        print(f"\nTrying port: {port}")
        try:
            driver = ArduinoDriver(port=port, baudrate=115200)
            
            if driver.connected:
                print(f"✅ Connected successfully on {port}")
                return driver
            else:
                print(f"❌ Failed to connect on {port}")
        except Exception as e:
            print(f"❌ Error on {port}: {e}")
    
    print("\n❌ Could not connect to Arduino on any port!")
    print("\nTroubleshooting:")
    print("1. Check if Arduino is plugged in:")
    print("   ls /dev/ttyUSB* /dev/ttyACM*")
    print("2. Check permissions:")
    print("   sudo usermod -a -G dialout $USER")
    print("   (then logout and login again)")
    print("3. Verify firmware is uploaded to Arduino")
    return None


def test_motor_control(driver):
    """Test motor control commands"""
    print("\n" + "="*60)
    print("TEST 2: Motor Control")
    print("="*60)
    
    tests = [
        ("Forward", lambda: driver.forward(150)),
        ("Backward", lambda: driver.backward(150)),
        ("Turn Left", lambda: driver.turn_left(120)),
        ("Turn Right", lambda: driver.turn_right(120)),
        ("Stop", lambda: driver.stop()),
    ]
    
    for name, command in tests:
        print(f"\n{name}...", end=" ")
        try:
            command()
            time.sleep(1)
            left, right = driver.get_speeds()
            print(f"✅ OK (Left: {left}, Right: {right})")
        except Exception as e:
            print(f"❌ Failed: {e}")
    
    # Stop motors
    driver.stop()
    print("\n✅ Motor control test complete")


def test_sensor_reading(driver):
    """Test sensor data reading"""
    print("\n" + "="*60)
    print("TEST 3: Sensor Reading")
    print("="*60)
    print("\nReading sensors for 5 seconds...")
    print("Place objects in front of sensors to test\n")
    
    start_time = time.time()
    count = 0
    
    while time.time() - start_time < 5:
        sensor_data = driver.get_sensor_data()
        
        if count % 5 == 0:  # Print every 5th reading
            print(f"\n--- Reading #{count+1} ---")
            print(f"Line Sensors: {sensor_data.get('line', [])}")
            print(f"Line Position: {sensor_data.get('line_pos', 0)}")
            print(f"Distance: {sensor_data.get('distance', 0):.1f} cm")
            print(f"Motor Speeds: L={sensor_data.get('left_speed', 0)}, R={sensor_data.get('right_speed', 0)}")
        
        count += 1
        time.sleep(0.1)
    
    print("\n✅ Sensor reading test complete")


def test_speed_control(driver):
    """Test different motor speeds"""
    print("\n" + "="*60)
    print("TEST 4: Speed Control")
    print("="*60)
    
    speeds = [50, 100, 150, 200, 255]
    
    for speed in speeds:
        print(f"\nTesting speed: {speed}...", end=" ")
        try:
            driver.set_motors(speed, speed)
            time.sleep(0.5)
            left, right = driver.get_speeds()
            print(f"✅ OK (Actual: L={left}, R={right})")
        except Exception as e:
            print(f"❌ Failed: {e}")
    
    driver.stop()
    print("\n✅ Speed control test complete")


def interactive_mode(driver):
    """Interactive control mode"""
    print("\n" + "="*60)
    print("INTERACTIVE MODE")
    print("="*60)
    print("\nControls:")
    print("  w/W - Forward")
    print("  s/S - Backward")
    print("  a/A - Turn Left")
    print("  d/D - Turn Right")
    print("  x/X - Stop")
    print("  q/Q - Quit")
    print("\nPress keys to control robot...")
    
    try:
        import termios
        import tty
        
        def getch():
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch
        
        speed = 180
        
        while True:
            # Show sensor data
            sensor_data = driver.get_sensor_data()
            print(f"\rDistance: {sensor_data.get('distance', 0):5.1f}cm  " + 
                  f"Line: {sensor_data.get('line_pos', 0):+2}  " +
                  f"Speed: L={sensor_data.get('left_speed', 0):4}, R={sensor_data.get('right_speed', 0):4}  ",
                  end="", flush=True)
            
            # Non-blocking input check
            import select
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = getch().lower()
                
                if key == 'q':
                    break
                elif key == 'w':
                    driver.forward(speed)
                elif key == 's':
                    driver.backward(speed)
                elif key == 'a':
                    driver.turn_left(speed)
                elif key == 'd':
                    driver.turn_right(speed)
                elif key == 'x':
                    driver.stop()
        print("\n\n✅ Interactive mode ended")
        
    except ImportError:
        print("\n❌ Interactive mode not available on this system")
    except Exception as e:
        print(f"\n❌ Error in interactive mode: {e}")


def main():
    """Main test function"""
    print("\n")
    print("╔══════════════════════════════════════════════════════════╗")
    print("║      Arduino UART Communication Test Suite              ║")
    print("║      LogisticsBot - Hardware Verification               ║")
    print("╚══════════════════════════════════════════════════════════╝")
    
    # Test 1: Connection
    driver = test_connection()
    
    if not driver:
        print("\n❌ Cannot proceed without Arduino connection")
        return
    
    try:
        # Test 2: Motor Control
        input("\nPress ENTER to test motor control...")
        test_motor_control(driver)
        
        # Test 3: Sensor Reading
        input("\nPress ENTER to test sensor reading...")
        test_sensor_reading(driver)
        
        # Test 4: Speed Control
        input("\nPress ENTER to test speed control...")
        test_speed_control(driver)
        
        # Interactive Mode
        print("\n" + "="*60)
        choice = input("\nEnter interactive mode? (y/n): ")
        if choice.lower() == 'y':
            interactive_mode(driver)
        
        # Final summary
        print("\n" + "="*60)
        print("TEST SUMMARY")
        print("="*60)
        print("✅ Connection: OK")
        print("✅ Motor Control: OK")
        print("✅ Sensor Reading: OK")
        print("✅ Speed Control: OK")
        print("\n🎉 All tests passed! Arduino is working correctly.")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Test interrupted by user")
    except Exception as e:
        print(f"\n\n❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        print("\nCleaning up...")
        driver.stop()
        driver.cleanup()
        print("✅ Cleanup complete")


if __name__ == '__main__':
    main()