"""
Robot Controller - Lite Version + IMU
Features: Auto Mode, Obstacle Avoidance, Precise Turning with IMU
"""

import threading
import time
import logging
# Import các module cần thiết
from perception.lane_detector import detect_line
from perception.camera_manager import get_web_camera
from perception.object_detector import ObjectDetector
from control.pid_controller import PIDController
# [QUAN TRỌNG] Import IMU
from perception.imu_sensor_fusion import IMUSensorFusion 

logger = logging.getLogger(__name__)

class RobotController:
    def __init__(self, motor_driver, config: dict):
        self.driver = motor_driver
        self.config = config
        self.is_running = False
        self.thread = None
        
        # 1. Cấu hình An Toàn
        safety_cfg = config.get('safety', {})
        self.SAFE_DISTANCE = safety_cfg.get('min_safe_distance', 25.0)
        self.is_avoiding = False
        
        # 2. AI & PID
        self.detector = ObjectDetector(model_path='data/models/best_ncnn_model', conf_threshold=0.5)
        
        pid_cfg = config.get('lane_following', {}).get('pid', {})
        self.pid = PIDController(
            kp=pid_cfg.get('kp', 0.8), ki=pid_cfg.get('ki', 0.0), kd=pid_cfg.get('kd', 0.3),
            output_min=-255, output_max=255
        )
        
        # 3. Tốc độ & Cấu hình
        lane_cfg = config.get('lane_following', {})
        self.base_speed = lane_cfg.get('base_speed', 150)
        self.default_speed = self.base_speed
        self.detection_config = config.get('ai', {}).get('lane_detection', {})
        
        # 4. Khoảng cách Biển báo
        self.DIST_PREPARE = 140
        self.DIST_EXECUTE = 170
        
        # 5. Debug Image
        self.latest_debug_frame = None
        
        # 6. KHỞI TẠO IMU (QUAN TRỌNG)
        try:
            self.imu = IMUSensorFusion()
            if self.imu.connected:
                self.imu.start()
                logger.info("✅ IMU Connected & Started")
            else:
                logger.warning("⚠️ IMU hardware not found")
                self.imu = None
        except Exception as e:
            logger.error(f"❌ IMU Init Error: {e}")
            self.imu = None

    def start(self):
        if not self.is_running:
            self.camera = get_web_camera(self.config)
            if not self.camera.is_running(): self.camera.start()
            
            self.is_running = True
            self.base_speed = self.default_speed
            self.pid.reset()
            self.is_avoiding = False
            
            # Reset IMU drift khi bắt đầu chạy (Tùy chọn)
            if self.imu: self.imu.reset_yaw()
            
            self.thread = threading.Thread(target=self._auto_loop, daemon=True)
            self.thread.start()
            logger.info("LITE AUTO MODE STARTED (WITH IMU)")

    def stop(self):
        self.is_running = False
        if self.thread: self.thread.join(timeout=1.0)
        self.driver.stop()
        logger.info("LITE AUTO MODE STOPPED")

    def cleanup(self):
        self.stop()
        if self.imu: self.imu.stop()
        self.driver.cleanup()

    def smart_turn(self, target_angle, speed=220):
        """
        Hàm rẽ chính xác sử dụng IMU.
        Nếu không có IMU, tự động chuyển về rẽ theo thời gian.
        """
        # --- TRƯỜNG HỢP KHÔNG CÓ IMU (FALLBACK) ---
        if not self.imu or not self.imu.connected:
            logger.warning(f"⚠️ No IMU -> Blind turn {target_angle}°")
            duration = 0.6 * (abs(target_angle) / 90.0)
            if target_angle > 0: self.driver.turn_left(speed)
            else: self.driver.turn_right(speed)
            time.sleep(duration)
            self.driver.stop()
            return

        # --- TRƯỜNG HỢP CÓ IMU (SMART TURN) ---
        logger.info(f"🔄 IMU Turn: Target {target_angle}°")
        self.imu.reset_yaw() # Reset góc về 0
        
        start_time = time.time()
        
        while True:
            # Lấy góc hiện tại
            current_yaw = self.imu.get_yaw()
            
            # Tính sai số (còn thiếu bao nhiêu độ?)
            error = abs(target_angle) - abs(current_yaw)
            
            # 1. Đã đến đích (sai số < 3 độ)
            if error <= 3.0:
                break
            
            # 2. Hết giờ (Timeout 4s) - Tránh xe quay mãi không dừng
            if time.time() - start_time > 4.0:
                logger.warning("⚠️ Turn Timeout!")
                break
            
            # 3. Điều khiển tốc độ (Giảm tốc khi gần đến đích)
            if error > 30:
                turn_speed = speed      # Quay nhanh
            elif error > 10:
                turn_speed = 180        # Quay vừa
            else:
                turn_speed = 130        # Quay chậm để chốt góc
            
            # Gửi lệnh quay
            if target_angle > 0: # Góc dương -> Rẽ Trái
                if current_yaw > target_angle: break # Lố đà -> Dừng
                self.driver.turn_left(turn_speed)
            else:                # Góc âm -> Rẽ Phải
                if current_yaw < target_angle: break # Lố đà -> Dừng
                self.driver.turn_right(turn_speed)
                
            time.sleep(0.01)

        self.driver.stop()
        time.sleep(0.2) # Dừng nghỉ một chút cho ổn định

    def perform_avoidance(self):
        """
        Kịch bản né vật cản (Có sử dụng IMU nếu có)
        """
        self.is_avoiding = True
        logger.warning(">>> AVOIDING OBSTACLE <<<")
        
        self.driver.stop(); time.sleep(0.5)
        
        # 1. Lùi lại
        self.driver.backward(150); time.sleep(0.8)
        
        # 2. Rẽ Trái (Né ra) - Dùng Smart Turn (90 độ hoặc 45 độ tùy không gian)
        # Ở đây giả sử né 45 độ là đủ
        self.smart_turn(60) 
        
        # 3. Đi Thẳng (Vượt qua)
        self.driver.forward(150); time.sleep(1.2)
        
        # 4. Rẽ Phải (Về làn)
        self.smart_turn(-60)
        
        # 5. Ổn định
        self.driver.stop(); time.sleep(0.2)
        self.pid.reset()
        self.is_avoiding = False

    def _auto_loop(self):
        prev_time = time.time()
        
        while self.is_running:
            try:
                # --- 1. KIỂM TRA VẬT CẢN ---
                dist = self.driver.get_distance()
                if 0 < dist < self.SAFE_DISTANCE and not self.is_avoiding:
                    logger.warning(f"Obstacle: {dist}cm -> AVOIDING")
                    self.perform_avoidance()
                    continue

                # --- 2. LẤY ẢNH ---
                frame = self.camera.capture_frame()
                if frame is None:
                    time.sleep(0.1); continue

                # --- 3. NHẬN DIỆN BIỂN BÁO ---
                detections, _ = self.detector.detect(frame)
                sign_action = False
                
                if detections:
                    sign = max(detections, key=lambda x: x['w'] * x['h'])
                    name = sign['class_name']
                    size = max(sign['w'], sign['h'])
                    
                    if self.DIST_PREPARE <= size <= self.DIST_EXECUTE:
                        logger.info(f"Sign Action: {name}")
                        if name in ['stop_sign', 'red_light']:
                            self.driver.stop(); time.sleep(0.1)
                            sign_action = True
                            
                        # SỬA DỤNG SMART TURN CHO BIỂN BÁO
                        elif name == 'left_turn_sign':
                            self.smart_turn(90); sign_action = True
                            
                        elif name == 'right_turn_sign':
                            self.smart_turn(-90); sign_action = True
                            
                        elif name == 'speed_limit_signs':
                            self.base_speed = 100
                        elif name == 'green_light':
                            self.base_speed = self.default_speed

                if sign_action: continue

                # --- 4. CHẠY THEO LÀN ---
                error, _, _, debug_frame = detect_line(frame, self.detection_config)
                self.latest_debug_frame = debug_frame 
                
                cur_time = time.time()
                dt = cur_time - prev_time
                prev_time = cur_time
                
                correction = self.pid.compute(error, dt)
                left = max(-255, min(255, int(self.base_speed - correction)))
                right = max(-255, min(255, int(self.base_speed + correction)))
                
                self.driver.set_motors(left, right)
                time.sleep(0.03)

            except Exception as e:
                logger.error(f"Loop Error: {e}")
                break
        
        self.driver.stop()