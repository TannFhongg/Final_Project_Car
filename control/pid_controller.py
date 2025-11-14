"""
PID Controller for Lane Following
Complete implementation with anti-windup, output clamping, and derivative smoothing
"""

import time


class PIDController:
    """
    PID Controller with advanced features:
    - Anti-windup (prevents integral term from growing unbounded)
    - Output clamping (limits controller output)
    - Derivative smoothing (reduces noise in derivative term)
    - Automatic reset on mode change
    """
    
    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.0,
                 output_min: float = -255, output_max: float = 255,
                 derivative_smoothing: float = 0.5):
        """
        Initialize PID controller
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            output_min: Minimum output value (clamp)
            output_max: Maximum output value (clamp)
            derivative_smoothing: Smoothing factor for derivative (0-1)
                                 0 = no smoothing, 1 = maximum smoothing
        """
        # PID gains
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # Output limits
        self.output_min = output_min
        self.output_max = output_max
        
        # Internal state
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0
        self.derivative_smoothing = derivative_smoothing
        
        # Timing
        self.last_time = None
        
        # Anti-windup limits (prevent integral from growing too large)
        self.integral_min = -100.0
        self.integral_max = 100.0
        
        # Statistics (for tuning)
        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0
        self.last_output = 0.0
    
    def compute(self, error: float, dt: float = None) -> float:
        """
        Compute PID control output
        
        Args:
            error: Current error (setpoint - measured_value)
                   For lane following: (frame_center - line_x_position)
            dt: Time step in seconds (if None, will calculate from last call)
        
        Returns:
            Control output (correction value)
        """
        # Calculate dt if not provided
        if dt is None:
            current_time = time.time()
            if self.last_time is None:
                dt = 0.0
                self.last_time = current_time
            else:
                dt = current_time - self.last_time
                self.last_time = current_time
        
        # Prevent division by zero
        if dt <= 0:
            dt = 0.001
        
        # ===== PROPORTIONAL TERM =====
        p_term = self.kp * error
        
        # ===== INTEGRAL TERM (with anti-windup) =====
        self.integral += error * dt
        
        # Anti-windup: Clamp integral term
        self.integral = max(self.integral_min, min(self.integral_max, self.integral))
        
        i_term = self.ki * self.integral
        
        # ===== DERIVATIVE TERM (with smoothing) =====
        # Calculate derivative
        derivative = (error - self.prev_error) / dt
        
        # Smooth derivative to reduce noise
        smoothed_derivative = (self.derivative_smoothing * self.prev_derivative + 
                              (1 - self.derivative_smoothing) * derivative)
        
        d_term = self.kd * smoothed_derivative
        
        # Update history
        self.prev_error = error
        self.prev_derivative = smoothed_derivative
        
        # ===== COMPUTE OUTPUT =====
        output = p_term + i_term + d_term
        
        # Clamp output to limits
        output = max(self.output_min, min(self.output_max, output))
        
        # Store for debugging/tuning
        self.last_p = p_term
        self.last_i = i_term
        self.last_d = d_term
        self.last_output = output
        
        return output
    
    def reset(self):
        """Reset PID controller state"""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0
        self.last_time = None
        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0
        self.last_output = 0.0
    
    def set_gains(self, kp: float = None, ki: float = None, kd: float = None):
        """Update PID gains on the fly"""
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
    
    def set_output_limits(self, output_min: float, output_max: float):
        """Update output limits"""
        self.output_min = output_min
        self.output_max = output_max
    
    def get_components(self) -> dict:
        """
        Get individual PID components for debugging/tuning
        
        Returns:
            Dictionary with P, I, D terms and total output
        """
        return {
            'p': self.last_p,
            'i': self.last_i,
            'd': self.last_d,
            'output': self.last_output,
            'integral_state': self.integral
        }
    
    def __str__(self) -> str:
        """String representation for debugging"""
        return (f"PID(kp={self.kp}, ki={self.ki}, kd={self.kd}, "
                f"P={self.last_p:.2f}, I={self.last_i:.2f}, D={self.last_d:.2f}, "
                f"output={self.last_output:.2f})")


class AdaptivePIDController(PIDController):
    """
    Adaptive PID that adjusts gains based on error magnitude
    Useful for different situations (straight line vs sharp curves)
    """
    
    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.0,
                 output_min: float = -255, output_max: float = 255,
                 derivative_smoothing: float = 0.5):
        super().__init__(kp, ki, kd, output_min, output_max, derivative_smoothing)
        
        # Base gains (will be scaled)
        self.base_kp = kp
        self.base_ki = ki
        self.base_kd = kd
        
        # Adaptive thresholds
        self.small_error_threshold = 10  # pixels
        self.large_error_threshold = 50  # pixels
    
    def compute(self, error: float, dt: float = None) -> float:
        """
        Compute with adaptive gains based on error magnitude
        """
        abs_error = abs(error)
        
        # Small error: reduce gains for stability
        if abs_error < self.small_error_threshold:
            scale = 0.7
        # Large error: increase gains for quick response
        elif abs_error > self.large_error_threshold:
            scale = 1.3
        # Medium error: normal gains
        else:
            scale = 1.0
        
        # Apply scaling
        self.kp = self.base_kp * scale
        self.kd = self.base_kd * scale
        # Keep ki constant to avoid windup issues
        
        return super().compute(error, dt)
    
    def set_gains(self, kp: float = None, ki: float = None, kd: float = None):
        """Update base gains"""
        super().set_gains(kp, ki, kd)
        self.base_kp = self.kp
        self.base_ki = self.ki
        self.base_kd = self.kd