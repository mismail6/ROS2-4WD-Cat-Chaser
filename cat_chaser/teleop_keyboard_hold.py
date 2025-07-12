#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios
import threading
import time

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard_hold')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Movement parameters
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        self.speed_increment = 0.1
        self.max_speed = 2.0
        self.min_speed = 0.1
        
        # Key bindings (linear, angular)
        self.key_bindings = {
            'w': (1, 0),   # forward
            's': (-1, 0),  # backward
            'a': (0, 1),   # left
            'd': (0, -1),  # right
            'q': (1, 1),   # forward left
            'e': (1, -1),  # forward right
            'z': (-1, 1),  # backward left
            'c': (-1, -1), # backward right
        }
        
        # Track currently pressed keys
        self.pressed_keys = set()
        self.running = True
        
        # Publisher rate (Hz)
        self.publish_rate = 20.0
        
        self.get_logger().info('Teleop Keyboard Hold Node Started')
        self.get_logger().info('Controls:')
        self.get_logger().info('  W/S: Forward/Backward')
        self.get_logger().info('  A/D: Left/Right')
        self.get_logger().info('  Q/E: Forward+Left/Forward+Right')
        self.get_logger().info('  Z/C: Backward+Left/Backward+Right')
        self.get_logger().info('  T: Increase speed')
        self.get_logger().info('  Y: Decrease speed')
        self.get_logger().info('  SPACE: Stop')
        self.get_logger().info('  CTRL+C: Exit')
        self.get_logger().info('Hold keys to move - release to stop!')
        self.get_logger().info(f'Current speed: {self.linear_speed:.1f} m/s')
    
    def get_key(self):
        """Non-blocking key input"""
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None
    
    def calculate_velocities(self):
        """Calculate linear and angular velocities based on pressed keys"""
        if not self.pressed_keys:
            return 0.0, 0.0
        
        total_linear = 0.0
        total_angular = 0.0
        
        for key in self.pressed_keys:
            if key in self.key_bindings:
                linear_dir, angular_dir = self.key_bindings[key]
                total_linear += linear_dir * self.linear_speed
                total_angular += angular_dir * self.angular_speed
        
        # Clamp values to prevent excessive speed from multiple keys
        total_linear = max(-self.linear_speed, min(self.linear_speed, total_linear))
        total_angular = max(-self.angular_speed, min(self.angular_speed, total_angular))
        
        return total_linear, total_angular
    
    def publish_velocities(self):
        """Continuously publish velocities at fixed rate"""
        rate = self.create_rate(self.publish_rate)
        
        while self.running and rclpy.ok():
            linear_vel, angular_vel = self.calculate_velocities()
            self.publish_twist(linear_vel, angular_vel)
            
            try:
                rate.sleep()
            except Exception:
                break
    
    def run(self):
        """Main loop that reads keys and manages pressed keys"""
        # Save original terminal settings
        old_attr = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        
        # Start publisher thread
        publisher_thread = threading.Thread(target=self.publish_velocities)
        publisher_thread.daemon = True
        publisher_thread.start()
        
        try:
            last_key_time = {}
            key_timeout = 0.1  # Timeout for key release detection
            
            while rclpy.ok() and self.running:
                key = self.get_key()
                current_time = time.time()
                
                if key:
                    if key == '\x03':  # Ctrl+C
                        break
                    elif key == ' ':  # Space to stop
                        self.pressed_keys.clear()
                        self.get_logger().info('STOP - All keys released')
                    elif key == 't':  # Increase speed
                        self.increase_speed()
                    elif key == 'y':  # Decrease speed
                        self.decrease_speed()
                    elif key in self.key_bindings:
                        if key not in self.pressed_keys:
                            self.pressed_keys.add(key)
                            linear_dir, angular_dir = self.key_bindings[key]
                            self.get_logger().info(f'Key pressed: {key} (linear_dir={linear_dir}, angular_dir={angular_dir})')
                        last_key_time[key] = current_time
                
                # Check for key releases (keys that haven't been pressed recently)
                keys_to_remove = []
                for pressed_key in self.pressed_keys:
                    if pressed_key in last_key_time:
                        if current_time - last_key_time[pressed_key] > key_timeout:
                            keys_to_remove.append(pressed_key)
                
                for key_to_remove in keys_to_remove:
                    self.pressed_keys.discard(key_to_remove)
                    if key_to_remove in last_key_time:
                        del last_key_time[key_to_remove]
                    self.get_logger().info(f'Key released: {key_to_remove}')
                
                # Keep ROS2 spinning
                rclpy.spin_once(self, timeout_sec=0.01)
                
        finally:
            # Cleanup
            self.running = False
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
            self.pressed_keys.clear()
            self.publish_twist(0.0, 0.0)
            self.get_logger().info('Node shutting down - robot stopped')
    
    def increase_speed(self):
        """Increase linear and angular speeds"""
        old_speed = self.linear_speed
        self.linear_speed = min(self.max_speed, self.linear_speed + self.speed_increment)
        self.angular_speed = min(self.max_speed, self.angular_speed + self.speed_increment)
        self.get_logger().info(f'Speed increased: {old_speed:.1f} -> {self.linear_speed:.1f} m/s')
    
    def decrease_speed(self):
        """Decrease linear and angular speeds"""
        old_speed = self.linear_speed
        self.linear_speed = max(self.min_speed, self.linear_speed - self.speed_increment)
        self.angular_speed = max(self.min_speed, self.angular_speed - self.speed_increment)
        self.get_logger().info(f'Speed decreased: {old_speed:.1f} -> {self.linear_speed:.1f} m/s')
    
    def publish_twist(self, linear, angular):
        """Publish velocity commands to cmd_vel topic"""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = TeleopKeyboard()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()