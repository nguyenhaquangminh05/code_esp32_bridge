#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import math
import sys
import time

# ================== ROBOT PARAMS ==================
WHEEL_DIAMETER = 0.162   # m
WHEEL_DIST     = 0.37    # m
TICKS_PER_REV  = 1400.0  # tick / 1 vòng bánh
MAX_RPM        = 382.71

SERIAL_PORT = "/dev/ttyUSB0"
BAUD = 115200

class SerialBridgeDebug(Node):
    def __init__(self):
        super().__init__('serial_bridge_debug')

        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.05)
            print("Connected ESP32!")
        except Exception as e:
            print(f"{e}")
            sys.exit(1)

        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # last setpoint
        self.last_set_l = 0.0
        self.last_set_r = 0.0

        # ticks state
        self.first_run = True
        self.left_ticks_prev = 0
        self.right_ticks_prev = 0
        self.t_prev = time.monotonic()

        # read serial ~50Hz
        self.timer = self.create_timer(0.02, self.update)

    def cmd_callback(self, msg: Twist):
        linear_x  = msg.linear.x
        angular_z = msg.angular.z

        vel_left  = linear_x - (angular_z * WHEEL_DIST / 2.0)
        vel_right = linear_x + (angular_z * WHEEL_DIST / 2.0)

        rpm_left  = vel_left  * 60.0 / (math.pi * WHEEL_DIAMETER)
        rpm_right = vel_right * 60.0 / (math.pi * WHEEL_DIAMETER)

        rpm_left  = max(min(rpm_left,  MAX_RPM), -MAX_RPM)
        rpm_right = max(min(rpm_right, MAX_RPM), -MAX_RPM)

        self.last_set_l = rpm_left
        self.last_set_r = rpm_right

        cmd = f"r {rpm_left:.1f} {rpm_right:.1f}\n"
        try:
            self.ser.write(cmd.encode('utf-8'))
        except:
            pass

    def update(self):
        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line.startswith('e'):
                    continue

                parts = line.split()
                if len(parts) != 3:
                    continue

                left_ticks = int(parts[1])
                right_ticks = int(parts[2])

                # first sample init
                if self.first_run:
                    self.left_ticks_prev = left_ticks
                    self.right_ticks_prev = right_ticks
                    self.t_prev = time.monotonic()
                    self.first_run = False
                    return

                t_now = time.monotonic()
                dt = t_now - self.t_prev
                if dt <= 0.0:
                    return

                dL = left_ticks - self.left_ticks_prev
                dR = right_ticks - self.right_ticks_prev
                self.left_ticks_prev = left_ticks
                self.right_ticks_prev = right_ticks
                self.t_prev = t_now

                # rpm feedback from ticks
                rpm_fb_l = (dL * 60.0) / (TICKS_PER_REV * dt)
                rpm_fb_r = (dR * 60.0) / (TICKS_PER_REV * dt)

                # LOG ONLY v_set and v_fb
                print(f"v_set {self.last_set_l:.1f} {self.last_set_r:.1f} | v_fb {rpm_fb_l:.1f} {rpm_fb_r:.1f}")

        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStop program")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
