#!/usr/bin/env python3
import math
import sys
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ================== ROBOT PARAMS ==================
WHEEL_DIAMETER = 0.162   # m
WHEEL_DIST     = 0.37    # m
MAX_RPM        = 380.0

SERIAL_PORT = "/dev/ttyUSB0"
BAUD = 115200

# Gửi lại lệnh định kỳ để ESP32 không timeout
CMD_SEND_PERIOD = 0.05   # 50 ms < 500 ms timeout


class SerialBridgeDebug(Node):
    def __init__(self):
        super().__init__('serial_bridge_debug')

        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.02)
            print("Connected ESP32!")
        except Exception as e:
            print(f"{e}")
            sys.exit(1)

        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # rpm setpoint gần nhất
        self.last_set_l = 0.0
        self.last_set_r = 0.0

        # timer gửi lệnh định kỳ
        self.cmd_timer = self.create_timer(CMD_SEND_PERIOD, self.send_latest_cmd)

        # timer đọc debug từ ESP32
        self.read_timer = self.create_timer(0.005, self.read_serial)

    def cmd_callback(self, msg: Twist):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        vel_left  = linear_x - (angular_z * WHEEL_DIST / 2.0)
        vel_right = linear_x + (angular_z * WHEEL_DIST / 2.0)

        rpm_left  = vel_left  * 60.0 / (math.pi * WHEEL_DIAMETER)
        rpm_right = vel_right * 60.0 / (math.pi * WHEEL_DIAMETER)

        rpm_left  = max(min(rpm_left,  MAX_RPM), -MAX_RPM)
        rpm_right = max(min(rpm_right, MAX_RPM), -MAX_RPM)

        self.last_set_l = rpm_left
        self.last_set_r = rpm_right

    def send_latest_cmd(self):
        cmd = f"r {self.last_set_l:.1f} {self.last_set_r:.1f}\n"
        try:
            self.ser.write(cmd.encode('utf-8'))
        except Exception:
            pass

    def read_serial(self):
        try:
            while self.ser.in_waiting > 0:
                raw = self.ser.readline()
                try:
                    line = raw.decode('utf-8', errors='ignore').strip()
                except Exception:
                    continue

                if not line:
                    continue

                # In nguyên dòng debug từ ESP32
                print(line)

        except Exception:
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
