#!/usr/bin/env python3
import math
import sys
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
import tf2_ros

# ================== CẤU HÌNH ĐẢO CHIỀU (QUAN TRỌNG) ==================
# Nếu bánh nào quay ngược, hãy đổi từ 1 thành -1 ở bánh đó
L_ID = -1    # Chiều bánh trái (1 hoặc -1)
R_ID = 1   # Chiều bánh phải (1 hoặc -1) - Thường robot diff-drive bánh phải sẽ ngược bánh trái

# ================== THÔNG SỐ ROBOT (Theo Xacro) ==================
WHEEL_DIAMETER = 0.17    # m (Từ wheel_radius="0.085")
WHEEL_DIST     = 0.37    # m (Từ wheel_offset_y="0.185" * 2)
TICKS_PER_REV  = 2800.0  # Số xung/vòng của ESP32
MAX_RPM        = 380.0

SERIAL_PORT = "/dev/ttyUSB0"
BAUD = 115200
CMD_SEND_PERIOD = 0.05   # 50 ms

class SerialBridgeOdom(Node):
    def __init__(self):
        super().__init__('serial_bridge_odom')

        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.02)
            self.get_logger().info("✅ Đã kết nối ESP32!")
        except Exception as e:
            self.get_logger().error(f"❌ Lỗi Serial: {e}")
            sys.exit(1)

        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.left_ticks_prev = 0
        self.right_ticks_prev = 0
        self.first_run = True
        self.last_set_l = 0.0
        self.last_set_r = 0.0

        self.cmd_timer = self.create_timer(CMD_SEND_PERIOD, self.send_latest_cmd)
        self.read_timer = self.create_timer(0.005, self.read_serial)

    def cmd_callback(self, msg: Twist):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Tính toán vận tốc lý thuyết (m/s)
        vel_left  = linear_x - (angular_z * WHEEL_DIST / 2.0)
        vel_right = linear_x + (angular_z * WHEEL_DIST / 2.0)

        # Chuyển sang RPM và áp dụng đảo chiều motor
        rpm_l_raw = (vel_left  * 60.0) / (math.pi * WHEEL_DIAMETER)
        rpm_r_raw = (vel_right * 60.0) / (math.pi * WHEEL_DIAMETER)

        # Áp dụng L_ID và R_ID để motor quay đúng hướng tiến/lùi
        self.last_set_l = max(min(rpm_l_raw * L_ID, MAX_RPM), -MAX_RPM)
        self.last_set_r = max(min(rpm_r_raw * R_ID, MAX_RPM), -MAX_RPM)

    def send_latest_cmd(self):
        cmd = f"r {self.last_set_l:.1f} {self.last_set_r:.1f}\n"
        try:
            self.ser.write(cmd.encode('utf-8'))
        except Exception:
            pass

    def read_serial(self):
        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line: continue
                
                if line.startswith('e'):
                    parts = line.split()
                    if len(parts) == 3:
                        try:
                            # Đọc tick và áp dụng đảo chiều để tick "tiến" luôn là số dương
                            l_ticks = int(parts[1]) * L_ID
                            r_ticks = int(parts[2]) * R_ID
                            self.update_odometry(l_ticks, r_ticks)
                        except ValueError: pass
                else:
                    print(f"ESP32 Log: {line}")
        except Exception as e:
            self.get_logger().warn(f"Read error: {e}")

    def update_odometry(self, l_ticks, r_ticks):
        if self.first_run:
            self.left_ticks_prev = l_ticks
            self.right_ticks_prev = r_ticks
            self.first_run = False
            return

        dist_per_tick = (math.pi * WHEEL_DIAMETER) / TICKS_PER_REV
        d_left = (l_ticks - self.left_ticks_prev) * dist_per_tick
        d_right = (r_ticks - self.right_ticks_prev) * dist_per_tick
        
        self.left_ticks_prev = l_ticks
        self.right_ticks_prev = r_ticks

        ds = (d_left + d_right) / 2.0
        d_th = (d_right - d_left) / WHEEL_DIST

        # Cập nhật vị trí
        if abs(d_th) < 1e-6:
            self.x += ds * math.cos(self.th + d_th / 2.0)
            self.y += ds * math.sin(self.th + d_th / 2.0)
            self.th += d_th
        else:
            radius = ds / d_th
            old_th = self.th
            self.th += d_th
            self.x += radius * (math.sin(self.th) - math.sin(old_th))
            self.y += -radius * (math.cos(self.th) - math.cos(old_th))

        self.th = math.atan2(math.sin(self.th), math.cos(self.th))

        # DEBUG: Sau khi áp dụng L_ID/R_ID, khi tiến thì Ticks và X phải tăng
        deg = math.degrees(self.th)
        print(f"TICKS (fixed): L={l_ticks}, R={r_ticks} | ODOM: X={self.x:.2f}, Y={self.y:.2f}, {deg:.1f}°")

        self.publish_data()

    def publish_data(self):
        now = self.get_clock().now().to_msg()
        q = self.euler_to_quaternion(0, 0, self.th)
        
        # TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        # Odom message
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        self.odom_pub.publish(odom)

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        return Quaternion(w=cr*cp*cy+sr*sp*sy, x=sr*cp*cy-cr*sp*sy, y=cr*sp*cy+sr*cp*sy, z=cr*cp*sy-sr*sp*cy)

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeOdom()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
