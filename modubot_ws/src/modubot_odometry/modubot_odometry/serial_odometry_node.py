#!/usr/bin/env python3
import math
import serial
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

def quat_from_yaw(yaw: float) -> Quaternion:
    # roll=pitch=0, yaw=z
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return Quaternion(x=0.0, y=0.0, z=sy, w=cy)

class SerialOdometryNode(Node):
    def __init__(self):
        super().__init__('serial_odometry')

        # ---- Parâmetros ----
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('ticks_per_rev', 90.0)
        self.declare_parameter('wheel_radius', 0.05)     # [m]
        self.declare_parameter('wheel_separation', 0.28) # [m]
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value or 115200

        try:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.02)
            self.get_logger().info(f'[*] Abrindo {port} @ {baud}')
        except Exception as e:
            self.get_logger().fatal(f'Erro abrindo serial: {e}')
            raise

        self.pub_odom = self.create_publisher(Odometry, '/odom', 20)
        self.tf_broadcaster = TransformBroadcaster(self) if self.get_parameter('publish_tf').value else None

        # Estado de pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_stamp: Time = self.get_clock().now()

        # Pré-cálculos
        self.R = float(self.get_parameter('wheel_radius').value)
        self.B = float(self.get_parameter('wheel_separation').value)
        self.N = float(self.get_parameter('ticks_per_rev').value)
        self.m_per_tick = 2.0 * math.pi * self.R / self.N

        self.get_logger().info(f'[*] ticks/rev={self.N}  R={self.R:.3f} m  B={self.B:.3f} m')

        # Timer de leitura/integração (100 Hz tentativa, mas leitura é não-bloqueante)
        self.timer = self.create_timer(0.01, self._spin_once)

    def _spin_once(self):
        # Leia todas as linhas disponíveis neste “tick”
        while True:
            try:
                raw = self.ser.readline()
                if not raw:
                    break
                line = raw.decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                # Esperamos formato: "T <dL> <dR>", onde dL/dR são inteiros (podem ser negativos)
                if not line.startswith('T'):
                    continue

                parts = line.split()
                if len(parts) != 3:
                    continue

                dL = int(parts[1])
                dR = int(parts[2])

                now = self.get_clock().now()
                dt = (now - self.last_stamp).nanoseconds * 1e-9
                if dt <= 0.0:
                    self.last_stamp = now
                    continue
                self.last_stamp = now

                # Converte ticks -> metros (delta por roda)
                sL = dL * self.m_per_tick
                sR = dR * self.m_per_tick

                # Cinemática diferencial (discretização simples)
                ds = 0.5 * (sR + sL)
                dth = (sR - sL) / self.B

                # Integração em coordenadas do robô
                self.yaw += dth
                self.x += ds * math.cos(self.yaw)
                self.y += ds * math.sin(self.yaw)

                vx = ds / dt
                wz = dth / dt

                # Publica /odom
                odom = Odometry()
                odom.header.stamp = now.to_msg()
                odom.header.frame_id = self.get_parameter('frame_id').value
                odom.child_frame_id = self.get_parameter('child_frame_id').value

                odom.pose.pose.position.x = self.move_small(self.x)
                odom.pose.pose.position.y = self.move_small(self.y)
                odom.pose.pose.position.z = 0.0
                odom.pose.pose.orientation = quat_from_yaw(self.yaw)

                odom.twist.twist.linear.x = vx
                odom.twist.twist.angular.z = wz

                self.pub_odom.publish(odom)

                # Publica TF (odom -> base_link), se habilitado
                if self.tf_broadcaster is not None:
                    tf = TransformStamped()
                    tf.header.stamp = now.to_msg()
                    tf.header.frame_id = self.get_parameter('frame_id').value
                    tf.child_frame_id = self.get_parameter('child_frame_id').value
                    tf.transform.translation.x = self.x
                    tf.transform.translation.y = self.y
                    tf.transform.translation.z = 0.0
                    q = quat_from_yaw(self.yaw)
                    tf.transform.rotation = q
                    self.tf_broadcaster.sendTransform(tf)

            except Exception as e:
                self.get_logger().warn(f'Falha parse/serial: {e}')
                break

    @staticmethod
    def move_small(v: float) -> float:
        # opcional: zera valores muito pequenos para evitar ruído numérico
        return 0.0 if abs(v) < 1e-12 else v

def main(args=None):
    rclpy.init(args=args)
    node = SerialOdometryNode()
    try:
        rclpy.spin(node)
    finally:
        if hasattr(node, 'ser') and node.ser:
            try:
                node.ser.close()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()

