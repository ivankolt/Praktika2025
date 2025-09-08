import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from math import sin, cos

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.publisher_ = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Инициализация переменных
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Пример данных с энкодеров и IMU
        self.v_left = 0.5  # Скорость левого колеса (м/с)
        self.v_right = 0.6  # Скорость правого колеса (м/с)
        self.wheel_base = 0.5  # Расстояние между колесами (м)
        self.imu_angular_velocity = 0.1  # Угловая скорость (рад/с)

    def timer_callback(self):
        # Расчет линейной и угловой скорости
        v = (self.v_left + self.v_right) / 2
        omega = (self.v_right - self.v_left) / self.wheel_base

        # Обновление положения
        dt = 0.1  # Временной шаг
        self.x += v * cos(self.theta) * dt
        self.y += v * sin(self.theta) * dt
        self.theta += omega * dt

        # Публикация трансформации odom -> base_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sin(self.theta / 2)
        t.transform.rotation.w = cos(self.theta / 2)
        self.tf_broadcaster.sendTransform(t)

        # Публикация одометрии
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = sin(self.theta / 2)
        msg.pose.pose.orientation.w = cos(self.theta / 2)

        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = omega

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    odom_publisher = OdomPublisher()
    rclpy.spin(odom_publisher)
    odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()