#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct

class HoverboardCommandNode(Node):
    def __init__(self):
        super().__init__('hoverboard_command_node')

        # Параметры UART
        self.serial_port = self.declare_parameter('serial_port', '/dev/ttyUSB0').value
        self.baud_rate = self.declare_parameter('baud_rate', 115200).value

        # Подключение к последовательному порту
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to {self.serial_port} at {self.baud_rate} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            rclpy.shutdown()

        # Подписка на топик /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def calculate_crc32(self, data):
        """
        Вычисление CRC32 для данных.
        """
        crc = 0xFFFFFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xEDB88320
                else:
                    crc >>= 1
        return crc ^ 0xFFFFFFFF

    def send_packet(self, steer, speed):
        """
        Отправка пакета на плату.
        """
        # Формирование структуры Serialcommand
        command = struct.pack('<hh', steer, speed)  # '<' для little-endian, 'h' для int16
        crc = self.calculate_crc32(command)
        packet = command + struct.pack('<I', crc)  # Добавление CRC как uint32

        # Отправка пакета
        self.ser.write(packet)
        self.get_logger().info(f"Sent packet: steer={steer}, speed={speed}")

    def cmd_vel_callback(self, msg):
        """
        Обработчик сообщений из топика /cmd_vel.
        Преобразует линейную и угловую скорость в команды для моторов.
        """
        linear_x = msg.linear.x  # Линейная скорость (м/с)
        angular_z = msg.angular.z  # Угловая скорость (рад/с)
        
        # Параметры дифференциального привода
        wheel_separation = 0.5  # Расстояние между колесами в метрах
        max_linear_speed = 1.0  # Максимальная линейная скорость (м/с)
        max_angular_speed = 1.0  # Максимальная угловая скорость (рад/с)
        
        # Ограничиваем входные скорости
        linear_x = max(-max_linear_speed, min(max_linear_speed, linear_x))
        angular_z = max(-max_angular_speed, min(max_angular_speed, angular_z))
        
        # Вычисляем скорости для каждого колеса (дифференциальный привод)
        left_speed = linear_x - (angular_z * wheel_separation / 2.0)
        right_speed = linear_x + (angular_z * wheel_separation / 2.0)
        
        # Преобразуем в команды для протокола STM32
        # steer = разность скоростей (поворот)
        # speed = средняя скорость (линейное движение)
        steer = int((right_speed - left_speed) * 1000 / max_linear_speed)
        speed = int(((left_speed + right_speed) / 2.0) * 1000 / max_linear_speed)
        
        # Ограничение значений
        steer = max(-1000, min(1000, steer))
        speed = max(-1000, min(1000, speed))
        
        self.get_logger().info(f"Input: linear={linear_x:.2f}, angular={angular_z:.2f}")
        self.get_logger().info(f"Wheels: left={left_speed:.2f}, right={right_speed:.2f}")
        self.get_logger().info(f"Output: steer={steer}, speed={speed}")

        # Отправка данных
        self.send_packet(steer, speed)

def main(args=None):
    rclpy.init(args=args)
    node = HoverboardCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()