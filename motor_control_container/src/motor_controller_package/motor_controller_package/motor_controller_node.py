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
        self.serial_port = self.declare_parameter('serial_port', '/dev/serial0').value
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

        # Преобразование в команды для моторов
        max_speed = 1000  # Максимальное значение для платы (-1000 до 1000)
        steer = int(angular_z * 100)  # Угловая скорость -> руль (-1000 до 1000)
        speed = int(linear_x * 100)   # Линейная скорость -> скорость (-1000 до 1000)

        # Ограничение значений
        steer = max(-1000, min(1000, steer))
        speed = max(-1000, min(1000, speed))

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