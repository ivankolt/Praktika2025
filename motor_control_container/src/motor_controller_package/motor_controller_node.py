#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial
import struct
import zlib
import time
import threading
from typing import Optional


class MotorControllerNode(Node):
    """
    ROS2 нода для управления моторами hoverboard через UART.
    
    Принимает команды Twist и отправляет их на STM32 контроллер
    в формате: [steer_low][steer_high][speed_low][speed_high][crc32_low][crc32_med1][crc32_med2][crc32_high]
    """
    
    def __init__(self):
        super().__init__('motor_controller_node')
        
        # Параметры
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('max_linear_speed', 1.0)  # м/с
        self.declare_parameter('max_angular_speed', 1.0)  # рад/с
        self.declare_parameter('wheel_separation', 0.5)  # м
        
        # Получаем параметры
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        
        # Инициализация UART
        self.serial_connection: Optional[serial.Serial] = None
        self.last_command_time = time.time()
        self.command_lock = threading.Lock()
        
        # Подписка на команды
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Публикация статуса
        self.status_publisher = self.create_publisher(
            String,
            'motor_status',
            10
        )
        
        # Таймер для отправки команд
        self.timer = self.create_timer(0.1, self.send_command_timer)
        
        # Инициализация UART
        self.init_serial()
        
        self.get_logger().info(f'Motor Controller Node запущен')
        self.get_logger().info(f'UART порт: {self.serial_port}')
        self.get_logger().info(f'Скорость: {self.baud_rate}')
        
    def init_serial(self):
        """Инициализация UART соединения"""
        try:
            self.serial_connection = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.timeout,
                write_timeout=self.timeout
            )
            self.get_logger().info(f'UART соединение установлено: {self.serial_port}')
            
            # Отправляем тестовую команду
            self.send_stop_command()
            
        except Exception as e:
            self.get_logger().error(f'Ошибка инициализации UART: {e}')
            self.serial_connection = None
    
    def cmd_vel_callback(self, msg: Twist):
        """Обработчик команд Twist"""
        with self.command_lock:
            self.last_command_time = time.time()
            
            # Ограничиваем скорости
            linear_speed = max(-self.max_linear_speed, min(self.max_linear_speed, msg.linear.x))
            angular_speed = max(-self.max_angular_speed, min(self.max_angular_speed, msg.angular.z))
            
            # Преобразуем в команды для колес (дифференциальный привод)
            left_speed = linear_speed - (angular_speed * self.wheel_separation / 2.0)
            right_speed = linear_speed + (angular_speed * self.wheel_separation / 2.0)
            
            # Масштабируем в диапазон -1000 до 1000
            left_motor = int(left_speed * 1000 / self.max_linear_speed)
            right_motor = int(right_speed * 1000 / self.max_linear_speed)
            
            # Ограничиваем значения
            left_motor = max(-1000, min(1000, left_motor))
            right_motor = max(-1000, min(1000, right_motor))
            
            self.get_logger().debug(f'Команда: linear={linear_speed:.2f}, angular={angular_speed:.2f}')
            self.get_logger().debug(f'Моторы: left={left_motor}, right={right_motor}')
            
            # Отправляем команду
            self.send_motor_command(left_motor, right_motor)
    
    def send_motor_command(self, left_speed: int, right_speed: int):
        """Отправка команды на моторы"""
        if self.serial_connection is None:
            return
        
        try:
            # Вычисляем steer и speed для протокола
            # steer = разность скоростей (поворот)
            # speed = средняя скорость (линейное движение)
            steer = right_speed - left_speed
            speed = (left_speed + right_speed) // 2
            
            # Ограничиваем значения
            steer = max(-1000, min(1000, steer))
            speed = max(-1000, min(1000, speed))
            
            # Создаем пакет данных (первые 4 байта)
            data = struct.pack('<hh', steer, speed)  # little-endian, 2 int16
            
            # Вычисляем CRC32
            crc32 = zlib.crc32(data) & 0xffffffff
            
            # Создаем полный пакет
            packet = data + struct.pack('<I', crc32)  # добавляем CRC32
            
            # Отправляем
            self.serial_connection.write(packet)
            self.serial_connection.flush()
            
            self.get_logger().debug(f'Отправлено: steer={steer}, speed={speed}, crc32={crc32:08x}')
            
        except Exception as e:
            self.get_logger().error(f'Ошибка отправки команды: {e}')
    
    def send_stop_command(self):
        """Отправка команды остановки"""
        self.send_motor_command(0, 0)
    
    def send_command_timer(self):
        """Таймер для проверки таймаута команд"""
        current_time = time.time()
        
        # Если прошло больше 0.5 секунд без команд - останавливаем моторы
        if current_time - self.last_command_time > 0.5:
            with self.command_lock:
                self.send_stop_command()
        
        # Публикуем статус
        status_msg = String()
        if self.serial_connection and self.serial_connection.is_open:
            status_msg.data = "connected"
        else:
            status_msg.data = "disconnected"
        
        self.status_publisher.publish(status_msg)
    
    def destroy_node(self):
        """Очистка ресурсов при завершении"""
        if self.serial_connection:
            self.send_stop_command()
            self.serial_connection.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = MotorControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
