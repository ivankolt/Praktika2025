#!/usr/bin/env python3
"""
Тестовый скрипт для проверки связи с hoverboard через UART.
Отправляет тестовые команды и проверяет ответы.
"""

import serial
import struct
import zlib
import time
import sys
import argparse


def calculate_crc32(data):
    """Вычисление CRC32 для данных"""
    return zlib.crc32(data) & 0xffffffff


def send_motor_command(ser, left_speed, right_speed):
    """Отправка команды на моторы"""
    # Вычисляем steer и speed для протокола
    steer = right_speed - left_speed
    speed = (left_speed + right_speed) // 2
    
    # Ограничиваем значения
    steer = max(-1000, min(1000, steer))
    speed = max(-1000, min(1000, speed))
    
    print(f"Отправка команды: left={left_speed}, right={right_speed}")
    print(f"Преобразовано в: steer={steer}, speed={speed}")
    
    # Создаем пакет данных (первые 4 байта)
    data = struct.pack('<hh', steer, speed)  # little-endian, 2 int16
    
    # Вычисляем CRC32
    crc32 = calculate_crc32(data)
    
    # Создаем полный пакет
    packet = data + struct.pack('<I', crc32)  # добавляем CRC32
    
    print(f"Пакет данных: {packet.hex()}")
    print(f"CRC32: {crc32:08x}")
    
    # Отправляем
    ser.write(packet)
    ser.flush()
    
    return True


def test_connection(port, baudrate=115200):
    """Тестирование соединения с hoverboard"""
    print(f"Тестирование соединения с {port} на скорости {baudrate}")
    
    try:
        # Открываем соединение
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=1.0,
            write_timeout=1.0
        )
        
        print("✓ UART соединение установлено")
        
        # Тест 1: Остановка моторов
        print("\n=== Тест 1: Остановка моторов ===")
        send_motor_command(ser, 0, 0)
        time.sleep(0.5)
        
        # Тест 2: Медленное движение вперед
        print("\n=== Тест 2: Медленное движение вперед ===")
        send_motor_command(ser, 100, 100)
        time.sleep(2.0)
        
        # Тест 3: Поворот влево
        print("\n=== Тест 3: Поворот влево ===")
        send_motor_command(ser, -200, 200)
        time.sleep(2.0)
        
        # Тест 4: Поворот вправо
        print("\n=== Тест 4: Поворот вправо ===")
        send_motor_command(ser, 200, -200)
        time.sleep(2.0)
        
        # Тест 5: Движение назад
        print("\n=== Тест 5: Движение назад ===")
        send_motor_command(ser, -100, -100)
        time.sleep(2.0)
        
        # Тест 6: Остановка
        print("\n=== Тест 6: Остановка ===")
        send_motor_command(ser, 0, 0)
        time.sleep(0.5)
        
        print("\n✓ Все тесты завершены успешно!")
        
        ser.close()
        return True
        
    except serial.SerialException as e:
        print(f"✗ Ошибка UART: {e}")
        return False
    except Exception as e:
        print(f"✗ Неожиданная ошибка: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(description='Тест связи с hoverboard')
    parser.add_argument('--port', default='/dev/serial0', 
                       help='UART порт (по умолчанию: /dev/serial0)')
    parser.add_argument('--baudrate', type=int, default=115200,
                       help='Скорость UART (по умолчанию: 115200)')
    parser.add_argument('--interactive', action='store_true',
                       help='Интерактивный режим для ручного тестирования')
    
    args = parser.parse_args()
    
    if args.interactive:
        print("Интерактивный режим тестирования")
        print("Команды: w(вперед), s(назад), a(влево), d(вправо), q(выход)")
        
        try:
            ser = serial.Serial(
                port=args.port,
                baudrate=args.baudrate,
                timeout=1.0,
                write_timeout=1.0
            )
            
            while True:
                cmd = input("Введите команду: ").lower().strip()
                
                if cmd == 'q':
                    break
                elif cmd == 'w':
                    send_motor_command(ser, 200, 200)  # вперед
                elif cmd == 's':
                    send_motor_command(ser, -200, -200)  # назад
                elif cmd == 'a':
                    send_motor_command(ser, -200, 200)  # влево
                elif cmd == 'd':
                    send_motor_command(ser, 200, -200)  # вправо
                elif cmd == ' ':
                    send_motor_command(ser, 0, 0)  # остановка
                else:
                    print("Неизвестная команда")
                
                time.sleep(0.1)
            
            ser.close()
            
        except KeyboardInterrupt:
            print("\nПрервано пользователем")
        except Exception as e:
            print(f"Ошибка: {e}")
    else:
        # Автоматический тест
        success = test_connection(args.port, args.baudrate)
        sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
