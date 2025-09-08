import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Vector3, Pose, Twist
from sensor_msgs.msg import Imu
from transforms3d.euler import euler2quat  # Альтернатива tf_transformations
from math import sin, cos, atan2, sqrt, degrees, radians
import smbus2
import time

# ==== Настройки I2C ====
I2C_BUS = 1
MPU_ADDR = 0x68

# ==== Инициализация шины ====
bus = smbus2.SMBus(I2C_BUS)

# ==== Выход из sleep режима ====
bus.write_byte_data(MPU_ADDR, 0x6B, 0x00)

# ==== Установка диапазона акселерометра и гироскопа ====
# ±2g, ±250°/s
bus.write_byte_data(MPU_ADDR, 0x1C, 0x00)  # Акселерометр
bus.write_byte_data(MPU_ADDR, 0x1B, 0x00)  # Гироскоп

# ==== Функция чтения 16-битного значения ====
def read_word(reg):
    high = bus.read_byte_data(MPU_ADDR, reg)
    low = bus.read_byte_data(MPU_ADDR, reg + 1)
    value = (high << 8) | low
    return value if value < 32768 else value - 65536

# ==== Калибровка датчика ====
def calibrate_sensors():
    print("Калибровка... Держите датчик неподвижно!")
    gx_sum, gy_sum, gz_sum = 0, 0, 0
    ax_sum, ay_sum, az_sum = 0, 0, 0

    for _ in range(500):
        gx, gy, gz = read_word(0x43), read_word(0x45), read_word(0x47)
        ax, ay, az = read_word(0x3B), read_word(0x3D), read_word(0x3F)

        gx_sum += gx
        gy_sum += gy
        gz_sum += gz

        ax_sum += ax
        ay_sum += ay
        az_sum += az

        time.sleep(0.002)

    gyro_calib = [
        gx_sum / 500 / 131.0,
        gy_sum / 500 / 131.0,
        gz_sum / 500 / 131.0
    ]

    accel_calib = [
        ax_sum / 500 / 16384.0,
        ay_sum / 500 / 16384.0,
        az_sum / 500 / 16384.0
    ]

    print(f"Калибровка завершена. Смещения:")
    print(f"Gyro offset: {gyro_calib}")
    print(f"Accel offset: {accel_calib}")

    return gyro_calib, accel_calib

# ==== ROS 2 Узел ====
class IMUToOdom(Node):
    def __init__(self):
        super().__init__('imu_to_odom')

        # Параметры
        self.gyro_calib, self.accel_calib = calibrate_sensors()
        self.angle_x, self.angle_y = 0.0, 0.0
        self.last_time = self.get_clock().now()

        # Переменные для расчета одометрии
        self.x, self.y, self.theta = 0.0, 0.0, 0.0

        # Паблишеры
        #self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)

        # Таймер
        timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Чтение данных с IMU
        ax = (read_word(0x3B) / 16384.0) - self.accel_calib[0]
        ay = (read_word(0x3D) / 16384.0) - self.accel_calib[1]
        az = (read_word(0x3F) / 16384.0) - self.accel_calib[2]

        gx = (read_word(0x43) / 131.0) - self.gyro_calib[0]
        gy = (read_word(0x45) / 131.0) - self.gyro_calib[1]
        gz = (read_word(0x47) / 131.0) - self.gyro_calib[2]

        # Расчет углов
        roll_accel = atan2(ay, sqrt(ax**2 + az**2))
        pitch_accel = atan2(-ax, sqrt(ay**2 + az**2))

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        self.angle_x = 0.98 * (self.angle_x + gx * dt) + 0.02 * roll_accel
        self.angle_y = 0.98 * (self.angle_y + gy * dt) + 0.02 * pitch_accel

        # Публикация IMU данных
        imu_msg = Imu()
        imu_msg.header.stamp = current_time.to_msg()
        imu_msg.header.frame_id = 'base_link'

        # Линейное ускорение
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az

        # Угловая скорость
        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz

        # Ориентация (кватернион)
        q = euler2quat(self.angle_x, self.angle_y, 0.0)
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]

        self.imu_pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMUToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
