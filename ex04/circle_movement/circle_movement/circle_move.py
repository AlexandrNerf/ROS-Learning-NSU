#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from math import sin, cos, pi

class CircleBot(Node):
    def __init__(self):
        super().__init__('circle_bot')

        # Публикация в топик /cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/robot/cmd_vel', 10)

        # Инициализация TransformBroadcaster для публикации tf
        self.tf_broadcaster = TransformBroadcaster(self)

        # Таймер для периодической публикации (10 Гц)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Параметры движения
        self.linear_speed = 0.2  # линейная скорость (м/с)
        self.angular_speed = 0.2  # угловая скорость (рад/с)
        self.radius = self.linear_speed / self.angular_speed  # радиус окружности

        # Время начала движения
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]

    def timer_callback(self):
        """Функция, вызываемая таймером для публикации команд и tf."""
        current_time = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time

        # Публикация команды на движение
        self.publish_cmd_vel()

        # Публикация TF трансформаций
        self.publish_tf(current_time)

    def publish_cmd_vel(self):
        """Публикация команды скорости для движения по кругу."""
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        self.cmd_pub.publish(twist)

    def publish_tf(self, t):
        """Публикация TF для робота и колес."""
        # Позиция и ориентация робота
        x = self.radius * cos(t * self.angular_speed)
        y = self.radius * sin(t * self.angular_speed)
        quaternion = quaternion_from_euler(0, 0, t * self.angular_speed)

        # Публикация TF для базы робота
        self.send_transform(x, y, 0, quaternion, "base_link", "world")

        # Публикация TF для колес
        wheel_offset = 0.1  # смещение колес от центра
        for side, sign in [("left_wheel", 1), ("right_wheel", -1)]:
            self.send_transform(
                0, sign * wheel_offset, 0, 
                quaternion_from_euler(0, 0, 0), 
                side, "base_link"
            )

    def send_transform(self, x, y, z, quat, child_frame, parent_frame):
        """Отправка трансформации через TransformBroadcaster."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    bot = CircleBot()
    rclpy.spin(bot)

    # Очистка ресурсов при завершении работы
    bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

