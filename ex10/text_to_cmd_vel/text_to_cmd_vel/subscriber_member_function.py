# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

class MinimalSubscriber(Node):

    def __init__(self):
        # Инициализируем узел
        super().__init__('text_to_cmd_vel')

        # Подписываемся на топик "cmd_text" с типом сообщения String
        self.subscription = self.create_subscription(
            String,
            'cmd_text',
            self.listener_callback,
            10
        )
        self.subscription  # предотвращаем удаление подписки сборщиком мусора

        # Паблишер для отправки команд в топик /turtle1/cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtlesim1/turtle1/cmd_vel', 10)

    def listener_callback(self, msg):
        # Создаем объект Twist для отправки команд скорости
        twist = Twist()

        # Обработка команды
        if msg.data == "turn_right":
            twist.angular.z = -1.5  # Вращение по часовой стрелке (1.5 рад/с)
            self.get_logger().info('Command: turn_right')
        elif msg.data == "turn_left":
            twist.angular.z = 1.5  # Вращение против часовой стрелки (1.5 рад/с)
            self.get_logger().info('Command: turn_left')
        elif msg.data == "move_forward":
            twist.linear.x = 1.0  # Движение вперед (1 м/с)
            self.get_logger().info('Command: move_forward')
        elif msg.data == "move_backward":
            twist.linear.x = -1.0  # Движение назад (-1 м/с)
            self.get_logger().info('Command: move_backward')
        else:
            self.get_logger().warn(f"Неизвестная команда: {msg.data}")
            return

        # Публикуем команду скорости
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
