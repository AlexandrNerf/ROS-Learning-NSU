import rclpy
import numpy as np
import time

from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nerfman_name.action import MessageTurtleCommands
from math import sqrt, pi, fabs


class TurtleActionServer(Node):

    def __init__(self):
        super().__init__('turtle_action_server')
        self.current_pose = Pose()
        self._action_server = ActionServer(
            self,
            MessageTurtleCommands,
            'turtle_act',
            self.execute_callback)
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.position_callback,
            10
        )
        self.dist = 0.0

    def position_callback(self, msg):
        self.current_pose = msg
    
    def execute_callback(self, goal_handle):

        self.get_logger().info('Received goal request.')

        command = goal_handle.request.command
        s = goal_handle.request.s
        angle = goal_handle.request.angle

        feedback_msg = MessageTurtleCommands.Feedback()
        feedback_msg.odom = 0.0

        odom = Twist()

        rate = self.create_rate(60)  # 10 Hz

        if command == 'forward':
            self.get_logger().info(f'Moving forward {s} meters.')
            odom.linear.x = 1.0  # Set linear velocity

            initial_pose = self.current_pose

            while rclpy.ok() and feedback_msg.odom < s:
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Goal canceled.')
                    goal_handle.canceled()
                    # Stop the turtle
                    odom.linear.x = 0.0
                    self.publisher.publish(odom)
                    return MessageTurtleCommands.Result(result=False)

                # Calculate distance traveled
                dx = self.current_pose.x - initial_pose.x
                dy = self.current_pose.y - initial_pose.y
                current_distance = sqrt(dx * dx + dy * dy)
                feedback_msg.odom = current_distance

                self.publisher.publish(odom)

                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info(f'Feedback: current distance: {current_distance:.2f} meters')

                rate.sleep()

        elif command in ['turn_left', 'turn_right']:
            direction = 1.0 if command == 'turn_left' else -1.0
            angle_rad = angle * pi / 180  # Convert degrees to radians
            self.get_logger().info(
                f'Turning {"left" if direction > 0 else "right"} by {angle} degrees.')

            odom.angular.z = direction * 0.5  # Set angular velocity (rad/s)
            initial_theta = self.current_pose.theta

            while rclpy.ok() and feedback_msg.odom < fabs(angle_rad):
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Goal canceled.')
                    goal_handle.canceled()
                    # Stop the turtle
                    odom.angular.z = 0.0
                    self.publisher.publish(odom)
                    return MessageTurtleCommands.Result(result=False)

                # Calculate angle turned
                current_theta = self.current_pose.theta
                delta_theta = self.angle_difference(current_theta, initial_theta)

                feedback_msg.odom = fabs(delta_theta)

                self.publisher.publish(odom)

                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info(f'Feedback: current angle turned: {feedback_msg.odom * 180/pi:.2f} degrees')

                rate.sleep()

        else:
            self.get_logger().warn(f'Unknown command: {command}')
            goal_handle.abort()
            result = MessageTurtleCommands.Result()
            result.result = False
            return result

        # Stop the turtle after movement
        odom.linear.x = 0.0
        odom.angular.z = 0.0
        self.publisher.publish(odom)

        # Mark the goal as succeeded
        goal_handle.succeed()

        result = MessageTurtleCommands.Result()
        result.result = True
        self.get_logger().info('Goal succeeded.')
        return result

    def angle_difference(self, current, initial):
        """Compute the smallest difference between two angles."""
        delta = current - initial
        while delta > pi:
            delta -= 2 * pi
        while delta < -pi:
            delta += 2 * pi
        return delta


def main(args=None):
    rclpy.init(args=args)

    action_server = TurtleActionServer()
    # Use MultiThreadedExecutor to allow concurrent callback processing
    executor = MultiThreadedExecutor()
    executor.add_node(action_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        action_server.destroy_node()
        rclpy.shutdown()

    rclpy.spin(action_server)


if __name__ == '__main__':
    main()
