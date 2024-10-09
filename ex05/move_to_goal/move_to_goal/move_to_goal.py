from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rclpy.node import Node
import rclpy
import math
import time
import argparse


class MoveToGoal(Node):

    def __init__(self, target_x, target_y, target_theta):
        super().__init__('move_to_goal')
        self.pub_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub_pose = self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10)

        self.target_x = target_x
        self.target_y = target_y
        self.target_theta = (target_theta + math.pi) % (2 * math.pi) - math.pi
        self.counter = 9
        self.pose = Pose() 
        self.timer = self.create_timer(0.1, self.move2goal)
        self.distance_tolerance = 0.1
        self.angle_tolerance = 0.1
        
    def update_pose(self, data):
        self.pose = data

    def move2goal(self):

        msg = Twist()
        distance = math.sqrt((self.target_x - self.pose.x)**2 + (self.target_y - self.pose.y) **2)
        angle = math.atan2(self.target_y - self.pose.y, self.target_x - self.pose.x)
        linear_vel = 1.0 * distance
        angular_vel = 4.0 * (angle - self.pose.theta)

        msg.linear.x = linear_vel
        msg.angular.z = angular_vel

        self.pub_vel.publish(msg)

        if distance < self.distance_tolerance and abs(angle) > self.angle_tolerance:
            msg.angular.z  = self.target_theta
            self.pub_vel.publish(msg)

            count = 0
            while count < self.counter:
                self.pub_vel.publish(msg)
                time.sleep(0.1)
                count += 1  
            
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            
            self.get_logger().info("Goal Reached!! ")
            self.timer.cancel()
            self.pub_vel.publish(msg)
            quit()

def main(args=None):
    rclpy.init(args=args)
    
    parser = argparse.ArgumentParser(description="Передвижение черепахи к цели")
    parser.add_argument('--x', type=float, default=5.5, help='X координата цели (default: 5.5)')
    parser.add_argument('--y', type=float, default=5.5, help='Y координата цели (default: 5.5)')
    parser.add_argument('--theta', type=float, default=0.0, help='Финальный угол цели (default: 0.0)')
    args = parser.parse_args()

    move_to_goal = MoveToGoal(args.x, args.y, args.theta)
    rclpy.spin(move_to_goal)
    move_to_goal.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 