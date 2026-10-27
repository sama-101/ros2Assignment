import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from rclpy.action import ActionServer
from mechatronics_interfaces.action import GoToPose

import math
from rcl_interfaces.msg import SetParametersResult

import time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from mechatronics_interfaces.srv import SetSpeed
from rclpy.parameter import Parameter

class GoToPoseServer(Node):

    def __init__(self):
        super().__init__('go_to_pose_server')

        self.current_pose = None
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 2.0)

        self.add_on_set_parameters_callback(
            self.validate_parameters
        )

        self.pose_callback_group = ReentrantCallbackGroup()

        # Read the turtle's current pose
        self.pose_subscriber = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10,
            callback_group=self.pose_callback_group
        )
        # Publisher for turtle velocity commands
        self.velocity_publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )
        self.action_server = ActionServer(
        self,
        GoToPose,
        '/go_to_pose',
        self.execute_callback
    )
        self.speed_service = self.create_service(
        SetSpeed,
        '/set_speed',
        self.set_speed_callback
    )

        self.get_logger().info('GoToPose node started')
    
    def validate_parameters(self, parameters):
        for param in parameters:
            if param.name in (
                'max_linear_speed',
                'max_angular_speed'
            ):
                if (
                    not math.isfinite(param.value)
                    or param.value <= 0.0
                ):
                    return SetParametersResult(
                        successful=False,
                        reason='Speed must be greater than zero'
                    )

        return SetParametersResult(successful=True)

    def pose_callback(self, msg):
        if self.current_pose is None:
            self.get_logger().info(
                f'Current pose: x={msg.x:.2f}, y={msg.y:.2f}'
            )

        self.current_pose = msg

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))


    def limit_speed(self, value, max_speed):
        return max(-max_speed, min(value, max_speed))


    def execute_callback(self, goal_handle):
        goal = goal_handle.request.target_pose

        self.get_logger().info(
            f'Goal received: x={goal.x}, y={goal.y}, yaw={goal.yaw}'
        )

        result = GoToPose.Result()
        feedback = GoToPose.Feedback()

        position_tolerance = 0.03
        angle_tolerance = 0.05

        # Wait until we receive the turtle's pose
        while self.current_pose is None:
            time.sleep(0.1)

        while True:

            # Stop if the goal is canceled
            if goal_handle.is_cancel_requested:
                self.velocity_publisher.publish(Twist())
                goal_handle.canceled()
                result.success = False
                result.message = 'Goal canceled'
                return result

            pose = self.current_pose

            # Calculate the distance to the target
            dx = goal.x - pose.x
            dy = goal.y - pose.y
            distance = math.hypot(dx, dy)

            # Publish feedback
            feedback.distance_remaining = distance
            goal_handle.publish_feedback(feedback)

            # Read speed limits every iteration
            max_linear = self.get_parameter(
                'max_linear_speed'
            ).value

            max_angular = self.get_parameter(
                'max_angular_speed'
            ).value

            cmd = Twist()

            if distance > position_tolerance:

                # Direction from the turtle to the target
                target_angle = math.atan2(dy, dx)

                angle_error = self.normalize_angle(
                    target_angle - pose.theta
                )

                # Rotate first if the target is far from our direction
                if abs(angle_error) > 0.05:
                    cmd.angular.z = self.limit_speed(
                        2.0 * angle_error,
                        max_angular
                    )

                else:
                    # Move forward and correct the direction
                    cmd.linear.x = min(
                        max_linear,
                        distance*.8
                    )

                    cmd.angular.z = self.limit_speed(
                        2.0 * angle_error,
                        max_angular
                    )

            else:
                # Position reached: adjust the final orientation
                angle_error = self.normalize_angle(
                    goal.yaw - pose.theta
                )

                if abs(angle_error) <= angle_tolerance:
                    break

                cmd.angular.z = self.limit_speed(
                    2.0 * angle_error,
                    max_angular
                )

            self.velocity_publisher.publish(cmd)

            time.sleep(0.02)

        # Stop the turtle when the goal is reached
        self.velocity_publisher.publish(Twist())

        goal_handle.succeed()

        result.success = True
        result.message = 'Target pose reached'

        self.get_logger().info('Target pose reached')

        return result

    def set_speed_callback(self, request, response):
        linear = request.max_linear_speed
        angular = request.max_angular_speed

        if (
            not math.isfinite(linear)
            or not math.isfinite(angular)
            or linear <= 0.0
            or angular <= 0.0
        ):
            response.success = False
            response.message = 'Speeds must be greater than zero'
            return response

        result = self.set_parameters_atomically([
            Parameter('max_linear_speed', value=linear),
            Parameter('max_angular_speed', value=angular)
        ])

        response.success = result.successful

        if result.successful:
            response.message = 'Speed parameters updated'
        else:
            response.message = result.reason

        return response
def main(args=None):
    rclpy.init(args=args)

    node = GoToPoseServer()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()