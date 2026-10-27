import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from mechatronics_interfaces.action import DrawSquare, GoToPose
from std_srvs.srv import Trigger


class SquareActionServer(Node):

    def __init__(self):
        super().__init__('square_action_server')

        self.client_group = ReentrantCallbackGroup()

        # Client for the reset service
        self.reset_client = self.create_client(
            Trigger,
            '/reset_turtle',
            callback_group=self.client_group
        )

        # Client for the GoToPose action server
        self.go_to_pose_client = ActionClient(
            self,
            GoToPose,
            '/go_to_pose',
            callback_group=self.client_group
        )

        # Our square action server
        self.square_action_server = ActionServer(
            self,
            DrawSquare,
            '/draw_square',
            self.execute_callback
        )

        self.get_logger().info('Square Action Server Started')

    def wait_for_future(self, future, timeout_sec, description):
        event = threading.Event()

        future.add_done_callback(lambda _: event.set())

        if not event.wait(timeout_sec):
            raise TimeoutError(
                f'Timeout while waiting for {description}'
            )

        return future.result()

    def execute_callback(self, goal_handle):
        result = DrawSquare.Result()

        side_length = goal_handle.request.side_length

        # Validate the square size
        if (
            not math.isfinite(side_length)
            or side_length <= 0.0
            or side_length > 4.0
        ):
            goal_handle.abort()
            result.success = False
            result.message = (
                'Side length must be greater than 0 '
                'and no greater than 4.0'
            )
            return result

        try:
            # Wait for the reset service
            if not self.reset_client.wait_for_service(
                timeout_sec=5.0
            ):
                raise RuntimeError(
                    '/reset_turtle service is unavailable'
                )

            # Wait for the GoToPose action server
            if not self.go_to_pose_client.wait_for_server(
                timeout_sec=5.0
            ):
                raise RuntimeError(
                    '/go_to_pose action server is unavailable'
                )

            # Reset the screen and turtle before drawing
            reset_future = self.reset_client.call_async(
                Trigger.Request()
            )

            reset_response = self.wait_for_future(
                reset_future,
                10.0,
                'reset service'
            )

            if not reset_response.success:
                raise RuntimeError(reset_response.message)

            self.get_logger().info(
                'Reset completed. Starting the square.'
            )

            # Turtlesim's original starting position
            start = 5.544445

            # Four corners, including the return to the start
            targets = [
                (start + side_length, start, 0.0),
                (
                    start + side_length,
                    start + side_length,
                    math.pi / 2
                ),
                (start, start + side_length, math.pi),
                (start, start, -math.pi / 2)
            ]

            # Send exactly four GoToPose goals, one at a time
            for side_number, (x, y, yaw) in enumerate(
                targets,
                start=1
            ):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Square drawing canceled'
                    return result

                feedback = DrawSquare.Feedback()
                feedback.current_side = side_number
                goal_handle.publish_feedback(feedback)

                self.get_logger().info(
                    f'Drawing side {side_number}/4'
                )

                sub_goal = GoToPose.Goal()
                sub_goal.target_pose.x = x
                sub_goal.target_pose.y = y
                sub_goal.target_pose.yaw = yaw

                # Send the goal
                send_future = (
                    self.go_to_pose_client.send_goal_async(
                        sub_goal
                    )
                )

                sub_goal_handle = self.wait_for_future(
                    send_future,
                    10.0,
                    f'GoToPose goal {side_number} response'
                )

                if not sub_goal_handle.accepted:
                    raise RuntimeError(
                        f'GoToPose rejected side {side_number}'
                    )

                # Wait until this side is completed
                result_future = (
                    sub_goal_handle.get_result_async()
                )

                sub_result = self.wait_for_future(
                    result_future,
                    90.0,
                    f'side {side_number} movement'
                )

                if not sub_result.result.success:
                    raise RuntimeError(
                        f'GoToPose failed on side {side_number}: '
                        f'{sub_result.result.message}'
                    )

            goal_handle.succeed()

            result.success = True
            result.message = 'Square drawn successfully'

            self.get_logger().info(
                'Square completed successfully'
            )

        except Exception as error:
            goal_handle.abort()

            result.success = False
            result.message = str(error)

            self.get_logger().error(str(error))

        return result


def main(args=None):
    rclpy.init(args=args)

    node = SquareActionServer()

    executor = MultiThreadedExecutor(num_threads=4)
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