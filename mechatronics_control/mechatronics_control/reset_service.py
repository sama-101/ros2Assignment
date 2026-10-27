import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_srvs.srv import Trigger, Empty
from turtlesim.srv import SetPen, TeleportAbsolute


class TurtleResetService(Node):

    def __init__(self):
        super().__init__('reset_service')

        self.client_group = ReentrantCallbackGroup()

        # Our own reset service
        self.reset_server = self.create_service(
            Trigger,
            '/reset_turtle',
            self.reset_callback
        )

        # Existing turtlesim services
        self.pen_client = self.create_client(
            SetPen,
            '/turtle1/set_pen',
            callback_group=self.client_group
        )

        self.clear_client = self.create_client(
            Empty,
            '/clear',
            callback_group=self.client_group
        )

        self.teleport_client = self.create_client(
            TeleportAbsolute,
            '/turtle1/teleport_absolute',
            callback_group=self.client_group
        )

        self.get_logger().info('Reset Service Started')

    def call_service(self, client, request, name):
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f'{name} service is unavailable')

        future = client.call_async(request)
        finished = threading.Event()

        future.add_done_callback(
            lambda _: finished.set()
        )

        if not finished.wait(timeout=5.0):
            raise RuntimeError(f'Timeout waiting for {name}')

        result = future.result()

        if result is None:
            raise RuntimeError(f'{name} returned no response')

        return result

    def set_pen_request(self, off):
        request = SetPen.Request()

        request.r = 63
        request.g = 0
        request.b = 0
        request.width = 3
        request.off = off

        return request

    def reset_callback(self, request, response):
        pen_disabled = False

        try:
            # 1. Disable the pen
            self.call_service(
                self.pen_client,
                self.set_pen_request(1),
                '/turtle1/set_pen'
            )
            pen_disabled = True

            # 2. Clear the screen
            self.call_service(
                self.clear_client,
                Empty.Request(),
                '/clear'
            )

            # 3. Return the turtle to its starting pose
            teleport = TeleportAbsolute.Request()
            teleport.x = 5.544445
            teleport.y = 5.544445
            teleport.theta = 0.0

            self.call_service(
                self.teleport_client,
                teleport,
                '/turtle1/teleport_absolute'
            )

            response.success = True
            response.message = (
                'Screen cleared and turtle returned to start'
            )

        except Exception as error:
            response.success = False
            response.message = str(error)
            self.get_logger().error(str(error))

        finally:
            # Re-enable the pen even if another step fails
            if pen_disabled:
                try:
                    self.call_service(
                        self.pen_client,
                        self.set_pen_request(0),
                        '/turtle1/set_pen'
                    )
                except Exception as error:
                    self.get_logger().error(
                        f'Could not re-enable pen: {error}'
                    )

        return response


def main(args=None):
    rclpy.init(args=args)

    node = TurtleResetService()

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