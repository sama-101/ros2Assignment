import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from mechatronics_interfaces.srv import SetSpeed


class SpeedController(Node):

    def __init__(self):
        super().__init__('speed_controller')

        # Declare the speed parameters with default values
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 2.0)

        # Validate parameter changes
        self.add_on_set_parameters_callback(
            self.validate_parameters
        )
        self.speed_service = self.create_service(
            SetSpeed,
            '/set_speed',
            self.set_speed_callback
        )
        self.get_logger().info(
            'Speed Controller Started'
        )

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
            rclpy.parameter.Parameter(
                'max_linear_speed',
                value=linear
            ),
            rclpy.parameter.Parameter(
                'max_angular_speed',
                value=angular
            )
        ])

        response.success = result.successful

        if result.successful:
            response.message = 'Speed parameters updated'
        else:
            response.message = result.reason

        return response
def main(args=None):
    rclpy.init(args=args)

    node = SpeedController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()