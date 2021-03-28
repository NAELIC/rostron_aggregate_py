import rclpy
from rclpy.node import Node
from .utils.StatePublisher import StatePublisher

from rostron_interfaces.msg import DetectionFrame, Robot, Ball, Robots


class MinimalFilter(Node):
    is_yellow_ = True
    ball = Ball()
    y_robots = Robots()
    b_robots = Robots()

    def __init__(self):
        super().__init__('minimal_aggregate')

        # Parameters
        self.declare_parameter('yellow', True)
        self.declare_parameter('publish_robots', [0])

        for id in self.get_parameter('publish_robots').get_parameter_value().integer_array_value:
            self.get_logger().info('%d' % id)

        # Initialisation
        self.is_yellow_ = self.get_parameter(
            'yellow').get_parameter_value().bool_value
        self.state_pub_ = StatePublisher(self, self.get_parameter(
            'publish_robots').get_parameter_value().integer_array_value)

        for id, r in enumerate(self.y_robots.robots):
            self.y_robots.robots[id].id = id
        for id, r in enumerate(self.b_robots.robots):
            self.b_robots.robots[id].id = id

        # Create subscription and timers
        self.subscription = self.create_subscription(
            DetectionFrame,
            'vision',
            self.vision_callback,
            10)
        self.timer = self.create_timer(0.16, self.timer_callback)

    def vision_callback(self, msg: DetectionFrame):
        for ball in msg.balls:
            self.ball.position = ball.position

        for r in msg.yellow:
            if r.confidence > 0.5:
                self.y_robots.robots[r.id].pose = r.pose
                self.y_robots.robots[r.id].active = True

        for r in msg.blue:
            if r.confidence > 0.5:
                self.b_robots.robots[r.id].pose = r.pose
                self.b_robots.robots[r.id].active = True

    def timer_callback(self):
        self.state_pub_.publish_ball(self.ball)
        self.state_pub_.publish_allies(
            self.y_robots if self.is_yellow_ else self.b_robots)
        self.state_pub_.publish_opponents(
            self.b_robots if self.is_yellow_ else self.y_robots)

    def debug():
        self.get_logger().debug(self.ball.debug())

        for id, r in enumerate(self.y_robots.robots):
            if r.active:
                self.get_logger().debug(
                    '[yellow] "%d" : "%s"' % (id, r.debug()))

        for id, r in enumerate(self.b_robots.robots):
            if r.active:
                self.get_logger().debug('[blue] "%d" : "%s"' % (id, r.debug()))


def main(args=None):
    rclpy.init(args=args)

    minimal_filter = MinimalFilter()
    rclpy.spin(minimal_filter)

    minimal_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
