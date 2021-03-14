import rclpy
from rclpy.node import Node
from .utils.Ball import Ball
from .utils.Robot import Robot
from .utils.StatePublisher import StatePublisher

from rostron_interfaces.msg import DetectionFrame, Robots as RobotsMsg, Ball as BallMsg, Robot as RobotMsg


class MinimalFilter(Node):
    def __init__(self):
        super().__init__('minimal_filter')
        self.ball = Ball()
        self.y_robots = [Robot() for c in range(16)]
        self.b_robots = [Robot() for c in range(16)]
        self.state_pub_ = StatePublisher(self)

        self.subscription = self.create_subscription(
            DetectionFrame,
            'vision',
            self.vision_callback,
            10)
        self.timer = self.create_timer(0.16, self.timer_callback)

    def vision_callback(self, msg: DetectionFrame):
        # balls
        for ball in msg.balls:
            self.ball.update(ball.position)

        for r in msg.yellow:
            if r.confidence > 0.5:
                self.y_robots[r.id].update(r.pose)

        for r in msg.blue:
            if r.confidence > 0.5:
                self.b_robots[r.id].update(r.pose)

    def timer_callback(self):
        self.state_pub_.publish_ball(self.ball)
        self.state_pub_.publish_allies(self.y_robots)
        self.state_pub_.publish_opponents(self.b_robots)

    def debug():
        self.get_logger().info(self.ball.debug())

        for id, r in enumerate(self.y_robots):
            if r.active:
                self.get_logger().info(
                    '[yellow] "%d" : "%s"' % (id, r.debug()))

        for id, r in enumerate(self.b_robots):
            if r.active:
                self.get_logger().info('[blue] "%d" : "%s"' % (id, r.debug()))


def main(args=None):
    rclpy.init(args=args)

    minimal_filter = MinimalFilter()
    rclpy.spin(minimal_filter)

    minimal_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
