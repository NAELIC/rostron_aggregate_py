import rclpy
from rclpy.node import Node
from .utils.Ball import Ball
from .utils.Robot import Robot

from rostron_interfaces.msg import DetectionFrame, Robots as RobotsMsg, Ball as BallMsg, Robot as RobotMsg


class MinimalFilter(Node):
    def __init__(self):
        super().__init__('minimal_filter')
        self.ball = Ball()
        self.y_robots = [Robot() for c in range(16)]
        self.b_robots = [Robot() for c in range(16)]
        self.pub_allies_ = self.create_publisher(RobotsMsg, 'allies', 10)
        self.pub_opponents_ = self.create_publisher(RobotsMsg, 'opponents', 10)
        self.pub_ball_ = self.create_publisher(BallMsg, 'ball', 10)

        self.pubs_allies_ = [self.create_publisher(RobotMsg, 'allies/r_%d' % c, 10) for c in range(16)]
        self.pubs_opponents_ = [self.create_publisher(RobotMsg, 'opponents/r_%d' % c, 10) for c in range(16)]

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
        msg = BallMsg()
        msg.position = self.ball.position

        self.pub_ball_.publish(msg)
        msg_allies = RobotsMsg()
        for id, r in enumerate(self.y_robots):
            msg_allies.robots[id].pose = r.pose
            msg_allies.robots[id].id = id
            msg_allies.robots[id].active = r.active

            msg_r = RobotMsg()
            msg_r.id = id
            msg_r.pose = r.pose
            msg_r.active = r.active

            self.pubs_allies_[id].publish(msg_r)
        self.pub_allies_.publish(msg_allies)

        msg_opponents = RobotsMsg()
        for id, r in enumerate(self.b_robots):
            msg_opponents.robots[id].pose = r.pose
            msg_opponents.robots[id].id = id
            msg_opponents.robots[id].active = r.active
            
            msg_r = RobotMsg()
            msg_r.id = id
            msg_r.pose = r.pose
            msg_r.active = r.active

            self.pubs_opponents_[id].publish(msg_r)

        self.pub_opponents_.publish(msg_opponents)

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
