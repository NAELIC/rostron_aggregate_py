from rostron_interfaces.msg import Robots as RobotsMsg, Ball as BallMsg, Robot as RobotMsg

class StatePublisher():

    def __init__(self, node):
        self.node_ = node
        self.pub_allies_ = self.node_.create_publisher(RobotsMsg, 'allies', 10)
        self.pub_opponents_ = self.node_.create_publisher(RobotsMsg, 'opponents', 10)
        self.pub_ball_ = self.node_.create_publisher(BallMsg, 'ball', 10)

        self.pub_allies_arr_ = [self.node_.create_publisher(
            RobotMsg, 'allies/r_%d' % c, 10) for c in range(16)]
        self.pub_opponents_arr_ = [self.node_.create_publisher(
            RobotMsg, 'opponents/r_%d' % c, 10) for c in range(16)]

    def publish_ball(self, ball):
        self.pub_ball_.publish(ball)

    def publish_allies(self, allies):
        for id, r in enumerate(allies.robots):
            self.pub_allies_arr_[id].publish(r)
        self.pub_allies_.publish(allies)

    def publish_opponents(self, opponents):
        for id, r in enumerate(opponents.robots):
            self.pub_opponents_arr_[id].publish(r)
        self.pub_opponents_.publish(opponents)
