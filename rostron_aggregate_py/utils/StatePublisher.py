from rostron_interfaces.msg import Robots as RobotsMsg, Ball as BallMsg, Robot as RobotMsg

class StatePublisher():

    def __init__(self, node, publish_robots):
        self.node_ = node

        self.publish_robots_ = publish_robots

        self.pub_ball_ = self.node_.create_publisher(BallMsg, 'ball', 10)

        self.pub_allies_ = self.node_.create_publisher(RobotsMsg, 'allies', 10)
        self.pub_allies_arr_ = [self.node_.create_publisher(
            RobotMsg, 'allies/r_%d' % id, 10) for id in self.publish_robots_]
        
        self.pub_opponents_ = self.node_.create_publisher(RobotsMsg, 'opponents', 10)

    def publish_ball(self, ball):
        self.pub_ball_.publish(ball)

    def publish_allies(self, allies):
        for id in self.publish_robots_:
            self.pub_allies_arr_[id].publish(allies.robots[id])
        self.pub_allies_.publish(allies)

    def publish_opponents(self, opponents):
        self.pub_opponents_.publish(opponents)
