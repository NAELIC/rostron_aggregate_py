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
        msg = BallMsg()
        msg.position = ball.position
        self.pub_ball_.publish(msg)

    def publish_allies(self, allies):
        msg = RobotsMsg()
        for id, r in enumerate(allies):
            msg_r = self.create_msg_robot(r, id)
            msg.robots[id] = msg_r
            self.pub_allies_arr_[id].publish(msg_r)
        self.pub_allies_.publish(msg)

    def publish_opponents(self, opponents):
        msg = RobotsMsg()
        for id, r in enumerate(opponents):
            msg_r = self.create_msg_robot(r, id)
            msg.robots[id] = msg_r
            self.pub_opponents_arr_[id].publish(msg_r)
        self.pub_opponents_.publish(msg)

    def create_msg_robot(self, r, id):
        msg_r = RobotMsg()
        msg_r.id = id
        msg_r.pose = r.pose
        msg_r.active = r.active
        return msg_r
