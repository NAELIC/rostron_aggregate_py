from geometry_msgs.msg import Pose


class Robot:
    def __init__(self):
        self.pose = Pose()
        self.active = False

    # Help with position put geometry msgs

    def update(self, pose):
        self.pose.position.x = pose.position.x
        self.pose.position.y = pose.position.y
        self.pose.orientation.z = pose.orientation.z
        self.active = True

    def debug(self) -> str:
        return '(x : "%f",y "%f", theta "%f")' % (self.pose.position.x, self.pose.position.y, self.pose.orientation.z)
