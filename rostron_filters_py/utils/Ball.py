from geometry_msgs.msg import Point

class Ball:
    def __init__(self):
        self.position = Point()

    # Help with position put geometry msgs
    def update(self, position):
        self.position.x = position.x
        self.position.y = position.y
        self.position.z = position.z
    
    def debug(self) -> str:
        return '(x : "%f",y "%f",z "%f")' % (self.position.x, self.position.y, self.position.z)