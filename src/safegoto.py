import rospy
from sys import argv
from time import sleep
from geometry_msgs import msg
from math import sqrt, atan2, cos, sin
from nav_msgs.msg import Odometry
from p2os_msgs.msg import SonarArray, MotorState

LAST_X= LAST_Y = 0
LAST_THETA = 3.14

class Robot:
    pub = rospy.Publisher
    pub2 = rospy.Publisher
    CURRENT_X = float
    CURRENT_Y = float
    THETA = float
    GOAL_X = float
    GOAL_Y = float
    GOTO_X = float
    GOTO_Y = float
    subOdom = rospy.Subscriber
    subSonar = rospy.Subscriber
    sonarInfo = list
    turning = bool

    def __init__(self, GOAL_X, GOAL_Y):
        self.GOAL_X = self.GOTO_X = GOAL_X
        self.GOAL_Y = self.GOTO_Y = GOAL_Y
        self.THETA = LAST_THETA
        self.CURRENT_X , self.CURRENT_Y = LAST_X, LAST_Y
        self.pub = rospy.Publisher('/cmd_vel', msg.Twist, latch=True, queue_size=10)
        self.pub2 = rospy.Publisher('/cmd_motor_state', MotorState, latch=True)
        rospy.init_node('talker', anonymous=True)
        self.subOdom = rospy.Subscriber('/pose', Odometry, self.newLocation)
        self.subSonar = rospy.Subscriber('/sonar', SonarArray, self.sonar)
        self.turning = False
        self.init()

    def init(self):
        ms = MotorState()
        ms.state = 1
        self.pub2.publish(ms)

    def distanceAway(self, x, y):
        return sqrt((self.CURRENT_X - x) ** 2 + (self.CURRENT_Y - y) ** 2)

    def within(self, x, y):
        return  self.distanceAway(x,y) <= 0.05

    def withinGoal(self):
        return self.within(self.GOAL_X, self.GOAL_Y)

    def withinGoTo(self):
        return self.within(self.GOTO_X, self.GOTO_Y)

    def turn_left(self, angle_away):
        flag = 1 if self.THETA < angle_away else -1
        greater, lesser = (self.THETA, angle_away) if flag == -1 else (angle_away, self.THETA)
        return (-1 if 6.28 - greater + lesser < greater - lesser else 1) * flag

    def newLocation(self, msg):
        self.CURRENT_X = msg.pose.pose.position.x
        self.CURRENT_Y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.THETA = (2 * atan2(q.z, q.w) + 3.14) % 6.28
        self.calculateGoTo()
        self.goTo()


    def sonar(self, msg):
        self.sonarInfo = msg.ranges
        # print(self.sonarInfo)
        # print(msg.header)
        # print(msg.ranges_count)

    def calculateGoTo(self):

        if self.turning:
            return
        if self.withinGoTo():
            self.GOTO_X, self.GOTO_Y = self.GOAL_X, self.GOAL_Y
            return

        distance = self.distanceAway(self.GOTO_X, self.GOAL_Y)

        if self.sonarInfo[3] > 2 and self.sonarInfo[4] > 2:
            return
        elif self.sonarInfo[3] > distance and self.sonarInfo[4] > distance:
            return

        turnRight = self.sonarInfo[5] > 1.5 and self.sonarInfo[6] > 1.5
        # turnLeft =  self.sonarInfo[1] > 1.5 and self.sonarInfo[2] > 1.5


        if turnRight:
            # print("\n\n\nright\n\n\n")
            angle_change = (3.14-(6.28 - self.THETA - 0.872665)) % 6.28
        else:
            # print("\n\n\nleft\n\n\n")
            angle_change = (3.14-(6.28 - self.THETA + 0.872665)) % 6.28

        self.GOTO_X = self.CURRENT_X + (0.5) * cos(angle_change)
        self.GOTO_Y = self.CURRENT_Y + (0.5) * sin(angle_change)



    def goTo(self):
        ms = msg.Twist()
        angle_away = (atan2(self.GOTO_Y - self.CURRENT_Y, self.GOTO_X - self.CURRENT_X) + 3.14) % 6.28
        # print(self.CURRENT_X, self.CURRENT_Y, self.GOTO_X, self.GOTO_Y, self.THETA)
        if self.withinGoal():
            ms.linear.x = 0
            ms.angular.z = 0
            self.turning = False
            self.close()
        elif abs((self.THETA % 6.28) - angle_away % 6.28) > 0.0872665:
            ms.linear.x = 0
            ms.angular.z = 0.2 * (self.turn_left(angle_away))
            self.turning = True
        else:
            ms.linear.x = 0.2
            ms.angular.z = 0
            self.turning = False
        self.pub.publish(ms)

    def close(self):
        global LAST_Y,LAST_X,LAST_THETA
        LAST_Y,LAST_X,LAST_THETA = self.CURRENT_Y, self.CURRENT_X, self.THETA
        self.subOdom.unregister()
        self.subSonar.unregister()

    def __str__(self):
        return f"{self.CURRENT_X} {self.CURRENT_Y} {self.THETA}"

def read_file(filename):
    positions = list()
    for line in open(filename):
        line_split = line.strip().split()
        positions.append((float(line_split[0]), float(line_split[1])))
    return positions


def main():
    # print(argv)
    for (x, y) in read_file(argv[1]):
        r = Robot(x,y)
        while not r.withinGoal():
            sleep(5)
        sleep(1)
        print(r)

if __name__ == '__main__':
    main()