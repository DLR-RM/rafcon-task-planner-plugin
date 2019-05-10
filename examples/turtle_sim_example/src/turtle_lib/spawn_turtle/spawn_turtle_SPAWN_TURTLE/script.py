import rospy
from turtlesim.srv import *
from turtlesim.msg import *


pose_received = False


def check_turtle_pose(pose):
    global pose_received
    pose_received = True


def execute(self, inputs, outputs, gvm):
    print gvm.get_variable('rtpp_data')
    turtle = gvm.get_variable('rtpp_data')[inputs["turtle"]]
    turtle_name = turtle['name']
    x = 5
    y = 5
    phi = 30

    service = "/spawn"
    rospy.wait_for_service(service)
    spawn_turtle_service = rospy.ServiceProxy(service, Spawn)
    resp1 = spawn_turtle_service(x, y, phi, turtle_name)
    self.logger.verbose("ROS external module: executed the {} service".format(service))
    self.logger.info('spawning turtle {}'.format(turtle_name))
    turtle_pos_subscriber = rospy.Subscriber("/" + turtle_name + "/pose", Pose, check_turtle_pose)

    r = rospy.Rate(10)
    global pose_received
    # wait until the first pose message was received
    while not pose_received:
        r.sleep()

    return 0
