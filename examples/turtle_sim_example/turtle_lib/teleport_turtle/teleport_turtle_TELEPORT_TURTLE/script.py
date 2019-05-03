import rospy
from turtlesim.srv import *


def execute(self, inputs, outputs, gvm):
    pmap = gvm.get_variable('rtpp_data')
    turtle_name = pmap[inputs["turtle"]]['name']
    loc = pmap[inputs['location']]
    x = loc['x_pos']
    y = loc["y_pos"]
    phi = 30
    service = "/" + turtle_name + "/teleport_absolute"
    rospy.wait_for_service(service)
    move_turtle = rospy.ServiceProxy(service, TeleportAbsolute)
    resp1 = move_turtle(x, y, phi)
    self.logger.info("ROS external module: executed the {} service".format(service))
    return 0
