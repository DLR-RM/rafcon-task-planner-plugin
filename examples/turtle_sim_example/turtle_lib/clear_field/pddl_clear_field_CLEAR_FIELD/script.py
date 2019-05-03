import rospy
from turtlesim.srv import *
from std_srvs.srv import Empty

def execute(self, inputs, outputs, gvm):
    service = "/clear"
    rospy.wait_for_service(service)
    clear_turtle_area_service = rospy.ServiceProxy(service, Empty)
    resp1 = clear_turtle_area_service()
    self.logger.verbose("ROS external module: executed the {} service".format(service))
    return 0