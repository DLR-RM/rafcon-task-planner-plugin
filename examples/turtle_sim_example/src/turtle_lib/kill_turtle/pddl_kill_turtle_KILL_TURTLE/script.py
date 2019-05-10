import rospy
from turtlesim.srv import Kill
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

def execute(self, inputs, outputs, gvm):
    turtle = gvm.get_variable('rtpp_data',per_reference=False, access_key=None, default=None)[inputs['turtle']]
    turtle_name = turtle['name']
    service = "/kill"
    rospy.wait_for_service(service)
    kill_turtle_service = rospy.ServiceProxy(service, Kill)
    resp1 = kill_turtle_service(turtle_name)
    self.logger.verbose("ROS external module: executed the {} service".format(service))
    self.logger.info('Killing {}'.format(turtle_name))
    return 0