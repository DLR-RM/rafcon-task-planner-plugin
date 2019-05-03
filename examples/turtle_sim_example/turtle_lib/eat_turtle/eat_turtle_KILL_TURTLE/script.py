import rospy
from turtlesim.srv import Kill
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

def execute(self, inputs, outputs, gvm):
    rtpp = gvm.get_variable('rtpp_data',per_reference=False, access_key=None, default=None)
    turtle = rtpp[inputs['turtle']]
    turtle_name = turtle['name']
    eater_name = rtpp[inputs['eater']]['name']
    service = "/kill"
    rospy.wait_for_service(service)
    kill_turtle_service = rospy.ServiceProxy(service, Kill)
    resp1 = kill_turtle_service(turtle_name)
    self.logger.verbose("ROS external module: executed the {} service".format(service))
    self.logger.info('{} is eating {}'.format(eater_name,turtle_name))
    return 0