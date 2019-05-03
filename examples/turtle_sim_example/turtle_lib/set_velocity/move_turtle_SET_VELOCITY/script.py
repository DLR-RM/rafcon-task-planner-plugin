from builtins import str
import rospy
import math
from turtlesim.srv import *
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

def execute(self, inputs, outputs, gvm):
    turtle = gvm.get_variable('rtpp_data')[inputs["turtle"]]
    turtle_name = turtle['name']
    global_storage_id_of_turtle_pose = turtle['global_storage_id_of_turtle_pos']
    my_x = gvm.get_variable(global_storage_id_of_turtle_pose + "/" + "x")
    my_y = gvm.get_variable(global_storage_id_of_turtle_pose + "/" + "y")
    x = inputs["x_vel"]
    #phi = math.tan(my_x/my_y)
    phi = 0.9
    rate = rospy.Rate(10)
    position_vector = Vector3(x, 0, 0)
    rotation_vector = Vector3(0, 0, phi)
    twist_msg = Twist(position_vector, rotation_vector)
    self.logger.info("moving turtle {} {}".format(str(x), str(phi)))
    self.logger.info("publish twist to turtle {}".format(turtle_name))
    turtle_vel_publisher = rospy.Publisher("/" + turtle_name + "/cmd_vel", Twist, queue_size=10, latch=True)
    turtle_vel_publisher.publish(twist_msg)
    rate.sleep()

    return 0