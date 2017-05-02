import rospy
from sensor_msgs.msg import Joy
rospy.init_node("test123")
pub = rospy.Publisher('joy', Joy, queue_size=5)
j = Joy()
j.axes = [1,2,3]
j.buttons = [0, 0, 0]
pub.publish(j)