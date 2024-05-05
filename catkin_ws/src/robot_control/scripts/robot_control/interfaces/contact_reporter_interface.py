import rospy
from std_msgs.msg import Bool


class ContactReporterInterface:
    def __init__(self):
        self.contact = False
        self.subscriber_contatct_ = rospy.Subscriber("arduino/contact", Bool, self._contact_callback)

    def _contact_callback(self, msg):
        self.contact = msg.data
