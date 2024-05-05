# Import files from rosilo library
from rosilo_datalogger.msg import AddValueMsg

import rospy


class StoreInterface:
    """
    This class works as an interface that sends values and their names to the "store" node using ROS publisher.
    """
    def __init__(self):
        self.publisher_store_data_ = rospy.Publisher("store/store_data", AddValueMsg, queue_size=1)

    def send_store_data(self, data_name, data_value):
        msg = AddValueMsg(name=data_name, value=data_value)
        self.publisher_store_data_.publish(msg)
