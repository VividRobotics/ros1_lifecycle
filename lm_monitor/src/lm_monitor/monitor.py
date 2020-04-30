#!/usr/bin/env python

import rospy

from lifecycle_msgs.msg import lm_events
from lifecycle_msgs.msg import Lifecycle
from lifecycle.lifecycle_model import LifecycleModel


# TODO(lucasw) this isn't providing a lot of value
class LmMonitor(object):
    """
    :class:`LmMonitor` collects the "/lm_monitor" and stores them in a dictionary for further use
    """

    def __init__(self):
        super(LmMonitor,self).__init__()
        self.LmEventsBuffer = {}

    def setLmEvent(self, lm_monitor_msg, time_stamp):
        """
        :param lm_monitor_msg: the lm_monitor message with node_name and lifecycle_event
        """
        self.LmEventsBuffer[lm_monitor_msg.ns + lm_monitor_msg.node_name] = lm_monitor_msg

if __name__ == '__main__':
    rospy.init_node('Lifecycle Monitor')

