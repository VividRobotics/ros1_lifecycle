#!/usr/bin/env python

import rospy

from lifecycle_msgs.msg import lm_events
from lifecycle_msgs.msg import Lifecycle

class LmEventBroadcaster(object):
    """
    :class:`LmEventBroadcaster` is a convenient way to send Lifecycle event updates on the ``"/lm_events"`` message topic.
    """

    def __init__(self, component_fqn, frame_id="map", queue_size=100):
        self.node_name = component_fqn
        self.frame_id = frame_id
        self.pub_lm_monitor = rospy.Publisher("/lm_events", lm_events, latch=True, queue_size=queue_size)

    def sendLmEvent(self, lifecycle_msg):
        """
        :param lifecycle_msg: the lifecycle message with transition, end_state and result code
        """
        lm_monitor_msg = lm_events()
        # TODO(lucasw) get the stamp from the lifecycle_msg
        lm_monitor_msg.header.stamp = rospy.Time.now()
        lm_monitor_msg.header.frame_id = self.frame_id
        lm_monitor_msg.node_name = self.node_name
        lm_monitor_msg.ns = rospy.get_namespace()
        # TODO(lucasw) delete this line?  Is it informal type info?
        lm_monitor_msg.lifecycle_event = Lifecycle()
        lm_monitor_msg.lifecycle_event = lifecycle_msg
        self.pub_lm_monitor.publish(lm_monitor_msg)


if __name__ == '__main__':
    rospy.init_node('LifecycleEventBroadcaster')
