#!/usr/bin/env python

import rospy
import sys

from lifecycle_msgs.msg import Lifecycle
from lm_monitor.listener import LmEventListener
from visualization_msgs.msg import Marker


class LifecycleToMarker(object):
    def __init__(self, node_names):
        self.node_names = node_names
        self.marker_pub = rospy.Publisher("lifecycle_markers", Marker, queue_size=10)
        self.listener = LmEventListener(self.lifecycle_callback)

    def lifecycle_callback(self, lm_events):
        if (self.node_names == None):
            self.displayAllNodeStatus(lm_events)
        else:
            for node_name in self.node_names:
                try:
                    self.displayNodeStatus(lm_events[node_name])
                except KeyError:
                    # TODO(lucasw) make a special Marker to show this
                    rospy.logwarn('Node not yet published the status {}'.format(node_name))

    def displayAllNodeStatus(self, lm_events_buffer):
        self.node_names = lm_events_buffer.keys()
        for node_name in self.node_names:
            node_status = lm_events_buffer[node_name]
            self.displayNodeStatus(node_status)

    def displayNodeStatus(self, lm_event):
        # TODO(lucasw) there also needs to be a keep alive capability,
        # where statuses need to be re-affirmed periodically?
        marker = Marker()
        marker.header.frame_id = lm_event.header.frame_id
        marker.header.stamp = lm_event.header.stamp
        marker.type = Marker.CUBE
        marker.ns = lm_event.header.frame_id
        marker.id = 0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        if lm_event.lifecycle_event.end_state == Lifecycle.PSTATE_UNCONFIGURED:
            marker.color.r = 0.5
            marker.color.g = 0.3
            marker.color.b = 0.3
        elif lm_event.lifecycle_event.end_state == Lifecycle.PSTATE_INACTIVE:
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        elif lm_event.lifecycle_event.end_state == Lifecycle.PSTATE_ACTIVE:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        elif lm_event.lifecycle_event.end_state == Lifecycle.PSTATE_FINALIZED:
            marker.color.r = 0.2
            marker.color.g = 0.0
            marker.color.b = 0.0
        else:
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
        marker.color.a = 1.0
        marker.frame_locked = True
        self.marker_pub.publish(marker)

if __name__ == '__main__':
    rospy.init_node('lifecycle_marker', anonymous=True)

    node_names = None
    # TODO(lucasw) this messes up rosparams
    if False:
        if len(sys.argv) < 2:
            node_names = None
        else:
            node_names = sys.argv[1:]

    lifecycle_marker = LifecycleToMarker(node_names)

    try:
        while not rospy.is_shutdown():
            pass
    except rospy.ROSInterruptException:
        pass
