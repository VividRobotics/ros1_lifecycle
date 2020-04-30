#!/usr/bin/env python

import rospy
import sys

from lifecycle_msgs.msg import Lifecycle
from lm_monitor.listener import LmEventListener
from visualization_msgs.msg import Marker, MarkerArray


class LifecycleToMarker(object):
    def __init__(self, node_names):
        self.node_names = node_names
        self.marker_pub = rospy.Publisher("lifecycle_marker_array", MarkerArray, queue_size=10)
        self.listener = LmEventListener(self.lifecycle_callback)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.update)

    def lifecycle_callback(self, lm_events):
        self.buffer = lm_events

    def update(self, event):
        marker_array = MarkerArray()
        markers = self.displayAllNodeStatus(self.buffer)
        marker_array.markers.extend(markers)

        # TODO(lucasw) optionally have a timeout and if a node hasn't updated recently
        # make it turn to special failed state, though also need to change broadcasters
        # to conform to this regular update expectation.
        if False:
            if (self.node_names == None):
                markers = self.displayAllNodeStatus(lm_events)
                marker_array.markers.extend(markers)
            else:
                for node_name in self.node_names:
                    try:
                        markers = self.get_status_markers(lm_events[node_name])
                        marker_array.markers.extend(markers)
                    except KeyError:
                        # TODO(lucasw) make a special Marker to show this
                        rospy.logwarn('Node not yet published the status {}'.format(node_name))

        self.marker_pub.publish(marker_array)


    def displayAllNodeStatus(self, lm_events_buffer):
        self.node_names = lm_events_buffer.keys()
        markers = []
        for node_name in self.node_names:
            lm_event = lm_events_buffer[node_name]
            markers.extend(self.get_status_markers(lm_event))
        return markers

    def get_status_markers(self, lm_event):
        markers = []
        marker = self.get_status_marker(lm_event)
        markers.append(marker)
        marker = self.get_status_marker(lm_event)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = lm_event.node_name
        marker.scale.x = 0.0
        marker.scale.y = 0.0
        marker.scale.z = 0.02
        marker.pose.position.x -= 0.1
        marker.color.r *= 0.8
        marker.color.g *= 0.8
        marker.color.b *= 0.8
        marker.id += 1
        markers.append(marker)
        return markers

    def get_status_marker(self, lm_event):
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
        return marker

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
