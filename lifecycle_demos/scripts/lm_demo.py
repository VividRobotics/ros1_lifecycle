#!/usr/bin/env python
# Copyright (c) 2021 Lucas Walter
# Lucas Walter
# April 18th

# Demonstrate a python lifecycle managed node with a variety of services and clients

import traceback
import rospy

from lifecycle.lifecycle_model import LifecycleModel, State
from lifecycle.managed_node import ManagedNode
from std_msgs.msg import Float32

class LmDemo(ManagedNode):
    def __init__(self):
        # TODO(lucasw) allow passing in a verbose flag that would do the loginfos
        # bracketing each state transition
        super(LmDemo, self).__init__(rospy.get_name())
        rospy.loginfo("init")
        self.pub = None
        self.sub = None

        # TODO(lucasw) also dynamic dynamic reconfigure,
        # action client or server

    def _on_configure(self):
        rospy.loginfo("Configuring")

        self.value = rospy.get_param("value", 1.0)
        self.pub = rospy.Publisher("pub", Float32, queue_size=4)
        self.sub = rospy.Subscriber("sub", Float32, queue_size=4)
        # TODO(lucasw) set up a service server + client

        rospy.loginfo("configured")
        return True

    def _on_activate(self):
        rospy.loginfo("Activating")
        self.timer = rospy.Timer(rospy.Duration(1.0), self._update)
        rospy.loginfo("activated")
        return True

    def _on_deactivate(self):
        rospy.loginfo("De-activating")
        self.timer.shutdown()
        rospy.loginfo("deactivated")
        return True

    def _on_cleanup(self):
        rospy.loginfo("Cleaning up")
        self.sub.unregister()
        self.pub.unregister()
        rospy.loginfo("cleaned up")
        return True

    def _on_shutdown(self):
        rospy.loginfo("Shutting down")
        rospy.Timer(rospy.Duration(1.0), self._exit_shutdown, oneshot=True)
        return True

    def _exit_shutdown(self, event):
        rospy.logwarn("exiting")
        rospy.signal_shutdown("exiting")

    def _on_error(self, ex):
        rospy.logerr(traceback.format_exc())
        # TODO(lucasw) return False instead?
        return True

    def _update(self, event):
        rospy.loginfo("activated update")

    def node_shutdown_signal(self):
        rospy.loginfo("ros shutdown signal from {}".format(lm_demo.get_state_str()))
        lm_demo.cleanup()
        return


if __name__ == '__main__':
    rospy.init_node('lm_demo')
    lm_demo = LmDemo()
    rospy.on_shutdown(lm_demo.node_shutdown_signal)
    rospy.spin()
