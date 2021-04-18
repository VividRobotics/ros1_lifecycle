#!/usr/bin/env python
# Copyright (c) 2021 Lucas Walter
# Lucas Walter
# April 18th

# Demonstrate a python lifecycle managed node with a variety of services and clients

import traceback
import rospy

from lifecycle.client import create_client
from lifecycle.lifecycle_model import LifecycleModel, State
from lifecycle.managed_node import ManagedNode
from std_msgs.msg import Float32


class LmParent(ManagedNode):
    def __init__(self):
        # TODO(lucasw) allow passing in a verbose flag that would do the loginfos
        # bracketing each state transition
        super(LmParent, self).__init__(rospy.get_name())
        rospy.loginfo("init")

        # TODO(lucasw) also dynamic dynamic reconfigure,
        # action client or server

    def _set_child_states(self, state, reverse=False):
        for key in sorted(self.children.keys(), reverse=reverse):
            if not self.children[key].go_to_state_timed(state, timeout=1.0):
                return False
        return True

    def _on_configure(self):
        rospy.loginfo("Configuring")

        children_ns = {}
        children_ns["001 child1"] = "lower/child1"
        children_ns["001 child2"] = "lower/child2"
        self.children = {}

        for key in sorted(children_ns.keys()):
            self.children[key] = create_client(children_ns[key])

        if not self._set_child_states(State.INACTIVE):
            return False
        # TODO(lucasw) set up a service server + client

        rospy.loginfo("configured")
        return True

    def _on_activate(self):
        rospy.loginfo("Activating")
        if not self._set_child_states(State.ACTIVE):
            return False
        self.timer = rospy.Timer(rospy.Duration(1.0), self._update)
        rospy.loginfo("activated")
        return True

    def _on_deactivate(self):
        rospy.loginfo("De-activating")
        self.timer.shutdown()
        if not self._set_child_states(State.INACTIVE, reverse=True):
            return False
        rospy.loginfo("deactivated")
        return True

    def _on_cleanup(self):
        rospy.loginfo("Cleaning up")
        if not self._set_child_states(State.UNCONFIGURED, reverse=True):
            # Try to clean up as much as possible
            # return False
            pass
        rospy.loginfo("cleaned up")
        return True

    def _on_shutdown(self):
        rospy.loginfo("Shutting down")
        if not self._set_child_states(State.FINALIZED):
            pass
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
        pass
        # rospy.loginfo("activated update")

    def node_shutdown_signal(self):
        rospy.loginfo("ros shutdown signal from {}".format(lm_parent.get_state_str()))
        lm_parent.cleanup()
        return


if __name__ == '__main__':
    rospy.init_node('lm_parent')
    lm_parent = LmParent()
    rospy.on_shutdown(lm_parent.node_shutdown_signal)
    rospy.spin()
