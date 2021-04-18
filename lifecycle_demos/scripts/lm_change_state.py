#!/usr/bin/env python
# Copyright (c) 2021 Lucas Walter
# Lucas Walter

# Move a lifecycle node to a different state, then exit
# TODO(lucasw) move this to a lifecycle utility package

import rospy

from lifecycle.client import create_client
from lifecycle.lifecycle_model import LifecycleModel


class LmChangeState(object):
    def __init__(self):
        # the namespace define which lifecycle node to connect to
        self._client = create_client()

        goal_str = rospy.get_param("~goal", "inactive")
        self.goal_state = LifecycleModel.STR_TO_STATE[goal_str]
        # TODO(lucasw) print current state
        rospy.loginfo("Goal state {} {}".format(goal_str, self.goal_state))
        self._client.go_to_state(self.goal_state, self.transition_cb)

    def transition_cb(self, result):
        if result:
            rospy.loginfo("result {}".format(result))
        else:
            rospy.logwarn("result {}".format(result))
        rospy.signal_shutdown("finished state change attempt, exiting")


if __name__ == '__main__':
    rospy.init_node("lm_change_state")
    lm_change_state = LmChangeState()
    rospy.spin()
