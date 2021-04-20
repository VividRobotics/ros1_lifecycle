#!/usr/bin/env python
# Copyright (c) 2021 Lucas Walter
# Lucas Walter

# Move a lifecycle node to a different state, then exit
# TODO(lucasw) move this to a lifecycle utility package

import rospy

from lifecycle.client import LifecycleClient
from lifecycle.lifecycle_model import LifecycleModel


class LmChangeState(object):
    def __init__(self):
        # the namespace define which lifecycle node to connect to
        self._client = LifecycleClient.create_client()

        cur = rospy.Time.now()
        while True:
            elapsed = rospy.Time.now() - cur
            if elapsed > rospy.Duration(4.0):
                rospy.logerr("Couldn't get server state")
                rospy.signal_shutdown("Couldn't get server state, exiting")
                return
            cur_state = self._client._server_state
            if cur_state != None:
                break
            rospy.sleep(0.2)

        cur_state_str = LifecycleModel.STATE_TO_STR[cur_state]
        rospy.loginfo("Cur state {} {} {}".format(rospy.get_namespace(), cur_state_str, cur_state))

        goal_str = rospy.get_param("~state", "inactive")
        self.goal_state = LifecycleModel.STR_TO_STATE[goal_str]
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
