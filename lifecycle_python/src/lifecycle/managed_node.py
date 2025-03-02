#!/usr/bin/env python

# ROS1 Lifecycle - A library implementing the ROS2 lifecycle for ROS1
#
# Copyright 2016,2017 Robert Bosch GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from abc import ABCMeta
from abc import abstractmethod

from lifecycle.lifecycle_model import LifecycleModel, State
from lifecycle.manager import LifecycleManager, Transition

class ManagedNode(object):
    __metaclass__ = ABCMeta
    def __init__(self, component_fqn, frame_id="map"):
        super(ManagedNode,self).__init__()
        self._lm = LifecycleManager(component_fqn, frame_id)
        self._lm.set_transition_callback(Transition.CONFIGURE, self._on_configure)
        self._lm.set_transition_callback(Transition.CLEANUP, self._on_cleanup)
        self._lm.set_transition_callback(Transition.ACTIVATE, self._on_activate)
        self._lm.set_transition_callback(Transition.DEACTIVATE, self._on_deactivate)
        self._lm.set_transition_callback(Transition.SHUTDOWN, self._on_shutdown)
        self._lm.set_error_cb(self._on_error)
        self._lm.set_state_change_cb(self._on_state_change)
        #start the action server
        self._lm.start()

    def __del__(self):
        self._lm.__del__()

    def _on_configure(self):
        return True

    def _on_cleanup(self):
        return False

    '''A node must not start directly after process creation when the life-cycle is in use,
    so the user needs to provide an on_activate callback and this is enforced by using abstractmethod.'''
    @abstractmethod
    def _on_activate(self):
        return True

    def _on_deactivate(self):
        return False

    def _on_shutdown(self):
        return True

    def _on_error(self, ex):
        return False

    def _on_state_change(self, state):
        pass

    # the above are overloaded with the actual state changing code,
    # below just changes state appropriately
    def handle_exception(self, ex):
        current_state = self._lm.get_current_state()
        if current_state != State.ErrorProcessing and current_state != State.UNCONFIGURED:
            self._lm.raise_error(ex)

    def configure(self):
        if self._lm.get_current_state() == State.UNCONFIGURED:
            self._lm.configure()

    def activate(self):
        self.configure()
        if self._lm.get_current_state() == State.INACTIVE:
            self._lm.activate()

    def deactivate(self):
        if self._lm.get_current_state() == State.ACTIVE:
            self._lm.deactivate()

    def cleanup(self):
        self.deactivate()
        if self._lm.get_current_state() == State.INACTIVE:
            self._lm.cleanup()

    def shutdown(self):
        self.deactivate()
        self.cleanup()
        if self._lm.get_current_state() == State.UNCONFIGURED:
            self._lm.shutdown()

    def get_state(self):
        return self._lm.get_current_state()

    def get_state_str(self):
        return LifecycleModel.STATE_TO_STR[self.get_state()]
