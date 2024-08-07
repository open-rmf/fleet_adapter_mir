# Copyright 2024 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


class MoveOff:
    def __init__(
            self,
            action_handle,
            signal_config,
            task_description
    ):
        self.action = action_handle
        self.config = signal_config
        self.task_desc = task_description

    def begin_waiting(self, description):
        '''
        This will be called when the robot has reached the waiting waypoint and
        begins waiting. Use this callback to trigger any process during the
        waiting period.
        '''
        # ------------------------
        # IMPLEMENT YOUR CODE HERE
        # ------------------------
        pass

    def is_move_off_ready(self):
        '''
        Checks if the robot is ready to move off.
        Returns True if ready, else False.
        '''
        # ------------------------
        # IMPLEMENT YOUR CODE HERE
        # ------------------------
        return False
