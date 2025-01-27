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

from abc import ABC, abstractmethod
from fleet_adapter_mir.robot_adapter_mir import ActionContext


class BaseMoveOff(ABC):
    def __init__(self, context: ActionContext):
        self.context = context

    '''
    This method verifies that the configuration for this move off signal is
    valid.
    Returns True if valid, else False.
    '''
    @abstractmethod
    def verify_signal(self, signal_config: dict) -> bool:
        # To be implemented
        ...

    '''
    This method is called when the robot reaches the waiting waypoint and
    begins waiting. Use this callback to trigger any process during the
    waiting period.
    Returns False if the signal setup fails, else True.
    '''
    @abstractmethod
    def begin_waiting(self, signal_config: dict) -> bool:
        # To be implemented
        ...

    '''
    This method checks if the robot is ready to move off.
    Returns True if ready, else False.
    '''
    @abstractmethod
    def is_move_off_ready(self) -> bool:
        # To be implemented
        ...
