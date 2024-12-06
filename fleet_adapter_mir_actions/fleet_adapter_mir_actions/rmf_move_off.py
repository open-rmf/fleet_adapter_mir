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
import requests
from urllib.error import HTTPError
from fleet_adapter_mir.robot_adapter_mir import ActionContext


class BaseMoveOff(ABC):
    def __init__(self, context: ActionContext):
        self.context = context

    '''
    This method is called when the robot reaches the waiting waypoint and
    begins waiting. Use this callback to trigger any process during the
    waiting period.
    '''
    @abstractmethod
    def begin_waiting(self, description: dict):
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

    # --------------------------------------------------------------------------
    # HELPFUL FUNCTIONS FOR INTERACTING WITH MIR REST API
    # --------------------------------------------------------------------------

    def register_get(self, register: int):
        if not self.context.api.connected:
            return None
        try:
            response = requests.get(
                self.context.api.prefix + f'registers/{register}',
                headers=self.context.api.headers,
                timeout=self.context.api.timeout)
            if self.context.api.debug:
                print(f"Response: {response.headers}")
            # Response value is string, return integer of value
            return int(response.json().get('value', 0))
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
            return None
        except Exception as err:
            print(f"Other  error: {err}")
            return None
