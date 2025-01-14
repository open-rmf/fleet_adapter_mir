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


class BaseCartDetection(ABC):
    def __init__(self, context: ActionContext):
        self.context = context

    '''
    This method checks if the robot's latch is open and carrying a cart.
    Return True if latch is open, else False.
    '''
    @abstractmethod
    def is_latch_open(self):
        # To be implemented
        ...

    '''
    This method checks if the robot is currently docked under a cart.
    Return True if robot is under any carts, else False.
    '''
    @abstractmethod
    def is_under_cart(self):
        # To be implemented
        ...

    '''
    This method checks if the detected cart identifier matches the target
    cart_id, if any.
    Return True if cart is correct, False if cart is wrong, None if no cart is
    detected.
    '''
    @abstractmethod
    def is_correct_cart(self, cart_id: str | None):
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
                self.context.node.get_logger().debug(
                    f'Response: {response.headers}'
                )
            # Response value is string, return integer of value
            return int(response.json().get('value', 0))
        except HTTPError as http_err:
            self.context.node.get_logger().debug(f'HTTP error: {http_err}')
            return None
        except Exception as err:
            self.context.node.get_logger().debug(f'Other error: {err}')
            return None

    def io_module_guid_status_get(self, io_guid: str):
        if not self.context.api.connected:
            return None
        if io_guid is None:
            return None
        try:
            response = requests.get(
                self.context.api.prefix + f'io_modules/{io_guid}/status',
                headers=self.context.api.headers,
                timeout=self.context.api.timeout)
            if self.context.api.debug:
                self.context.node.get_logger().debug(
                    f'Response: {response.headers}'
                )
            if 'input_state' not in response.json():
                return None
            return response.json()['input_state']
        except HTTPError as http_err:
            self.context.node.get_logger().debug(f'HTTP error: {http_err}')
            return None
        except Exception as err:
            self.context.node.get_logger().debug(f'Other error: {err}')
            return None

    def io_modules_get(self):
        if not self.context.api.connected:
            return None
        try:
            response = requests.get(
                self.context.api.prefix + f'io_modules',
                headers=self.context.api.headers,
                timeout=self.context.api.timeout)
            if self.context.api.debug:
                self.context.node.get_logger().debug(
                    f'Response: {response.headers}'
                )
            # Response value is string, return integer of value
            return response.json()
        except HTTPError as http_err:
            self.context.node.get_logger().debug(f'HTTP error: {http_err}')
            return None
        except Exception as err:
            self.context.node.get_logger().debug(f'Other error: {err}')
            return None
