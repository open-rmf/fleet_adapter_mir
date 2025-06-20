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

import requests
from urllib.error import HTTPError

from .rmf_move_off import BaseMoveOff
from fleet_adapter_mir.robot_adapter_mir import ActionContext


class MoveOff(BaseMoveOff):
    def __init__(self, context: ActionContext):
        BaseMoveOff.__init__(self, context)

        '''
        This example demonstrates how we can notify the robot to move off when
        it reads a positive signal from a PLC register
        '''
        self.register = None

    def verify_signal(self, signal_config: dict) -> bool:
        plc_register = signal_config.get('register')
        if plc_register is None or not isinstance(plc_register, int):
            self.context.node.get_logger().error(
                f'[rmf_move_off_on_plc] requires a default PLC register, '
                f'but no PLC register was provided in the action config! '
            )
            return False
        return True

    def begin_waiting(self, signal_config: dict) -> bool:
        self.register = signal_config['register']
        return True

    def is_move_off_ready(self) -> bool:
        # Update register to check if PLC register returns a non-zero value
        value = self.register_get(self.register)
        if value:
            self.context.node.get_logger().info(
                f'[{self.context.name}] PLC register {self.register} detected '
                f'value {value}, robot is ready to move off.'
            )
            return True
        return False

    def register_get(self, register: int) -> int:
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
            value = response.json().get('value', 0)
            # Convert value into int if required
            if not isinstance(value, int):
                try:
                    return int(value)
                except ValueError as value_err:
                    self.context.node.get_logger().debug(
                        f'Value error: {value_err}'
                    )
                    return None
            else:
                return value
        except HTTPError as http_err:
            self.context.node.get_logger().debug(f'HTTP error: {http_err}')
            return None
        except Exception as err:
            self.context.node.get_logger().debug(f'Other  error: {err}')
            return None
