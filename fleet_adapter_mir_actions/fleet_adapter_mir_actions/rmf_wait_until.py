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

import time
import importlib
import requests
from urllib.error import HTTPError

from fleet_adapter_mir_actions.mir_action import MirAction, MirActionFactory
from fleet_adapter_mir.mir_api import MirAPI, MirStatus, MiRStateCode


class ActionFactory(MirActionFactory):
    def make_action(self,
                    node,
                    name,
                    mir_api,
                    update_handle,
                    fleet_config,
                    action_config) -> MirAction:
        return WaitUntil(
            node, name, mir_api, update_handle, fleet_config, action_config)


class WaitUntil(MirAction):
    def __init__(
            self,
            node,
            name,
            mir_api,
            update_handle,
            fleet_config,
            action_config
        ):
        MirAction.__init__(self, node, name, mir_api, update_handle,
                           fleet_config, action_config)

        self.execution = None
        self.start_time = None
        self.wait_timeout = action_config.get('timeout', 1800) # seconds
        self.move_off_cb = None

    def perform_action(self, category, description, execution):
        self.start_time = self.node.get_clock().now().nanoseconds / 1e9
        # Retrieve task description
        if description.get('timeout') is not None:
            self.wait_timeout = description['timeout']

        # The only accepted action name is "wait_until"
        if category != 'wait_until':
            self.node.get_logger().info(
                f'[{self.name}] Invalid perform action [{category}] passed '
                f'to WaitUntil, ending action.')
            execution.finished()
            return

        self.move_off_cb = self.create_move_off_cb(description)

        if self.move_off_cb is None:
            # Insufficient information provided to configure the check move-off
            # callback, mark action as completed and continue task
            self.node.get_logger().info(
                f'Insufficient information provided to configure the move-'
                f'off behavior, ignoring WaitUntil action and marking action '
                f'as completed.'
            )
            execution.finished()
            return

        self.node.get_logger().info(
            f'Robot [{self.name}] will start to perform wait_until '
            f'action with a timeout of {self.wait_timeout} seconds.'
        )
        # Store this execution object
        self.execution = execution

    def update_action(self):
        if self.execution is None:
            return
        # Check if action has been cancelled or killed
        if not self.execution.okay():
            self.node.get_logger().info(
                f'[{self.name}] wait_until action has been killed/canceled'
            )
            return True

        # Check if we've received a move-off signal
        if self.move_off_cb():
            self.node.get_logger().info(
                f'[{self.name}] has received move-off signal, marking '
                f'action as completed.'
            )
            self.execution.finished()
            return True

        # Check if the default timeout has passed
        now = self.node.get_clock().now().nanoseconds / 1e9
        if now > self.start_time + self.wait_timeout:
            self.node.get_logger().info(
                f'Robot [{self.name}] has completed waiting for '
                f'{self.wait_timeout} seconds without move-off signal, '
                f'action as complete.'
            )
            return True

        # Log the robot waiting every 30 seconds
        seconds_passed = round(now - self.start_time)
        if seconds_passed%30 == 0:
            self.node.get_logger().info(
                f'{seconds_passed} seconds have passed since robot '
                f'[{self.name}] started its waiting action.'
            )

        return False

    def create_move_off_cb(self, description: dict):
        signal_cb = None

        # Determine which move-off behavior config to use. Any config populated
        # in the task description overrides the default config provided in
        # fleet config.
        if 'signal_type' in description:
            signal_config = description
        elif 'signal_type' in self.action_config:
            signal_config = self.action_config
        else:
            # We have no move-off behavior configured at all, we will just wait
            # until we reach the default timeout
            self.node.get_logger().info(
                f'No move-off behavior was configured for [{self.name}], the '
                f'robot will begin waiting until the default timeout.'
            )
            signal_cb = lambda: False
            return signal_cb

        # Configure type of move off signal
        signal_type = signal_config['signal_type']
        match signal_type:
            case "mission":
                mission_name = signal_config.get('mission_name', None)
                resubmit_on_abort = signal_config.get('resubmit_on_abort',
                                                      False)
                if mission_name is None:
                    self.node.get_logger().info(
                        f'No mission name provided!'
                    )
                    return None
                elif mission_name not in self.api.known_missions:
                    self.node.get_logger().info(
                        f'Mission {mission_name} not found on robot '
                        f'{self.name}!'
                    )
                    return None
                mission_actions = self.api.missions_mission_id_actions_get(
                    self.api.known_missions[mission_name]['guid']
                )
                if not mission_actions:
                    self.node.get_logger().info(
                        f'Mission {mission_name} actions not found on robot '
                        f'{self.name}!'
                    )
                    return None
                # Queue the waiting mission for this robot
                retry_count = 10
                count = 0
                mission_queue_id = None
                while count < retry_count and not mission_queue_id:
                    count += 1
                    self.node.get_logger().info(
                        f'Queueing mission {mission_name} for robot '
                        f'[{self.name}]...'
                    )
                    try:
                        mission_queue_id = self.api.queue_mission_by_name(
                            mission_name)
                        if mission_queue_id is not None:
                            break
                    except Exception as err:
                        self.node.get_logger().info(
                            f'Failed to queue mission {mission_name}: {err}. '
                            f'Retrying...'
                        )
                    time.sleep(1)
                if not mission_queue_id:
                    self.node.get_logger().info(
                        f'Unable to queue mission {mission_name} for robot '
                        f'[{self.name}]!'
                    )
                    return None
                self.node.get_logger().info(
                    f'Mission {self.mission_name} queued for [{self.name}] '
                    f'with mission queue id {mission_queue_id}.'
                )
                signal_cb = lambda: self.check_mission_complete(
                    mission_queue_id, resubmit_on_abort)
                self.node.get_logger().info(
                    f'Configuring robot [{self.name}] move-off behavior: '
                    f'robot will wait until mission {mission_name} with '
                    f'mission queue id {mission_queue_id} is completed.'
                )
            case "plc":
                register = signal_config.get('plc_register', None)
                if register is None:
                    self.node.get_logger().info(
                        f'No PLC register provided!'
                    )
                    return None
                signal_cb = lambda: self.check_plc_register(register)
                self.node.get_logger().info(
                    f'Configuring robot [{self.name}] move-off behavior: '
                    f'robot will wait until the PLC register {register} '
                    f'returns True.'
                )
            case "custom":
                signal_cb = lambda: self.create_custom_signal(
                    signal_config, description)
                self.node.get_logger().info(
                    f'Configuring robot [{self.name}] move-off behavior: '
                    f'robot will wait until the custom move-off behavior '
                    f'signals that the robot is ready to move off.'
                )
            case _:
                self.node.get_logger().info(
                    f'Invalid move off signal type provided, unable to '
                    f'initialize a WaitUntil action for {self.name}.'
                )
        return signal_cb

    def check_mission_complete(self,
                               mission_queue_id,
                               resubmit_on_abort=False):
        mission_status = self.api.mission_queue_id_get(mission_queue_id)
        if (mission_status is not None and
                mission_status['state'] == 'Done'):
            # Mission has completed, we can set move_off to True
            self.node.get_logger().info(
                f'Robot [{self.name}] has completed its mission '
                f'with mission queue id {mission_queue_id}'
            )
            return True

        if (mission_status is not None and
                mission_status['state'] == 'Aborted'):
            if resubmit_on_abort:
                # Mission aborted for some reason, let's submit the mission
                # again
                new_mission_queue_id = self.api.queue_mission_by_name(
                    self.mission_name)
                if not new_mission_queue_id:
                    # If we didn't successfully post a new mission, we'll
                    # try again in the next loop
                    return
                # Update the check move off callback with the updated mission
                # queue id
                self.move_off_cb = lambda: self.check_mission_complete(
                    new_mission_queue_id, resubmit_on_abort)
                self.node.get_logger().info(
                    f'Robot [{self.name}] aborted mission with queue id '
                    f'{mission_queue_id}, re-submitting mission with new '
                    f'queue id {new_mission_queue_id}'
                )
            else:
                # If mission is aborted without option to resubmit on abort,
                # mark mission as finished
                self.node.get_logger().info(
                    f'Robot [{self.name}] has aborted its mission with '
                    f'mission queue id {mission_queue_id}, marking action as '
                    f'completed'
                )
                return True
        return False

    def check_plc_register(self, register: int):
        # Update register to check if PLC register returns True
        value = self.register_get(register)
        if value:
            self.node.get_logger().info(
                f'[{self.name}] PLC register {register} detected '
                f'value {value}, robot is ready to move off.'
            )
            return True
        return False

    def create_custom_signal(self, signal_config, description):
        # Import the move-off behavior module
        if 'move_off_module' not in signal_config:
            self.node.get_logger().info(
                f'Move-off behavior is set to custom but no move-off module '
                f'was provided!'
            )
            return None
        move_off_module = signal_config['move_off_module']
        move_off_plugin = importlib.import_module(move_off_module)
        move_off_signal = move_off_plugin.MoveOff(
            self, signal_config, description)

        # Trigger the begin waiting callback
        move_off_signal.begin_waiting(description)
        # Define the move-off callback to be called on every update
        move_off_cb = lambda: move_off_signal.is_move_off_ready()
        return move_off_cb

    # ------------------------------------------------------------------------------------------------------------------
    # MIR API FOR WAIT RELATED MISSIONS
    # ------------------------------------------------------------------------------------------------------------------

    def register_get(self, register: int):
        if not self.api.connected:
            return None
        try:
            response = requests.get(self.api.prefix + f'registers/{register}',
                                    headers=self.api.headers,
                                    timeout=self.api.timeout)
            if self.api.debug:
                print(f"Response: {response.headers}")
            # Response value is string, return integer of value
            return int(response.json().get('value', 0))
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
            return None
        except Exception as err:
            print(f"Other  error: {err}")
            return None