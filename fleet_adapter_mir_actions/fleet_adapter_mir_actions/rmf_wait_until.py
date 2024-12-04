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
from fleet_adapter_mir.robot_adapter_mir import ActionContext
from fleet_adapter_mir.mir_api import MirAPI, MirStatus, MiRStateCode


class ActionFactory(MirActionFactory):
    def __init__(self, context: ActionContext):
        MirActionFactory.__init__(context)
        self.move_off = None
        # Raise error if config file is invalid
        if 'signal_type' in context.action_config:
            signal_type = context.action_config['signal_type']
            if 'mission' in signal_type:
                if 'mission_name' not in signal_type['mission']:
                    raise KeyError(
                        f'WaitUntil MirAction requires a default mission name '
                        f'for signal type [mission], but mission name is not '
                        f'provided in the action config! Unable to '
                        f'instantiate an ActionFactory.')
            if 'plc' in signal_type:
                if not isinstance(signal_type['plc'], int):
                    raise TypeError(
                        f'WaitUntil MirAction requires a default PLC register '
                        f'number for signal type [plc], but the value '
                        f'provided is not an integer! Unable to instantiate '
                        f'an ActionFactory.')
            # If user provides a custom MoveOff module, import it
            if 'custom' in signal_type:
                if 'module' not in signal_type['custom']:
                    raise KeyError(
                        f'WaitUntil MirAction requires a custom module for '
                        f'signal type [custom], but path to module is not '
                        f'provided in the action config! Unable to '
                        f'instantiate an ActionFactory.')
                try:
                    move_off_module = context.action_config['module']
                    move_off_plugin = importlib.import_module(move_off_module)
                    self.move_off = move_off_plugin.MoveOff(self.context)
                except ImportError:
                    self.context.node.get_logger().warn(
                        f'Unable to import module {move_off_module}! The '
                        f'WaitUntil ActionFactory will be instantiated '
                        f'without supporting any user-defined signal module.'
                    )
        else:
            # If the user did not provide any default signal config, log a
            # warning to remind users to provide signal type config in any task
            # description they submit
            self.context.node.get_logger().warn(
                f'WaitUntil ActionFactory is instantiated for robot '
                f'[{self.context.name}], but no signal_type config has been '
                f'provided in the action config! Any move off signal config '
                f'will have to be provided in the task description submitted '
                f'to RMF.'
            )

    def supports_action(self, category: str) -> bool:
        match category:
            case 'wait_until':
                return True
            case _:
                return False

    def perform_action(
        self,
        category: str,
        description: dict,
        execution
    ) -> MirAction:
        match category:
            case 'wait_until':
                return WaitUntil(
                    description, execution, self.context, self.move_off)


class WaitUntil(MirAction):
    def __init__(
        self,
        description: dict,
        execution,
        context: ActionContext,
        move_off
    ):
        MirAction.__init__(self, context, execution)

        self.move_off_cb = self.create_move_off_cb(description, move_off)
        if self.move_off_cb is None:
            # Insufficient information provided to configure the check move off
            # callback, mark action as completed and continue task
            self.context.node.get_logger().info(
                f'Insufficient information provided to configure the move-'
                f'off behavior for robot [{self.context.name}], cancelling '
                f'task...'
            )
            self.cancel_task(
                label='Move off behavior cannot be configured, unable to ' + \
                    'perform wait until action.'
            )
            return

        self.logging_gap = \
            context.action_config.get('logging_gap', 60)  # seconds
        if description.get('logging_gap') is not None:
            self.logging_gap = description['logging_gap']
        self.wait_timeout = context.action_config.get('timeout', 60)  # seconds
        if description.get('timeout') is not None:
            self.wait_timeout = description['timeout']

        self.start_time = self.context.node.get_clock().now().nanoseconds / 1e9
        self.context.node.get_logger().info(
            f'New wait until action with a timeout of {self.wait_timeout} '
            f'seconds requested for robot [{self.context.name}]')

    def cancel_task(self, label: str = ''):
        def _cancel_success():
            pass
        def _cancel_fail():
            pass
        self.cancel_task_of_action(_cancel_success, _cancel_fail, label)

    def update_action(self):
        # If the action is no longer active, mark action as complete
        if self.execution is not None and not self.execution.okay():
            self.context.node.get_logger().info(
                f'[wait_until] action is no longer underway and valid, '
                f'marking action as complete.')
            return True

        # Check if we've received a move off signal
        if self.move_off_cb():
            self.context.node.get_logger().info(
                f'[{self.context.name}] has received move off signal, marking '
                f'action as completed.'
            )
            self.execution.finished()
            return True

        # Check if the default timeout has passed
        now = self.context.node.get_clock().now().nanoseconds / 1e9
        if now > self.start_time + self.wait_timeout:
            self.context.node.get_logger().info(
                f'Robot [{self.context.name}] has completed waiting for '
                f'{self.wait_timeout} seconds without move off signal, '
                f'action as complete.'
            )
            return True

        # Log the robot waiting every X seconds
        seconds_passed = round(now - self.start_time)
        if seconds_passed%self.logging_gap == 0:
            self.context.node.get_logger().info(
                f'{seconds_passed} seconds have passed since robot '
                f'[{self.context.name}] started its waiting action.'
            )

        return False

    def create_move_off_cb(self, description: dict, move_off):
        signal_cb = None

        # Determine which move off signal config to use. Any config populated
        # in the task description overrides the default config provided in
        # action config.
        if 'signal_type' in description:
            signal_type = description['signal_type']
        elif 'signal_type' in self.context.action_config:
            signal_type = self.context.action_config['signal_type']
        else:
            # There is no move off signal provided, we will just wait for the
            # duration of the configured timeout
            timeout = description.get('timeout', self.wait_timeout)
            self.context.node.get_logger().info(
                f'No move off signal was configured for [{self.context.name}]'
                f', the robot will begin waiting until the configured timeout '
                f'of [{timeout}] seconds.'
            )
            signal_cb = lambda: False
            return signal_cb

        # Configure the specified move off signal
        match signal_type:
            case "mission":
                mission_config = self.context.action_config.get('mission')
                mission_name = description.get('mission_name', None)
                resubmit_on_abort = description.get('resubmit_on_abort', None)
                retry_count = description.get('retry_count', None)

                # Check for default mission config values if they are not
                # provided in the task description
                if mission_config is not None:
                    if mission_name is None:
                        mission_name = mission_config['mission_name']
                    if resubmit_on_abort is None:
                        resubmit_on_abort = \
                            mission_config.get('resubmit_on_abort', False)
                    if retry_count is None:
                        retry_count = \
                            mission_config.get('retry_count', 10)

                # Check if mission config values are valid
                if mission_name is None:
                    self.context.node.get_logger().info(
                        f'MoveOff signal type [mission] was selected for '
                        f'robot [{self.context.name}], but no mission name '
                        f'was provided! Please ensure that the required '
                        f'fields are provided in the fleet config.'
                    )
                    return None
                if mission_name not in self.context.api.known_missions:
                    self.context.node.get_logger().info(
                        f'Mission {mission_name} not found on robot '
                        f'{self.context.name}!'
                    )
                    return None
                mission_actions = \
                    self.context.api.missions_mission_id_actions_get(
                    self.context.api.known_missions[mission_name]['guid']
                )
                if not mission_actions:
                    self.context.node.get_logger().info(
                        f'Mission {mission_name} actions not found on robot '
                        f'{self.context.name}!'
                    )
                    return None
                # Queue the waiting mission for this robot
                count = 0
                mission_queue_id = None
                while count < retry_count and not mission_queue_id:
                    count += 1
                    self.context.node.get_logger().info(
                        f'Queueing mission {mission_name} for robot '
                        f'[{self.context.name}]...'
                    )
                    try:
                        mission_queue_id = self.context.api.queue_mission_by_name(
                            mission_name)
                        if mission_queue_id is not None:
                            break
                    except Exception as err:
                        self.context.node.get_logger().info(
                            f'Failed to queue mission {mission_name}: {err}. '
                            f'Retrying...'
                        )
                    time.sleep(1)
                if not mission_queue_id:
                    self.context.node.get_logger().info(
                        f'Unable to queue mission {mission_name} for robot '
                        f'[{self.context.name}]!'
                    )
                    return None
                self.context.node.get_logger().info(
                    f'Mission {mission_name} queued for [{self.context.name}] '
                    f'with mission queue id {mission_queue_id}.'
                )
                signal_cb = lambda: self.check_mission_complete(
                    mission_name, mission_queue_id, resubmit_on_abort)
                self.context.node.get_logger().info(
                    f'Configuring robot [{self.context.name}] move off signal: '
                    f'robot will wait until mission {mission_name} with '
                    f'mission queue id {mission_queue_id} is completed.'
                )
            case "plc":
                register = description.get(
                    'plc', self.context.action_config.get('plc', None))
                if register is None:
                    self.context.node.get_logger().info(
                        f'MoveOff signal type [plc] was selected for '
                        f'robot [{self.context.name}], but no plc register '
                        f'was provided! Please ensure that the required '
                        f'fields are provided in the fleet config.'
                    )
                    return None
                signal_cb = lambda: self.check_plc_register(register)
                self.context.node.get_logger().info(
                    f'Configuring robot [{self.context.name}] move off '
                    f'signal: robot will wait until the PLC register '
                    f'{register} returns True.'
                )
            case "custom":
                if move_off is None:
                    self.context.node.get_logger().info(
                        f'MoveOff signal type [custom] was selected for '
                        f'robot [{self.context.name}], but no valid move off '
                        f'signal module was provided! Please ensure that the '
                        f'required fields are provided in the fleet config.'
                    )
                    return None
                # Trigger the begin waiting callback
                move_off.begin_waiting(description)
                # Define the move off callback to be called on every update
                signal_cb = lambda: move_off.is_move_off_ready()
                self.context.node.get_logger().info(
                    f'Configuring robot [{self.context.name}] move off '
                    f'behavior: robot will wait until the custom move off '
                    f'behavior signals that the robot is ready to move off.'
                )
            case _:
                self.context.node.get_logger().info(
                    f'Invalid move off signal type [{signal_type}] provided, '
                    f'unable to initialize a WaitUntil action for '
                    f'{self.context.name}.'
                )
        return signal_cb

    def check_mission_complete(
        self,
        mission_name,
        mission_queue_id,
        resubmit_on_abort
    ):
        mission_status = self.context.api.mission_queue_id_get(mission_queue_id)
        if (mission_status is not None and
                mission_status['state'] == 'Done'):
            # Mission has completed, we can set move_off to True
            self.context.node.get_logger().info(
                f'Robot [{self.context.name}] has completed its mission '
                f'with mission queue id {mission_queue_id}'
            )
            return True

        if (mission_status is not None and
                mission_status['state'] == 'Aborted'):
            if resubmit_on_abort:
                # Mission aborted for some reason, let's submit the mission
                # again
                new_mission_queue_id = self.context.api.queue_mission_by_name(
                    mission_name)
                if not new_mission_queue_id:
                    # If we didn't successfully post a new mission, we'll
                    # try again in the next update_action loop
                    return
                # Update the check move off callback with the updated mission
                # queue id
                self.move_off_cb = lambda: self.check_mission_complete(
                    mission_name,
                    new_mission_queue_id)
                self.context.node.get_logger().info(
                    f'Robot [{self.context.name}] aborted mission with queue '
                    f'id {mission_queue_id}, re-submitting mission with new '
                    f'queue id {new_mission_queue_id}'
                )
            else:
                # If mission is aborted without option to resubmit on abort,
                # mark mission as finished
                self.context.node.get_logger().info(
                    f'Robot [{self.context.name}] has aborted its mission '
                    f'with mission queue id {mission_queue_id}, marking '
                    f'action as completed.'
                )
                return True
        return False

    def check_plc_register(self, register: int):
        # Update register to check if PLC register returns True
        value = self.register_get(register)
        if value:
            self.context.node.get_logger().info(
                f'[{self.context.name}] PLC register {register} detected '
                f'value {value}, robot is ready to move off.'
            )
            return True
        return False

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