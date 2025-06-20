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

import importlib

from fleet_adapter_mir_actions.mir_action import MirAction, MirActionFactory
from fleet_adapter_mir.robot_adapter_mir import ActionContext


class ActionFactory(MirActionFactory):
    def __init__(self, context: ActionContext):
        MirActionFactory.__init__(self, context)

        # Import configured move off plugins
        self.move_off_plugins = {}
        plugins = context.action_config.get('move_off_plugins')
        if plugins is None:
            # If the user did not provide any move off plugins, log a warning
            # to remind users to provide signal type config in any task
            # description they submit
            self.context.node.get_logger().warn(
                f'WaitUntil ActionFactory is instantiated for robot '
                f'[{self.context.name}], but no valid signal plugin has been '
                f'provided in the action config! There will be no move '
                f'off signal available and the robot will wait for the full '
                f'duration of the timeout when this action is triggered.'
            )
            return
        for plugin_name, module in plugins.items():
            try:
                move_off_plugin = importlib.import_module(module)
                self.move_off_plugins[plugin_name] = move_off_plugin
            except ImportError:
                self.context.node.get_logger().warn(
                    f'Unable to import module {module}! Unable to instantiate '
                    f'WaitUntil MirActionFactory.'
                )

        # Validate signal config provided
        signals = context.action_config.get('signals')
        if signals is None:
            # If the user did not provide any valid signal config, log a
            # warning to remind users to provide signal type config in any task
            # description they submit
            self.context.node.get_logger().warn(
                f'WaitUntil ActionFactory is instantiated for robot '
                f'[{self.context.name}], but no valid signals config has been '
                f'provided in the action config! Any move off signal config '
                f'will have to be provided in the task description submitted '
                f'to RMF.'
            )
            return
        for signal_name, signal_config in signals.items():
            plugin_name = signal_config.get('plugin')
            if (plugin_name is None or
                    plugin_name not in self.move_off_plugins):
                raise KeyError(
                    f'No registered plugin found for {signal_name}! '
                    f'Please ensure that the signal config populated is valid. '
                    f'Unable to instantiate WaitUntil MirActionFactory.'
                )
            move_off_plugin = self.move_off_plugins[plugin_name]
            move_off_obj = move_off_plugin.MoveOff(self.context)
            if not move_off_obj.verify_signal(signal_config):
                raise ValueError(
                    f'Invalid signal config provided for {signal_name}! '
                    f'Please ensure that the populated signal config is valid. '
                    f'Unable to instantiate WaitUntil MirActionFactory.'
                )

        # Validate default signal provided
        default_signal = context.action_config.get('default_signal')
        if default_signal is None:
            self.context.node.get_logger().warn(
                f'WaitUntil ActionFactory instantiated for robot '
                f'[{self.context.name}], but no default signal has been '
                f'provided in the action config! If no signal name/config is '
                f'populated in the task description, there will be no move '
                f'off signal available and the robot will wait for the full '
                f'duration of the timeout when this action is triggered.'
            )
            return
        if default_signal not in signals.keys():
            raise ValueError(
                f'User provided a default signal {default_signal} in the '
                f'action config for WaitUntil MirAction, but the signal is '
                f'not configured! Please ensure that the default signal '
                f'points to one of the configured signals. '
                f'Unable to instantiate WaitUntil MirActionFactory.')

    def supports_action(self, category: str) -> bool:
        return category == 'wait_until'

    def perform_action(
        self,
        category: str,
        description: dict,
        execution
    ) -> MirAction:
        if category == 'wait_until':
            return WaitUntil(
                description, execution, self.context, self.move_off_plugins)
        raise ValueError(
            f'Action [{category}] has been called for rmf_wait_until, '
            f'but it is not a supported action!'
        )

class WaitUntil(MirAction):
    def __init__(
        self,
        description: dict,
        execution,
        context: ActionContext,
        move_off_plugins: dict
    ):
        MirAction.__init__(self, context, execution)

        self.update_gap = description.get(
            'update_gap',
            context.action_config.get('update_gap', 60))  # seconds
        self.default_timeout = description.get(
            'default_timeout',
            context.action_config.get('default_timeout', 60))  # seconds
        self.signals = context.action_config.get('signals')
        self.default_signal = context.action_config.get('default_signal')

        self.move_off_cb = \
            self.create_move_off_cb(description, move_off_plugins)
        if self.move_off_cb is None:
            # Insufficient information provided to configure the check move off
            # callback, mark action as completed and continue task
            self.context.node.get_logger().info(
                f'Insufficient information provided to configure the move-'
                f'off behavior for robot [{self.context.name}], cancelling '
                f'task...'
            )
            self.cancel_task(
                label='Move off behavior cannot be configured, unable to '
                      'perform wait until action.'
            )
            return

        self.start_time = self.context.node.get_clock().now().nanoseconds / 1e9
        self.context.node.get_logger().info(
            f'New wait until action with a timeout of {self.default_timeout} '
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
        if now > self.start_time + self.default_timeout:
            self.context.node.get_logger().info(
                f'Robot [{self.context.name}] has completed waiting for '
                f'{self.default_timeout} seconds without move off signal, '
                f'marking action as complete.'
            )
            return True

        # Log the robot waiting every X seconds
        seconds_passed = round(now - self.start_time)
        if seconds_passed % self.update_gap == 0:
            self.context.node.get_logger().info(
                f'{seconds_passed} seconds have passed since robot '
                f'[{self.context.name}] started its waiting action.'
            )

        return False

    def create_move_off_cb(self, description: dict, move_off_plugins: dict):
        signal_config = None
        selected_plugin = None

        # Determine which move off signal config to use. Any config populated
        # in the task description overrides the default config provided in
        # action config.
        if 'signal_name' in description:
            signal_name = description['signal_name']
            if signal_name in self.signals:
                signal_config = self.signals[signal_name]
        elif ('signal_config' in description and
                'plugin' in description['signal_config']):
            signal_config = description['signal_config']
        elif self.default_signal is not None:
            signal_config = self.signals[self.default_signal]
        else:
            # There is no move off signal provided, we will just wait for the
            # duration of the configured timeout
            default_timeout = description.get(
                'default_timeout', self.default_timeout)
            self.context.node.get_logger().info(
                f'No move off signal was configured for [{self.context.name}]'
                f', the robot will begin waiting until the configured timeout '
                f'of [{default_timeout}] seconds.'
            )
            signal_cb = lambda: False
            return signal_cb

        if signal_config is None:
            self.context.node.get_logger().error(
                f'The submitted signal name or config is invalid! Unable to '
                f'use a default signal as none was configured. Please ensure '
                f'that a valid signal name or plugin is provided in the task '
                f'description.'
            )
            return None

        plugin_name = signal_config['plugin']
        selected_plugin = move_off_plugins.get(plugin_name)
        if selected_plugin is None:
            self.context.node.get_logger().error(
                f'The submitted signal plugin {plugin_name} has not been '
                f'configured! Please ensure that a valid and registered plugin '
                f'is provided in the task description.'
            )
            return None

        move_off_obj = selected_plugin.MoveOff(self.context)
        if not move_off_obj.verify_signal(signal_config):
            self.context.node.get_logger().error(
                f'The submitted signal config is invalid! Please resubmit the '
                f'task with a valid signal config.'
            )
            return None
        if not move_off_obj.begin_waiting(signal_config):
            self.context.node.get_logger().error(
                f'[{self.context.name}] failed to configure the signal with '
                f'plugin {plugin_name}! Unable to begin waiting.'
            )
            return None

        self.context.node.get_logger().info(
            f'Configured robot [{self.context.name}] move off behavior: robot '
            f'will wait until the [{plugin_name}] plugin signals that the '
            f'robot is ready to move off.'
        )
        signal_cb = lambda: move_off_obj.is_move_off_ready()
        return signal_cb
