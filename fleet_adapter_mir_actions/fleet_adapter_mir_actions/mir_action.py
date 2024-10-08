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

import json
from abc import ABC, abstractmethod
from typing import Callable
from fleet_adapter_mir.robot_adapter_mir import ActionContext


class MirAction(ABC):
    def __init__(self, context: ActionContext):
        self.context = context

    '''
    This method is called on every update by the robot adapter to monitor the
    progress and completion of the action.
    Return True if the action is completed, else False.
    '''
    @abstractmethod
    def update_action(self):
        # To be populated in the plugins
        ...

    '''
    This method may be used to cancel the current ongoing task.
    '''
    def cancel_current_task(self,
                            cancel_success: Callable[[], None],
                            cancel_fail: Callable[[], None],
                            label: str = ''):
        current_task_id = self.context.update_handle.more().current_task_id()
        self.context.node.get_logger().info(
            f'[{self.context.name}] Cancel task requested for '
            f'[{current_task_id}]')

        def _on_cancel(result: bool):
            if result:
                self.context.node.get_logger().info(
                    f'[{self.context.name}] Found task [{current_task_id}], '
                    f'cancelling...')
                cancel_success()
            else:
                self.context.node.get_logger().info(
                    f'[{self.context.name}] Failed to cancel task '
                    f'[{current_task_id}]')
                cancel_fail()
        self.context.update_handle.more().cancel_task(
            current_task_id, [label], lambda result: _on_cancel(result))


class MirActionFactory(ABC):
    def __init__(self, context: ActionContext):
        self.context = context

        if 'actions' not in context.action_config:
            raise KeyError(
                f'List of supported actions is not provided in the action '
                f'config! Unable to instantiate an ActionFactory.')
        self.actions = context.action_config['actions']

        # Create required MiR missions on the robot
        missions_json = context.action_config.get('missions_json')
        self.create_missions(missions_json)

    '''
    This method can be used to verify whether this MirActionFactory supports
    the configured action.
    '''
    @abstractmethod
    def supports_action(self, category: str) -> bool:
        # To be populated in the plugins
        ...

    '''
    This method creates a MirAction object for the robot adapter to begin and
    interact with an action.
    '''
    @abstractmethod
    def perform_action(
        self,
        category: str,
        description: dict,
        execution,
    ) -> MirAction:
        # To be populated in the plugins
        ...

    '''
    This method may be used to create any MiR missions required for this
    MirAction on the robot from a JSON file.
    '''
    def create_missions(self, missions_json):
        if missions_json is None:
            return

        with open(missions_json, 'r') as g:
            action_missions = json.load(g)

        # Check if these missions are already created on the robot
        missions_created = True
        for mission_name in action_missions.keys():
            if mission_name not in self.context.api.known_missions:
                missions_created = False
                break
        if missions_created:
            return

        # Create these missions on the robot
        self.context.api.create_missions(action_missions)
        # Update mission actions stored in MirAPI
        for mission, mission_data in self.context.api.known_missions.items():
            self.context.api.mission_actions[mission] = \
                self.context.api.missions_mission_id_actions_get(
                    mission_data['guid'])
