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

from .rmf_move_off import BaseMoveOff
from fleet_adapter_mir.robot_adapter_mir import ActionContext


class MoveOff(BaseMoveOff):
    def __init__(self, context: ActionContext):
        BaseMoveOff.__init__(self, context)

        '''
        This example demonstrates how we can notify the robot to move off when
        a specified MiR mission is complete.
        '''
        self.mission_name = None
        self.mission_queue_id = None
        self.resubmit_on_abort = False

    def verify_signal(self, signal_config: dict) -> bool:
        mission_name = signal_config.get('mission_name')
        if mission_name is None:
            self.context.node.get_logger().error(
                f'[rmf_move_off_on_mission] requires a mission name, but no '
                f'mission name was provided in the action config! '
            )
            return False
        if mission_name not in self.context.api.known_missions:
            self.context.node.get_logger().info(
                f'[rmf_move_off_on_mission] Mission {mission_name} not found '
                f'on robot {self.context.name}!'
            )
            return False

        mission_actions = \
            self.context.api.missions_mission_id_actions_get(
                self.context.api.known_missions[mission_name]['guid']
            )
        if not mission_actions:
            self.context.node.get_logger().info(
                f'[rmf_move_off_on_mission] Mission {mission_name} actions '
                f'not found on robot {self.context.name}!'
            )
            return False

        retry_count = signal_config.get('retry_count')
        if retry_count is not None and \
                (not isinstance(retry_count, int) or retry_count < 0):
            self.context.node.get_logger().error(
                f'[rmf_move_off_on_mission] takes in a retry count, but the '
                f'value provided {retry_count}] is invalid! '
                f'Ignoring provided value and using the default retry count '
                f'of 10.'
            )

        return True

    def begin_waiting(self, signal_config: dict) -> bool:
        self.mission_name = signal_config['mission_name']
        self.resubmit_on_abort = signal_config.get(
            'resubmit_on_abort', False)
        retry_count = signal_config.get('retry_count', 10)

        # Queue the waiting mission for this robot
        count = 0  # we should attempt (retry_count + 1) times in total
        mission_queue_id = None
        while count <= retry_count and not mission_queue_id:
            count += 1
            self.context.node.get_logger().info(
                f'Queueing mission {self.mission_name} for robot '
                f'[{self.context.name}]...'
            )
            try:
                mission_queue_id = \
                    self.context.api.queue_mission_by_name(
                        self.mission_name)
                if mission_queue_id is not None:
                    break
            except Exception as err:
                self.context.node.get_logger().info(
                    f'Failed to queue mission {self.mission_name}: {err}. '
                    f'Retrying...'
                )
            time.sleep(1)

        if not mission_queue_id:
            self.context.node.get_logger().info(
                f'Unable to queue mission {self.mission_name} for robot '
                f'[{self.context.name}]!'
            )
            return False

        self.context.node.get_logger().info(
            f'Mission {self.mission_name} queued for [{self.context.name}] '
            f'with mission queue id {mission_queue_id}. Robot will wait until '
            f'mission {self.mission_name} with mission queue id '
            f'{mission_queue_id} is completed.'
        )

        self.mission_queue_id = mission_queue_id
        return True

    def is_move_off_ready(self) -> bool:
        mission_status = \
            self.context.api.mission_queue_id_get(self.mission_queue_id)
        if (mission_status is not None and
                mission_status['state'] == 'Done'):
            # Mission has completed, we can set move_off to True
            self.context.node.get_logger().info(
                f'Robot [{self.context.name}] has completed its mission '
                f'with mission queue id {self.mission_queue_id}'
            )
            return True

        if (mission_status is not None and
                mission_status['state'] == 'Aborted'):
            if not self.resubmit_on_abort:
                # If mission is aborted without option to resubmit on abort,
                # mark mission as finished
                self.context.node.get_logger().info(
                    f'Robot [{self.context.name}] has aborted its mission '
                    f'with mission queue id {self.mission_queue_id}, marking '
                    f'action as completed.'
                )
                return True

            # Mission aborted for some reason, let's submit the mission
            # again
            new_mission_queue_id = self.context.api.queue_mission_by_name(
                self.mission_name)
            if not new_mission_queue_id:
                # If we didn't successfully post a new mission, we'll
                # try again in the next loop
                return False
            self.context.node.get_logger().info(
                f'Robot [{self.context.name}] aborted mission with queue '
                f'id {self.mission_queue_id}, re-submitting mission with new '
                f'queue id {new_mission_queue_id}'
            )
            self.mission_queue_id = new_mission_queue_id
        return False
