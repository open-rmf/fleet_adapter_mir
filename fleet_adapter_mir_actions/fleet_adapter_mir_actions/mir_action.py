
import json
from abc import ABC, abstractmethod
from typing import Callable
import rclpy
import rclpy.node as Node
import rmf_adapter.easy_full_control as rmf_easy
from fleet_adapter_mir.mir_api import MirAPI


class MirAction(ABC):
    def __init__(
            self,
            node,
            name,
            mir_api: MirAPI,
            update_handle,
            fleet_config,
            action_config: dict | None,
    ):
        self.node = node
        self.name = name
        self.api = mir_api
        self.update_handle = update_handle
        self.fleet_config = fleet_config
        self.action_config = action_config
        self.actions = self.action_config.get('actions')

        missions_json = self.action_config.get('missions_json')
        if missions_json:
            with open(missions_json, 'r') as g:
                action_missions = json.load(g)

            # Check if these missions are already created on the robot
            missions_created = True
            for mission_name in action_missions.keys():
                if not mission_name in self.api.known_missions:
                    missions_created = False
                    break
            if missions_created:
                return

            # Create these missions on the robot
            self.api.create_missions(action_missions)
            # Update mission actions stored in MirAPI
            for mission, mission_data in self.api.known_missions.items():
                self.api.mission_actions[mission] = self.api.missions_mission_id_actions_get(mission_data['guid'])

    # This will be called whenever an action has begun
    @abstractmethod
    def perform_action(self,
                       category: str,
                       description: dict,
                       execution  # rmf_fleet_adapter.ActionExecution
        ): 
        # To be populated in the plugins
        ...

    # This will be called on every update to check on the action's current state
    @abstractmethod
    def update_action(self):
        # To be populated in the plugins
        ...

    def cancel_current_task(self,
                            cancel_success: Callable[[], None],
                            cancel_fail: Callable[[], None],
                            label: str = None):
        current_task_id = self.update_handle.more().current_task_id()
        self.node.get_logger().info(f'Cancel task requested for [{current_task_id}]')
        def _on_cancel(result: bool):
            if result:
                self.node.get_logger().info(f'Found task [{current_task_id}], cancelling...')
                cancel_success()
            else:
                self.node.get_logger().info(f'Failed to cancel task [{current_task_id}]')
                cancel_fail()
        self.update_handle.more().cancel_task(current_task_id, [label], lambda result: _on_cancel(result))


class MirActionFactory(ABC):
    def __init__(self):
        pass

    def make_action(self,
                    node: Node,
                    name: str,
                    mir_api: MirAPI,
                    update_handle,  # rmf_fleet_adapter.RobotUpdateHandle
                    fleet_config: rmf_easy.FleetConfiguration,
                    action_config) -> MirAction:
        # To be populated in the plugins
        pass
