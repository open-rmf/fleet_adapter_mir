
import json
from .mir_api import MirAPI, MirStatus, MiRStateCode


class MirAction:
    def __init__(
            self,
            node,
            name,
            mir_api: MirAPI,
            update_handle,
            actions: list[str],
            missions_json: str | None,
            action_config: dict | None,
    ):
        self.node = node
        self.name = name
        self.api = mir_api
        self.update_handle = update_handle
        self.actions = actions
        self.action_config = action_config

        if missions_json:
            with open(missions_json, 'r') as g:
              action_missions = json.load(g)
            # Create these missions on the robot
            self.api.create_missions(action_missions)
            # Update mission actions stored in MirAPI
            for mission, mission_data in self.api.known_missions.items():
                self.api.mission_actions[mission] = self.api.missions_mission_id_actions_get(mission_data['guid'])

    def update_action(self):
        # To be populated in the plugins
        pass

    def perform_action(self, category, description, execution):
        # To be populated in the plugins
        pass

    def cancel_current_task(self, cancel_success, cancel_fail, label):
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
