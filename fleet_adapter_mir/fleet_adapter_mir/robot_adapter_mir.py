import math
import numpy as np
from icecream import ic
import enum
import copy
import time
from typing import Any
from dataclasses import dataclass
import json
import datetime
from rmf_adapter.robot_update_handle import Tier
import rmf_adapter.easy_full_control as rmf_easy
import rclpy
import rclpy.node as Node
from rclpy.duration import Duration
from .mir_api import MirAPI, MirStatus, MiRStateCode
from threading import Lock
import importlib


# Parallel processing solution derived from
# https://stackoverflow.com/a/59385935
def parallel(f):
    def run_in_parallel(*args, **kwargs):

        return args[0].event_loop.run_in_executor(
            None, f, *args, **kwargs
        )

    return run_in_parallel


class MissionHandle:
    def __init__(self, execution, docking=False, localize=False, charger=None):
        self.execution = execution
        self.docking = docking
        self.localize = localize
        self.charger = charger
        self.done = False
        # mission_queue_id gets set asynchronously
        self.mission_queue_id = None
        self.do_not_cancel = False
        self.mutex = Lock()
        # Block before beginning the request to guarantee that a call to stop()
        # cannot possibly lock it first
        self.mutex.acquire(blocking=True)

    def set_mission_queue_id(self, mission_queue_id):
        self.mission_queue_id = mission_queue_id
        self.mutex.release()

    @property
    def activity(self):
        # Move the execution reference into a separate variable just in case
        # another thread modifies self.execution while we're still using it.
        execution = self.execution
        if execution is not None and not self.done:
            return execution.identifier
        return None

class RobotAdapterMiR:
    def __init__(
        self,
        name: str,
        rmf_config: rmf_easy.RobotConfiguration,
        mir_config: dict,
        # pickup_config: dict,
        conversions: dict,
        rmf_missions: dict,
        fleet_handle,
        fleet_config,
        plugin_config: dict | None,
        node: Node,
        event_loop,
        debug=False
    ):
        self.name = name
        self.node = node
        self.mission: MissionHandle | None = None
        self.event_loop = event_loop
        self.debug = debug

        self.disconnect = False

        prefix = mir_config['base_url']
        headers = {
            'Content-Type': mir_config['user'],
            'Authorization': mir_config['password'],
        }
        self.api: MirAPI = MirAPI(prefix, headers, conversions, rmf_missions)

        status = self.api.status_get()
        while self.api.status_get() is None:
            status = self.api.status_get()
            time.sleep(0.1)
        self.last_known_status: MirStatus = status
        self.current_map = status.state.map
        self.fleet_config = fleet_config

        self.api.update_known_positions()
        self.api.positions_delete()

        # This must be done after the API is made
        self.lift_positions: dict = conversions.get('lift_positions', {})
        for lift, levels in self.lift_positions.items():
            for level, position in levels.items():
                position_name = position.get('name')
                assert position_name is not None, (
                    f'Missing "name" field for level [{level}] in lift '
                    f'[{lift}] for lift_positions in configuration file'
                )
                known = self.api.known_positions.get(position_name)
                assert known is not None, (
                    f'MiR is missing the position [{position_name}] needed as '
                    f'a lift position on level [{level}] for lift [{lift}]'
                )
        self.nav_issue_ticket = None  # We should only have one issue ticket at a time to manage unsuccessful missions
        self.requested_replan = False
        self.replan_counts = 0

        self.fleet_handle = fleet_handle
        self.update_handle = fleet_handle.add_robot(
            self.name,
            status.state,
            rmf_config,
            self._make_callbacks(),
        )

        self.current_action = None  # Tracks the current ongoing action
        self.plugin_config = plugin_config  # Stores all the configured plugin action configs

    @property
    def activity(self):
        # Move the mission reference into a separate variable just in case
        # another thread modifies self.mission while we're still using it.
        mission = self.mission
        if mission is not None:
            return self.mission.activity
        return None

    async def update_loop(self, period):
        while rclpy.ok():
            now = self.node.get_clock().now()
            next_wakeup = now + Duration(nanoseconds = period * 1e9)
            await self.request_update()
            while self.node.get_clock().now() < next_wakeup:
                time.sleep(0.01)

    @parallel
    def request_update(self):
        # Retrieve the latest MiR status from robot
        status = self.api.status_get()
        if status is None:
            self.disconnect = True
            self.node.get_logger().warn(f'Unable to retrieve status from robot [{self.name}]!')
            return
        if self.disconnect:
            self.node.get_logger().info(f'Robot [{self.name}] connectivity resumed, received status from robot successfully.')
            self.disconnect = False
        more = self.update_handle.more()
        if more is not None:
            more.unstable_debug_positions(self.debug)

        # Update the stored mission status from MiR
        mission = self.mission
        self.update_mission_status(status, mission)

        # Update RMF with the latest RobotState
        if mission is None or (mission.localize and mission.done) or not mission.localize:
            self.update_handle.update(status.state, self.activity)
            self.last_known_status = status
        else:
            self.node.get_logger().info(
                f'Mission is None / Robot is localizing, ignore status update')

        # Update RMF to mark the ActionExecution as finished
        if mission is not None and mission.done:
            self.update_rmf_finished(mission)

        # PerformAction related checks
        if self.current_action:
            if self.current_action.update_action():
                # This means that the action has ended, we can clear the current action object
                self.node.get_logger().info(f'Robot [{self.name}] has completed its current action.')
                self.current_action = None

        # Clear error on updates
        if status is not None and 'errors' in status.response and len(status.response['errors']) > 0:
            self.api.clear_error()
        if status is not None and (status.response['state_id'] == MiRStateCode.PAUSE or status.response['state_id'] == MiRStateCode.ERROR):
            self.api.status_put(MiRStateCode.READY)

    def is_charging(self, status: MirStatus):
        # Note: Not the best way to verify that robot is charging but there's currently no other option
        if status.response.get('mission_text') == "Charging... Waiting for new mission...":
            return True
        return False

    def update_rmf_finished(self, mission: MissionHandle):
        if not mission.execution:
            return
        mission.execution.finished()
        mission.execution = None

    def update_mission_status(self, status: MirStatus, mission: MissionHandle):
        if mission is None or mission.mission_queue_id is None:
            # Either we don't have a mission or we don't know the
            # mission_queue_id yet so just continue as normal for now
            return

        if mission.execution is None:
            # This mission is already finished, so we return early
            return

        mission_status = self.api.mission_queue_id_get(mission.mission_queue_id)
        executing_mission = status.response.get('mission_queue_id')
        if executing_mission == mission.mission_queue_id:
            # The current mission is still being executed, so we don't
            # need to change anything
            if mission.charger is not None and self.is_charging(status):
                self.node.get_logger().info(f'Robot [{self.name}] has begun charging...')
                mission.docking = False
                mission.done = True
            return

        # The mission is not being executed according to the status, so either
        # the mission has finished or it has not started yet. We need a second
        # API call to figure out which it is.
        if mission_status is None or mission_status.get('state') is None:
            # It shouldn't come to here, but we'll prevent any potential crashes
            # with a check here
            return
        if mission_status['state'] == 'Done':
            # The mission has finished so let's trigger execution.finished() and
            # then clear it out
            self.node.get_logger().info(f'MiR [{self.name}] has completed mission {mission.mission_queue_id}, '
                                        f'marking ActionExecution object as finished.')
            mission.docking = False
            mission.done = True
            # If the mission previously issued a ticket, let's resolve that ticket
            if self.nav_issue_ticket is not None:
                msg = {}
                self.nav_issue_ticket.resolve(msg)
                self.nav_issue_ticket = None
                self.node.get_logger().info(f'Issue ticket is resolved!')
                self.replan_counts = 0
        elif mission_status['state'] == 'Aborted':
            # The robot is unable to perform the mission for some reason, so we
            # raise an issue and re-attempt the mission.
            tier = Tier.Error
            category = mission_status['state']
            detail = {
                'mission_queue_id': mission.mission_queue_id,
                'message': 'Mission has been aborted.'
            }
            # Save the issue ticket somewhere so that we can resolve it later
            self.nav_issue_ticket = self.update_handle.more().create_issue(tier, category, detail)
            self.node.get_logger().info(f'Created [{category}] issue ticket for mission queue ID [{mission.mission_queue_id}]')

            # After issuing a ticket, let's request a replan
            mission.done = True
            self.update_handle.more().replan()
            # We keep track of the number of times we are replanning for the same mission
            self.replan_counts += 1
            self.node.get_logger().info(f'[{self.name}] Replan count: {self.replan_counts}')

        # Ensure that robot is charging
        if mission.done and mission.charger is not None and not self.is_charging(status):
            # Charging mission is marked as done but robot is not charging. Let's send
            # the robot to the charger again.
            mission.done = False
            self.update_handle.more().replan()
            self.replan_counts += 1
            self.node.get_logger().info(f'[{self.name}] Replan count: {self.replan_counts}')

    def _make_callbacks(self):
        callbacks = rmf_easy.RobotCallbacks(
            lambda destination, execution: self.navigate(
                destination, execution
            ),
            lambda activity: self.stop(activity),
            lambda category, description, execution: self.perform_action(
                category, description, execution
            )
        )

        callbacks.localize = lambda estimate, execution: self.localize(
            estimate, execution
        )

        return callbacks

    def navigate(self, destination, execution):
        # If the nav command coming in is to bring the robot to a charger, but the robot is
        # already charging at the same charger, we ignore this nav command
        status = self.last_known_status
        if status and self.is_charging(status):
            ignore_charging_cmd = False
            mission = self.mission
            # If the robot's previous mission was a charging mission to the same charger
            if mission and  mission.charger is not None and destination.name == mission.charger:
                ignore_charging_cmd = True
            # If the robot was already charging at the same charger before any missions were
            # sent/stored
            elif destination.map == self.current_map and \
                    (self.dist(destination.position, status.state.position) <
                     self.update_handle.max_merge_waypoint_distance()):
                ignore_charging_cmd = True

            if ignore_charging_cmd:
                self.node.get_logger().info(f'[{self.name}] Received navigation command to dock to '
                                            f'charger {destination.name} when robot is already charging '
                                            f'at the same charger, ignoring command and marking it as '
                                            f'finished.')
                execution.finished()
                return

        # If the robot already has a mission, cancel it
        self.node.get_logger().info(f'[{self.name}] Calling stop for new navigation command.')
        self.request_stop(self.mission)

        # If this is a docking task, send the appropriate docking mission
        if destination.dock is not None:
            dock_json = json.loads(destination.dock)
            if dock_json.get('mission_name') == self.api.dock_and_charge_mission:
                self.mission = MissionHandle(execution, charger=destination.name)
                # Clear the mission queue before requesting dock and charge
                self.node.get_logger().info(f'Clearing mission queue for [{self.name}] before submitting a dock_and_charge mission.')
                self.api.mission_queue_delete()
            else:
                self.mission = MissionHandle(execution)
            mission_name = dock_json.get('mission_name')
            self.node.get_logger().info(f'Requesting [{mission_name}] mission for [{self.name}].')
            self.request_dock(dock_json, self.mission)
            return

        if destination.inside_lift is not None:
            positions_for_lift = self.lift_positions.get(destination.inside_lift.name)
            if positions_for_lift is not None:
                position_info = positions_for_lift.get(destination.map)
                if position_info is not None:
                    position_name = position_info['name']
                    mir_pos = self.api.known_positions.get(position_name)
                    if mir_pos is not None:
                        p_x = mir_pos['pos_x']
                        p_y = mir_pos['pos_y']
                        r = self.last_known_status.response['position']
                        dx = p_x - r['x']
                        dy = p_y - r['y']
                        dist = math.sqrt(dx*dx + dy*dy)
                        tolerance = position_info.get('tolerance', 0.3)
                        if dist < tolerance:
                            # We are already close enough to the point, so we
                            # will just have the robot stop and tell RMF to go
                            # ahead.
                            self.node.get_logger().info(f'[{self.name}] Calling stop in the lift.')
                            self.request_stop(self.mission)
                            self.mission = None
                            execution.finished()
                            return
                        # We are inside the lift but not close enough to the
                        # desired center point, so we will ask the robot to
                        # simply move there
                        self.mission = MissionHandle(execution)
                        self.request_go_to_known_position(position_name, self.mission)
                        return

        # If this is a move to task, send rmf_move mission
        self.mission = MissionHandle(execution)
        self.request_navigate(destination, self.mission)

    @parallel
    def request_navigate(
        self,
        destination,
        mission_handle: MissionHandle
    ):
        mission_queue_id = None
        if destination.name:
            self.node.get_logger().info(f'[{self.name}] is going to MiR position {destination.name}')
            mission_queue_id = self.api.go_to_known_position(destination.name)
        if mission_queue_id is None:
            self.node.get_logger().info(f'[{self.name}] is going to MiR coordinates [{destination.position}]')
            mission_queue_id = self.api.navigate(destination.position)
        mission_handle.set_mission_queue_id(mission_queue_id)

    @parallel
    def request_go_to_known_position(
        self,
        position_name: str,
        mission_handle: MissionHandle
    ):
        mission_queue_id = self.api.go_to_known_position(position_name)
        mission_handle.set_mission_queue_id(mission_queue_id)


    @parallel
    def request_dock(
        self,
        docking_points,
        mission_handle: MissionHandle
    ):
        if not ('description' in docking_points):
            return
        self.node.get_logger().info(f'Requested dock mission for [{self.name}]: {docking_points}')
        end_waypoint = docking_points['description']['end_waypoint']
        if 'start_waypoint' in docking_points['description']:
            start_waypoint = docking_points['description']['start_waypoint']
        else:
            start_waypoint = None
        mission_name = docking_points['mission_name']

        mission_queue_id = self.api.dock(mission_name, start_waypoint, end_waypoint)
        mission_handle.set_mission_queue_id(mission_queue_id)

    def stop(self, activity):
        mission = self.mission
        if mission is not None:
            if mission.docking:
                self.node.get_logger().info(f'Robot is performing docking mission, ignoring stop issued by RMF')
                return
            if mission.execution is not None and activity.is_same(mission.execution.identifier):
                self.node.get_logger().info(f'[{self.name}] Stop requested from RMF!')
                self.request_stop(mission)
                self.mission = None

    @parallel
    def request_stop(self, mission: MissionHandle):
        if mission is not None:
            with mission.mutex:
                if mission.mission_queue_id is not None and not mission.do_not_cancel:
                    self.api.mission_queue_id_delete(mission.mission_queue_id)

    def localize(self, estimate, execution):
        self.mission = MissionHandle(execution, localize=True)
        self.mission.do_not_cancel = True
        self.request_localize(estimate, self.mission)

    @parallel
    def request_localize(self, estimate, mission: MissionHandle):
        count = 0
        mission_queue_id = None
        while count < self.retries and not mission_queue_id:
            count += 1
            try:
                mission_queue_id = self.api.localize(
                    estimate.map, estimate.position, estimate.graph_index
                )
                if mission_queue_id is not None:
                    self.node.get_logger().info(f'Localize mission is successfully queued!')
                    break
            except Exception as err:
                self.node.get_logger().info(
                    f'Failed to localize on map {estimate.map}: {err}. Retrying...'
                )
            time.sleep(1)

        if not mission_queue_id:
            self.node.get_logger().info(
                f'Failed to localize on map {estimate.map}. Maximum localize retries exceeded!'
            )
            mission.execution.finished()
            mission_queue_id = None
            return

        mission.set_mission_queue_id(mission_queue_id)

    def perform_action(self, category, description, execution):
        if self.current_action:
            # Should not reach here, but we log an error anyway
            self.node.get_logger().info(f'Robot is busy with another perform action! Ignoring new action [{category}]')
            execution.finished()
            return

        for plugin_name, config in self.plugin_config.items():
            actions = config['actions']
            if category in actions:
                # Import relevant plugin
                module = config['module']
                plugin = importlib.import_module(module)
                # Create the relevant MirAction
                action_obj = plugin.ActionFactory().make_action(self.node,
                                                                self.name,
                                                                self.api,
                                                                self.update_handle,
                                                                self.fleet_config,
                                                                config)
                # Begin performing the plugin action
                action_obj.perform_action(category, description, execution)
                # Keep track of the current action
                self.current_action = action_obj
                return

        # No relevant perform action found
        self.node.get_logger().info(f'Perform action [{category}] was not configured for this fleet')
        raise NotImplementedError

    def dist(self, A, B):
        assert(len(A) > 1)
        assert(len(B) > 1)
        return math.sqrt((A[0] - B[0])**2 + (A[1] - B[1])**2)
