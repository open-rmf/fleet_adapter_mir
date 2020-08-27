from rmf_fleet_msgs.msg import Location, RobotMode, RobotState

import rmf_adapter as adpt

from mir100_client.rest import ApiException
from mir100_client.models import PostMissionQueues, PostMissions, \
    PostMissionActions, PutStatus

from collections import namedtuple
import threading
import urllib3
import copy
import enum
import math

__all__ = [
    "MiRLocation",
    "MiRState",
    "MiRPositionTypes",
    "MiRCommandHandle",
    "MiRRetryContext"
]


###############################################################################
# TYPES
###############################################################################

MiRLocation = namedtuple("MiRLocation", ['x', 'y', 'yaw'])


class MiRState(enum.IntEnum):
    READY = 3
    PAUSE = 4
    EXECUTING = 5
    MANUAL_CONTROL = 11
    ERROR = 12


class MiRPositionTypes(enum.IntEnum):
    ROBOT = 0
    CHARGING_STATION = 7
    CHARGING_STATION_ENTRY = 8


###############################################################################
# CLASSES
###############################################################################

class MiRCommandHandle(adpt.RobotCommandHandle):
    def __init__(self,
                 name,
                 node,
                 rmf_graph,
                 robot_state_update_frequency=1,
                 dry_run=False):
        adpt.RobotCommandHandle.__init__(self)

        self.name = name  # Name of robot object in config yaml
        self.node = node
        self.dry_run = dry_run  # For testing only. Disables REST calls.

        self.paused = False
        self.paused_path = []

        # Robot State =========================================================
        self.robot_state = RobotState()
        self.last_robot_state_update = -1
        self.robot_state_update_frequency = robot_state_update_frequency

        # NOTE(CH3): This is a naively monotonically increasing task counter.
        #
        # There is no interface to get the task request message ID!
        # Instead I am just following the behaviour from rmf_core's
        # full_control adapter.
        self.current_task_id = 0

        self.transforms = {'rmf_to_mir': None,
                           'mir_to_rmf': None}

        # RMF Variables =======================================================
        self.rmf_updater = None

        self.rmf_graph = rmf_graph
        self.rmf_lane_dict = {}  # Maps entry, exit to lane index
        self.rmf_map_name = ""

        # NOTE(CH3): This is required for fleet state publishing
        # The path is in reverse order! (i.e. [last, ... first])
        self.rmf_remaining_path = []

        # Populate lane dict
        for i in range(self.rmf_graph.num_lanes):
            graph_lane = self.rmf_graph.get_lane(i)
            id_tuple = (graph_lane.entry.waypoint_index,
                        graph_lane.exit.waypoint_index)

            self.rmf_lane_dict[id_tuple] = graph_lane

        # RMF Location Trackers ===============================================
        self.rmf_current_lane_index = None  # None when moving
        self.rmf_current_waypoint_index = None
        self.rmf_target_waypoint_index = None  # None when not moving

        # RMF Execution Flags =================================================
        self.rmf_docking_executed = False
        self.rmf_docking_requested = False

        self.rmf_path_requested = False

        # MiR Variables =======================================================
        self.mir_name = ""  # Name of robot on MiR REST server
        self.mir_missions = {}  # MiR Mission Name-GUID Dict
        self.mir_positions = {}  # MiR Place Name-GUID Dict
        self.mir_api = None  # MiR REST API
        self.mir_state = MiRState.PAUSE

        # Thread Management ===================================================
        # Path queue execution thread
        self._path_following_thread = None
        self._path_quit_event = threading.Event()
        self._path_quit_cv = threading.Condition()

        # Dock queue execution thread
        self._docking_thread = None
        self._docking_quit_event = threading.Event()
        self._docking_quit_cv = threading.Condition()

        # Start State Update Timer ============================================
        self.state_update_timer = self.node.create_timer(
            self.robot_state_update_frequency,
            self.update_internal_robot_state
        )

    ##########################################################################
    # ROBOTCOMMANDHANDLE OVERLOADS (DO NOT CHANGE METHOD SIGNATURES)
    ##########################################################################
    # Pause and resume are not technically overrides...
    # But it's neater to put them here
    def pause(self):
        """Set pause flag and hold on to any requested paths."""
        self.paused = True

        if self.rmf_remaining_path:
            self.node.get_logger().info(
                '[PAUSE] {self.name}: Current path saved!'
            )

            self.paused_path = self.rmf_remaining_path
            self.rmf_remaining_path = []

    def resume(self):
        """Unset pause flag and substitute paused paths if no paths exist."""
        if self.paused:
            self.paused = False

            if self.rmf_remaining_path:
                return
            elif self.paused_path:
                self.rmf_remaining_path = self.paused_path
                self.paushed_path = []

                self.node.get_logger().info(
                    '[RESUME] {self.name}: Saved path restored!'
                )
        else:
            return

    def stop(self):
        """Stop all path following and docking commands."""
        self.rmf_remaining_path.clear()
        self.rmf_path_requested = False
        self.rmf_target_waypoint_index = None

        self.rmf_docking_requested = False
        self.rmf_docking_executed = False

        if self._path_following_thread is not None:
            self._path_quit_event.set()
            self._path_quit_cv.acquire()
            self._path_quit_cv.notify_all()
            self._path_quit_cv.release()
            self._path_following_thread.join()

        if self._docking_thread is not None:
            self._docking_quit_event.set()
            self._docking_quit_cv.acquire()
            self._docking_quit_cv.notify_all()
            self._docking_quit_cv.release()
            self._docking_thread.join()

        if not self.dry_run:
            self.mir_api.mission_queue_delete()

        old_state = self.mir_state
        self.mir_state = MiRState.PAUSE

        # Prevent repeat and needless logs
        if (old_state != MiRState.PAUSE
                and self.robot_state.mode.mode != RobotMode.MODE_IDLE):
            self.node.get_logger().info(
                '[ABORT] {self.name}: Robot stop called!'
            )

    def follow_new_path(self,
                        waypoints,
                        next_arrival_estimator,  # function!
                        path_finished_callback):
        self.stop()
        self.current_task_id += 1

        self.rmf_path_requested = True
        self.rmf_remaining_path = copy.copy(waypoints)

        # Construct remaining path list
        for waypoint in waypoints[::-1]:
            # Split timestamp into decimal and whole second portions
            _sub_seconds, _seconds = math.modf(waypoint.time.timestamp())

            _msg = Location()
            _msg.x, _msg.y, _msg.yaw = waypoint.position
            _msg.t.sec, _msg.t.nanosec = int(_seconds), int(_sub_seconds * 1e9)

            self.rmf_remaining_path.append(_msg)

        if not self.dry_run:
            status = PutStatus(state_id=MiRState.READY)
            self.mir_api.status_put(status)

        def path_following_closure():
            _current_waypoint = None
            _next_waypoint = None

            # LOOP ============================================================
            # Kept alive if paused
            while ((self.rmf_remaining_path or self.paused)
                    or _current_waypoint):
                if not self.paused:  # Skipped if paused
                    if _current_waypoint is None:
                        _current_waypoint = self.rmf_remaining_path.pop()
                        self.rmf_path_requested = True

                    waypoint_leave_msg = _current_waypoint.t
                    ros_waypoint_leave_time = (
                        waypoint_leave_msg.sec
                        + waypoint_leave_msg.nanosec / 1e9
                    )

                    ros_now = self.node.get_clock().now().nanoseconds / 1e9
                    next_mission_wait = (ros_waypoint_leave_time - ros_now)
                else:
                    # Prevent spinning out of control when paused
                    self._path_quit_cv.acquire()
                    self._path_quit_cv.wait(1)
                    self._path_quit_cv.release()

                # CHECK FOR PRE-EMPT ==========================================
                if self._path_quit_event.is_set():
                    self.stop()
                    self.node.get_logger().info(
                        '[ABORT] {self.name}: Pre-empted path following!'
                    )
                    return

                # EXECUTE NEXT COMMAND ========================================
                # Wait for time to leave and robot to finish moving
                if (next_mission_wait <= 0
                    and self.mir_state == MiRState.READY
                        and not self.paused):  # Skipped if paused

                    # END =====================================================
                    if not self.rmf_remaining_path:  # We're done!
                        self.rmf_path_requested = False

                        path_finished_callback()
                        self.execute_updates()

                        return

                    # ASSIGN NEXT TARGET ======================================
                    else:
                        _next_waypoint = self.rmf_remaining_path[-1]

                        # Grab graph indices
                        if _next_waypoint.graph_index.has_value:
                            _next_index = _next_waypoint.graph_index.value
                        else:
                            _next_index = None

                        if _current_waypoint.graph_index.has_value:
                            _current_index = (
                                _current_waypoint.graph_index.value
                            )
                        else:
                            _current_index = None

                        _current_waypoint = None

                        # Update Internal Location Trackers ===================
                        # [IdleAtWaypoint -> RMF_Move]
                        # [IdleAtLane -> RMF_Move]
                        # [IdleAtUnknown -> RMF_Move]

                        # Set target index
                        self.rmf_target_waypoint_index = _next_index

                        # Infer and set lane index
                        if not self.rmf_current_lane_index:
                            if _current_index is not None:
                                self.rmf_current_lane_index = (
                                    self.lane_dict.get((_current_index,
                                                        _next_index))
                                )

                        # Unset current index
                        self.rmf_current_waypoint_index = None

                    # SEND NEXT TARGET ========================================
                    _mir_pos = self.transforms['rmf_to_mir'].transform(
                        [_next_waypoint.position[0],
                         _next_waypoint.position[1]]
                    )
                    _mir_ori_rad = (
                        math.radians(_next_waypoint.position[2] % 360)
                        + self.transforms['rmf_to_mir'].get_rotation()
                    )

                    # NOTE(CH3): MiR Location is sent in Degrees
                    _mir_ori = math.degrees(_mir_ori_rad % (2 * math.pi))

                    if _mir_ori > 180.0:
                        _mir_ori = _mir_ori - 360.0
                    elif _mir_ori <= -180.0:
                        _mir_ori = _mir_ori + 360.0

                    mir_location = MiRLocation(x=_mir_pos[0],
                                               y=_mir_pos[1],
                                               yaw=_mir_ori)

                    print(f"RMF location x:{_next_waypoint.position[0]}"
                          f"y:{_next_waypoint.position[1]}")
                    print(f'MiR location: {mir_location}')

                    self.queue_move_coordinate_mission(mir_location)
                    self.execute_updates()
                    continue

                if not self.paused:  # Skipped if paused
                    # Prevent spinning out of control
                    if next_mission_wait <= 0:
                        next_mission_wait = 0.1

                    self._path_quit_cv.acquire()
                    self._path_quit_cv.wait(next_mission_wait)
                    self._path_quit_cv.release()

        self._path_quit_event.clear()
        self._path_following_thread = threading.Thread(
            target=path_following_closure
        )
        self._path_following_thread.start()

    def dock(self, dock_name, docking_finished_callback):
        """Start thread to invoke MiR docking mission, then notify rmf_core."""
        self.stop()

        self.current_task_id += 1

        self.rmf_docking_requested = True
        self.rmf_docking_executed = False

        if not self.dry_run:
            status = PutStatus(state_id=MiRState.READY)
            self.mir_api.status_put(status)

        def dock_closure():
            if not self.dry_run:
                self.queue_dock_mission(dock_name)

            # Check for docking complete!
            while self.rmf_docking_requested:
                if not self.dry_run:
                    api_response = self.mir_api.status_get()
                    self.rmf_docking_executed = (
                        'docking' in api_response.mission_text.lower())
                else:
                    api_response = None
                    self.rmf_docking_executed = False

                self.execute_updates(api_response)

                # Docking completed
                if not self.dry_run:
                    if (self.rmf_docking_executed
                            and api_response.state_id == MiRState.READY):
                        self.rmf_docking_requested = False
                        docking_finished_callback()

                        self.node.get_logger().info(
                            '[COMPLETE] Completed dock at: "{dock_name}"!'
                        )
                        return
                else:
                    self.rmf_docking_requested = False
                    self.rmf_docking_executed = True
                    self.node.get_logger().info(
                        '[COMPLETE-DRYRUN] Completed dock at: "{dock_name}"!'
                    )
                    docking_finished_callback()
                    return

                # Docking pre-empted
                if self._docking_quit_event.is_set():
                    self.rmf_docking_requested = False
                    self.rmf_docking_executed = False

                    self.stop()

                    self.node.get_logger().info(
                        '[ABORT] Pre-empted dock at: "{dock_name}"!'
                    )
                    return

                self._docking_quit_cv.acquire()
                self._docking_quit_cv.wait(1)
                self._docking_quit_cv.release()

        self._docking_quit_event.clear()
        self._docking_thread = threading.Thread(target=dock_closure)
        self._docking_thread.start()

    ##########################################################################
    # INIT METHODS
    ##########################################################################
    def load_mir_missions(self):
        if self.dry_run:
            self.node.get_logger().info('{self.name}: DRY_RUN LOAD MISSIONS')
            return

        self.node.get_logger().info('{self.name}: Retrieving MiR Missions...')
        robot_missions_ls = self.mir_api.missions_get()
        for i in robot_missions_ls:
            if i.name not in self.mir_missions:
                self.mir_missions[i.name] = i
            else:
                if "move_coordinate" in i.name:
                    print("removing {}".format(i.name))
                    self.mir_api.missions_guid_delete(i.guid)

        self.node.get_logger().info(
            f'retrieved {len(self.mir_missions)} missions'
        )

    def load_mir_positions(self):
        if self.dry_run:
            self.node.get_logger().info('{self.name}: DRY_RUN LOAD POSITIONS')
            return

        self.node.get_logger().info('{self.name}: Retrieving MiR Positions...')
        count = 0

        for pos in self.mir_api.positions_get():
            if (
                pos.name not in self.mir_positions
                or pos.guid != self.mir_positions[pos.name].guid
            ):
                if (
                    pos.type_id == MiRPositionTypes.ROBOT
                    or pos.type_id == MiRPositionTypes.CHARGING_STATION_ENTRY
                ):
                    self.mir_positions[pos.name] = (
                        self.mir_api.positions_guid_get(pos.guid)
                    )
                    count += 1

        self.node.get_logger().info(f'updated {count} positions')

    ##########################################################################
    # MISSION METHODS
    ##########################################################################
    def queue_move_coordinate_mission(self, mir_location):
        """Add a move mission to the mission queue, creating when needed."""
        mission_name = ('move_coordinate_to'
                        f'_{mir_location.x:.3f}',
                        f'_{mir_location.y:.3f}',
                        f'_{mir_location.yaw:.3f}')

        # Get mission GUID. If missing, create one and save it.
        mission_id = self.missions.get(
            mission_name, self.create_move_coordinate_mission(mir_location)
        )

        # Queue mission
        try:
            mission = PostMissionQueues(mission_id=mission_id)
            self.mir_api.mission_queue_post(mission)
        except KeyError:
            self.node.get_logger().error(
                '{self.name}: No mission to move coordinates to '
                '[{mir_location.x:3f}_{mir_location.y:.3f}]!'
            )

    def create_move_coordinate_mission(self, mir_location, retries=10):
        mission_name = ('move_coordinate_to'
                        f'_{mir_location.x:.3f}',
                        f'_{mir_location.y:.3f}',
                        f'_{mir_location.yaw:.3f}')

        mission = PostMissions(
            group_id='mirconst-guid-0000-0001-missiongroup',
            name=mission_name,
            description='automatically created by mir fleet adapter',
        )
        response = self.mir_api.missions_post(mission)
        action = PostMissionActions(
            action_type='move_to_position',
            mission_id=response.guid,
            parameters=[
                {'id': 'x', 'value': mir_location.x},
                {'id': 'y', 'value': mir_location.y},
                {'id': 'orientation', 'value': mir_location.yaw},
                {'id': 'retries', 'value': retries},
                {'id': 'distance_threshold', 'value': 0.1},
            ],
            priority=1
        )
        self.mir_api.missions_mission_id_actions_post(
            mission_id=response.guid,
            body=action
        )
        self.node.get_logger().info(
            f'{self.name}: '
            f'Created mission to move coordinate to "{mir_location}"'
        )

        # NOTE(CH3): Unsure if I should be doing this
        self.missions[mission_name] = response.guid

        return response.guid

    def queue_dock_mission(self, dock_name):
        """Add a dock mission to the mission queue, creating when needed."""
        mission_name = f'dock_to_{dock_name}'

        # Get mission GUID. If missing, create one and save it.
        mission_id = self.missions.get(mission_name,
                                       self.create_dock_mission(dock_name))

        # Queue mission
        try:
            mission = PostMissionQueues(mission_id=mission_id)
            self.mir_api.mission_queue_post(mission)
        except KeyError:
            self.node.get_logger().error(
                f'{self.name}: No mission to dock to {dock_name}!'
            )

    def create_dock_mission(self, dock_name):
        """Create, POST, and populate MiR docking mission, then save it."""
        mission_name = f'dock_to_{dock_name}'

        mission = PostMissions(
            # mir const, retrieved with GET /mission_groups
            group_id='mirconst-guid-0000-0001-missiongroup',
            name=mission_name,
            description='automatically created by mir fleet handler',
        )
        response = self.mir_api.missions_post(mission)

        action = PostMissionActions(
            action_type='docking',
            mission_id=response.guid,
            parameters=[
                {'id': 'marker', 'value': dock_name},
            ],
            priority=1
        )
        self.mir_api.missions_mission_id_actions_post(
            mission_id=response.guid,
            body=action
        )

        self.node.get_logger().info(
            f'created mission to move and dock to: "{dock_name}"'
        )

        # NOTE(CH3): Unsure if I should be doing this
        self.missions[mission_name] = response.guid

        return response.guid

    ##########################################################################
    # RMF CORE INTERACTION METHODS
    ##########################################################################
    def get_position(self, rmf=True, api_response=None, as_dimensions=False):
        """Get MiR or RMF robot location from the MiR REST API."""
        if api_response is None:
            if not self.dry_run:
                api_response = self.mir_api.status_get()
            else:
                if as_dimensions:
                    return [[0.0], [0.0], [0.0]]
                else:
                    return [0.0, 0.0, 0.0]

        mir_pos = [api_response.position.x, api_response.position.y]
        mir_ori = api_response.position.yaw

        # Output is [x, y, yaw]
        if rmf:
            rmf_pos = self.transforms['mir_to_rmf'].transform(mir_pos)
            rmf_ori = (math.radians(mir_ori % 360)
                       + self.transforms['mir_to_rmf'].get_rotation())
            output = [*rmf_pos, rmf_ori]
        else:
            output = [*mir_pos, mir_ori]

        if as_dimensions:
            return [[x] for x in output]
        else:
            return output

    # Priority...
    # 1. update_position(waypoint, orientation) [At waypoint]
    # 2. update_position(position, lanes) [In transit]
    # 3. update_position(position, target_waypoint) [In transit, unknown lane]
    # 4. update_position(map_name, position) [Lost]
    def update_position(self, api_response=None):
        """Update position using the MiR status location."""
        if api_response is None:
            if not self.dry_run:
                api_response = self.mir_api.status_get()
            else:
                self.rmf_updater.update_position(self.rmf_map_name,
                                                 [0.0, 0.0, 0.0])
                self.node.get_logger().info("[DRYRUN] Updated Position: "
                                            "pos: [0, 0] | ori: [0]")
                return

        mir_pos = [api_response.position.x, api_response.position.y]
        mir_ori = api_response.position.yaw

        rmf_pos = self.transforms['mir_to_rmf'].transform(mir_pos)
        rmf_ori = (math.radians(mir_ori % 360)
                   + self.transforms['mir_to_rmf'].get_rotation())

        rmf_3d_pos = [*rmf_pos, rmf_ori]

        # At waypoint
        # States: (0, 1, 0)
        if self.rmf_current_waypoint_index:
            self.rmf_updater.update_position(self.rmf_current_waypoint_index,
                                             self.rmf_ori)
        # In Transit or Idle in Lane
        # States: (1, 0, 0), (1, 0, 1)
        elif self.rmf_current_lane_index:
            self.rmf_updater.update_position(rmf_3d_pos,
                                             self.rmf_current_lane_index)
        # In Transit, Unknown Lane
        # States: (0, 0, 1)
        elif self.rmf_target_waypoint_index:  # In Unknown Lane
            self.rmf_updater.update_position(rmf_3d_pos,
                                             self.rmf_target_waypoint_index)
        # Lost or MiR Commanded
        # States: (0, 0, 0)
        else:
            self.rmf_updater.update_position(self.rmf_map_name,
                                             rmf_3d_pos)

        self.node.get_logger().info(f"Updated Position: pos: {rmf_pos} | "
                                    f"ori: {rmf_ori}")

    def update_internal_location_trackers(self):
        """Traverses the state machine to help manage robot location."""
        state_tuple = (self.rmf_current_lane_index is not None,
                       self.rmf_current_waypoint_index is not None,
                       self.rmf_target_waypoint_index is not None)

        # In the absence of a state, treat it as paused
        if self.robot_state:
            robot_mode = self.robot_state.mode.mode
        else:
            robot_mode = RobotMode.MODE_PAUSED

        # SEMANTIC STATE INFERENCE AND ADJUSTMENT =============================
        # See docs for more information on the state transitions

        # MiR_Move: Non-RMF Commanded Move-To-Coordinate
        # (0, 1, 0) and (1, 0 ,0) --> (0, 0, 0)
        # When robot is done moving, robot will be IdleAtUnknown
        if not self.rmf_path_requested and robot_mode == RobotMode.MODE_MOVING:
            # Unset all
            self.rmf_current_lane_index = 0
            self.rmf_current_waypoint_index = 0
            self.rmf_target_waypoint_index = 0

        # RMF_ReachedWaypoint -> IdleAtWaypoint
        # Set current to target's value, unset target and lane
        # (0, 0, 1) and (1, 0, 1) --> (0, 1, 0)
        if (state_tuple == (0, 0, 1) or state_tuple == (1, 0, 1)
                and robot_mode == RobotMode.MODE_IDLE
                and not self.rmf_path_requested):
            self.rmf_current_waypoint_index = self.rmf_target_waypoint_index
            self.rmf_target_waypoint_index = None
            self.rmf_current_lane_index = None

        # IdleAtWaypoint/Lane/Unknown -> RMF_Move
        #
        # Defined in self.follow_new_path's path_following_closure
        # and called during path following execution

    def update_internal_robot_state(self, api_response=None):
        """Update internal robot state message. Does not publish!"""

        # NOTE(CH3): You might need to use robot.mir_name depending
        # on whether you want to use the config yaml name or MiR server
        # name whereever the FleetState message is intended to be used
        robot_state = RobotState()  # Temporary msg to avoid race conditions
        robot_state.name = self.name

        # NOTE(CH3): Presuming model here means robot model, not sim model
        robot_state.model = "MiR100"

        if self.dry_run:
            self.robot_state = robot_state
            return

        try:
            if api_response is None:
                api_response = self.mir_api.status_get()

            now_sec, now_ns = math.modf(
                self.node.get_clock().now().seconds_nanoseconds())

            # Populate Location message
            rmf_location = Location()
            rmf_location.x, rmf_location.y, rmf_location.yaw = (
                self.get_position(rmf=True, api_response=api_response)
            )

            rmf_location.level_name = self.rmf_map_name

            # Populate RobotState message
            robot_state.task_id = str(self.current_task_id)
            robot_state.battery_percent = api_response.battery_percentage

            robot_state.location = rmf_location
            robot_state.path = self.rmf_remaining_path
            robot_state.location.t.sec = now_sec
            robot_state.location.t.nanosec = now_ns

            if api_response.mission_text.startswith('Charging'):  # Charging
                robot_state.mode.mode = RobotMode.MODE_CHARGING
                self.mir_state = MiRState.READY
            elif api_response.state_id == MiRState.PAUSE:  # Paused/Pre-empted
                self.pause()
                robot_state.mode.mode = RobotMode.MODE_PAUSED
                self.mir_state = MiRState.PAUSE
            elif (api_response.state_id == MiRState.EXECUTING  # Moving
                  and not api_response.mission_text.startswith('Charging')):
                self.resume()
                robot_state.mode.mode = RobotMode.MODE_MOVING
                self.mir_state = MiRState.EXECUTING
            elif api_response.state_id == MiRState.READY:  # Idle/Moved
                self.resume()
                robot_state.mode.mode = RobotMode.MODE_IDLE
                self.mir_state = MiRState.READY

            if self.rmf_docking_requested:
                if self.rmf_docking_executed:  # Docked
                    if api_response.state_id == MiRState.READY:
                        robot_state.mode.mode = RobotMode.MODE_IDLE
                    else:  # Docking
                        robot_state.mode.mode = RobotMode.MODE_DOCKING

            # Update internal RobotState
            self.robot_state = robot_state

            self.last_robot_state_update = (
                self.node.get_clock().now().nanoseconds / 1e9
            )
        except ApiException as e:
            self.node.get_logger().warn('Exception when calling '
                                        'DefaultApi->status_get: %s\n'
                                        % e)

    ##########################################################################
    # INTERNAL UPDATE LOOP
    ##########################################################################
    def execute_updates(self, api_response=None):
        if api_response is None:
            api_response = self.mir_api.status_get()

        self.update_internal_robot_state(api_response=api_response)
        self.update_internal_location_trackers()
        self.update_position(api_response=api_response)

        self.state_update_timer.reset()


###############################################################################
# HELPER FUNCTIONS AND CLASSES
###############################################################################
class MiRRetryContext():
    """Context to prevent race conditions during robot startup."""
    def __init__(self, robot):
        self.robot = robot
        self.connection_pool_kw = (self.robot
                                   .mir_api
                                   .api_client
                                   .rest_client
                                   .pool_manager.connection_pool_kw)
        self.orig_retries = self.connection_pool_kw.get('retries')

    def __enter__(self):
        retries = urllib3.Retry(10)
        retries.backoff_factor = 1
        retries.status_forcelist = (404,)
        self.connection_pool_kw['retries'] = retries

        return self.robot

    def __exit__(self, exc_type, exc_value, exc_traceback):
        if self.orig_retries is not None:
            self.connection_pool_kw['retries'] = self.orig_retries
        else:
            del self.connection_pool_kw['retries']
