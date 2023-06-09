from rmf_fleet_msgs.msg import RobotMode

import math
import time

from .config import Config
from .MiRFleetClientAPI import MirAPI

from fleet_adapter_mir import MiRCommandHandle
from fleet_adapter_mir.MiRCommandHandle import MiRState, MiRPositionTypes, MiRCommandHandle

__all__ = [
    "MiRLocation",
    "MiRState",
    "MiRPositionTypes",
    "MiRCommandHandle",
    #"MiRRetryContext"
]

class MiRFleetCommandHandle(MiRCommandHandle):
    def __init__(self,
                 name,
                 mir_id: str,
                 node,
                 rmf_graph,
                 robot_traits,
                 robot_state_update_frequency=1,
                 dry_run=False):
        MiRCommandHandle.__init__(
            self, name, node, rmf_graph, robot_traits,
            robot_state_update_frequency, dry_run)

        self.mir_id = mir_id
        self.mir_api: MirAPI | None = None  # Ensure MirAPI class is correct

    ##########################################################################
    # INIT METHODS
    ##########################################################################
    def load_mir_maps(self, config: Config):
        if not self.mir_api:
            return
        mir_maps = self.mir_api.maps_get()
        self.rmf_to_mir_maps = {}
        for m in mir_maps:
            rmf_map = config['mirfm']['maps'].get(m['name'], None)
            if rmf_map is None:
                print(f'MiR map {m["name"]} has no mapping to RMF map')
                continue
            self.rmf_to_mir_maps[rmf_map] = m['guid']

    ##########################################################################
    # MIRBASECOMMANDHANDLE OVERLOADS
    ##########################################################################
    def update_mir_map(self, waypoints):
        for wp in waypoints:
            print(f'--- pos = {wp.position}')
            print(f'--- graph index = {wp.graph_index}')

        # FIXME(koonpeng): When exiting from a lift, RMF may instruct the robot to go to a
        # temporary waypoint with no graph index. There is no known way to find the
        # level for this waypoint. This uses a (possibly) hacky solution by looking for the
        # first waypoint with a graph index.
        first_non_none_wp = next(x for x in waypoints if x.graph_index is not None)
        wp_map_name = self.rmf_graph.get_waypoint(first_non_none_wp.graph_index).map_name
        if wp_map_name != self.rmf_map_name:
            print(f'Changing map to {wp_map_name}')
            map_id = self.rmf_to_mir_maps.get(wp_map_name)
            if not map_id:
                print(f'Failed to change map (cannot find corresponding MiR map for {wp_map_name})')
                exit(1)

            # tell MiR the new robot location
            # FIXME(koonpeng): This only works if the MiR maps for each levels are aligned.
            # A proper fix would require us to know the initial robot position on the new
            # map, which RMF doesn't seem to provide. The first waypoint after a map change
            # is the temp lift exit waypoint. Even if we look at the last waypoint of the previous
            # path, we can't find the "connected" waypoint on the new map as vertex properties
            # are not exposed to the python API. Fiducials are also not exposed.
            new_pos, new_rot = self.get_transformation(
                'rmf_to_mir',
                [self.robot_state.location.x, self.robot_state.location.y],
                wp_map_name)
            ori = math.degrees(self.get_position(rmf=True)[2] + new_rot)
            # adapted from https://stackoverflow.com/questions/2320986/easy-way-to-keeping-angles-between-179-and-180-degrees
            ori = ori % 360
            ori = (ori + 360) % 360
            if (ori > 180):
                ori -= 360
            self.mir_api.status_put2({
                'map_id': map_id,
                'position': {
                    'x': new_pos[0],
                    'y': new_pos[1],
                    'orientation': ori,
                }
            })

            self.mir_api.mission_queue_post(self.mir_missions[self.mir_localizse_mission]['guid'])

            self.rmf_map_name = wp_map_name

    def queue_rmf_move_coordinate_mission(self, mir_location):
        MiRCommandHandle.queue_rmf_move_coordinate_mission(mir_location)
        # FIXME(koonpeng): It is possible that the mission has not been
        # queued into MiR yet. If that happens, the state resulting may
        # remain in READY, which causes the adapter to think that the
        # mission has already been completed.
        # This sleep is a hacky workaround to avoid that scenario.
        time.sleep(5)

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
        # FIXME(kp): This has a race condition which cause the condition to be True
        # even if no non-RMF path is made.
        # if not self.rmf_path_requested and robot_mode == RobotMode.MODE_MOVING:
        #     # Unset all
        #     self.rmf_current_lane_index = None
        #     self.rmf_current_waypoint_index = None
        #     self.rmf_target_waypoint_index = None

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
