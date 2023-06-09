import json
import math
import sys
import yaml
import argparse
from base64 import b64encode
from hashlib import sha256
from pprint import pprint

import rclpy
import rclpy.node
from rmf_fleet_msgs.msg import FleetState

import rmf_adapter as adpt
import rmf_adapter.vehicletraits as traits
import rmf_adapter.geometry as geometry
import rmf_adapter.graph as graph
import rmf_adapter.plan as plan

from mir_fleet_client import ApiClient, Configuration
from mir_fleet_client.apis import (
    ChargingGroupsApi,
    MapsApi,
    MissionGroupsApi,
    MissionsApi,
    RobotsApi,
    PositionsApi,
    SettingsApi,
)

import fleet_adapter_mir as mir_adapter

from .config import Config
from .MiRFleetClientAPI import MirFleetAPI
from .MiRFleetCommandHandle import MiRFleetCommandHandle

###############################################################################
# HELPER FUNCTIONS AND CLASSES
###############################################################################

def add_mir_chargers(api_client: ApiClient, config: Config, nav_graph, transforms):
    charging_groups_api = ChargingGroupsApi(api_client)
    robots_api = RobotsApi(api_client)
    positions_api = PositionsApi(api_client)
    maps_api = MapsApi(api_client)

    current_charging_groups: set[int] = set()
    for robot_summary in robots_api.robots_get():
        resp = charging_groups_api.robots_robot_id_charging_groups_get(robot_summary['id'])
        robot_charging_groups = set(x['group_id'] for x in resp)
        if current_charging_groups and current_charging_groups != robot_charging_groups:
            print(f'All robots must have the same charging groups')
            exit(1)
        current_charging_groups = robot_charging_groups

    chargers = []
    for group_id in current_charging_groups:
        chargers.extend(charging_groups_api.charging_groups_group_id_chargers_get(group_id))

    mir_maps = {m.guid: m for m in maps_api.maps_get()}

    for charger in chargers:
        charger_pos = positions_api.positions_guid_get(charger['charger_id'])
        mir_map = mir_maps.get(charger_pos['map_id'], None)
        if mir_map:
            rmf_map_name = config['mirfm']['maps'].get(mir_map['name'], None)
        else:
            rmf_map_name = None
        if rmf_map_name is None:
            print(f'Charger [{charger_pos["name"]}] not in RMF map')
            continue
        mir_to_rmf = transforms[rmf_map_name]['mir_to_rmf']
        wp = nav_graph.add_waypoint(rmf_map_name,
            mir_to_rmf.transform((charger_pos['pos_x'], charger_pos['pos_y'])))
        wp.set_parking_spot(True)
        wp.set_charger(True)
        print(f'Added charger [{charger_pos["name"]}] to RMF')

def assert_homogeneous_fleet(api_client: ApiClient):
    '''
    Checks if the robots in MiR Fleets are all of the same model.
    
    Returns the response from `GET /robots/{id}` if homogeneous.
    Returns `None` if no robot is in MiR Fleet.
    Exits with non-zero code otherwise.
    '''
    robots_api = RobotsApi(api_client)
    robot_model: str | None = None
    robot = None
    for summary in robots_api.robots_get():
        robot = robots_api.robots_id_get(summary['id'])
        if robot_model and robot_model != robot['robot_model']:
            print('Heterogeneous fleet is not supported by RMF and multi fleet is not supported by this adapter')
            exit(1)
    return robot

def create_fleet(config: Config, api_client: ApiClient, nav_graph_path, transforms, mock, dry_run=False):
    """Create RMF Adapter, FleetUpdateHandle, and parse navgraph."""
    if not dry_run:
        robot = assert_homogeneous_fleet(api_client)
    else:
        robot = None

    if 'profile' not in config['rmf_fleet']:
        if robot is None:
            print('Unable to determine vehicle traits. No vehicle profile is provided and no robots is discovered')
            exit(1)
        mir_footprint = json.loads(robot['status']['footprint'])
        center = [sum(x) / len(x) for x in zip(*mir_footprint)]
        longest = 0
        for p in mir_footprint:
            length = math.sqrt((center[0] - p[0]) ** 2 + (center[1] - p[1]) ** 2)
            if length > longest:
                longest = length
        print(f'Automatically detected footprint to have radius {longest}')

        profile = traits.Profile(
            geometry.make_final_convex_circle(longest),
            geometry.make_final_convex_circle(longest * 1.2)
        )
    else:
        profile = mir_adapter.create_profile(config)

    robot_traits = mir_adapter.create_vehicle_traits(config, profile)
    nav_graph = graph.parse_graph(nav_graph_path, robot_traits)

    if not dry_run:
        add_mir_chargers(api_client, config, nav_graph, transforms)

    # RMF_CORE Fleet Adapter: Manages delivery or loop requests
    if mock:
        adapter = adpt.MockAdapter(config['node_names']['rmf_fleet_adapter'])
    else:
        adapter = adpt.Adapter.make(config['node_names']['rmf_fleet_adapter'])
    assert adapter, ("Adapter could not be init! "
                     "Ensure a ROS2 scheduler node is running")

    fleet_name = config['rmf_fleet']['name']
    fleet = mir_adapter.add_fleet(config, adapter, nav_graph)

    # Accept Standard RMF Task which are defined in config.yaml
    fleet = mir_adapter.accept_rmf_tasks(config, fleet)

    # Whether to accept custom RMF action tasks
    def _consider(description: dict):
        confirm = adpt.fleet_update_handle.Confirmation()
        confirm.accept()
        return confirm
    # TODO(AA): To check if the MiR fleet supports these missions names, before
    # confirming.
    # Configure this fleet to perform action category
    fleet = mir_adapter.add_fleet_actions(config, fleet, _consider)

    return adapter, fleet, fleet_name, robot_traits, nav_graph

def mir_auth(username: str, password: str) -> str:
    '''
    Returns the HTTP "Authorization" header to for MiR authentication
    '''
    pw = sha256(password.encode()).hexdigest().encode()
    return 'Basic ' + b64encode(username.encode() + b":" + pw).decode()

def create_robot_command_handles(config: Config, api_client: ApiClient, handle_data, dry_run=False):
    # We need to make calls to MiR Fleet to find the list of robots
    if dry_run:
        return []

    settings_api = SettingsApi(api_client)
    mir_settings = {x['name']: x['value'] for x in settings_api.settings_get()}
    if mir_settings['Auto charging'] == 'true':
        print('!' * 40)
        print(f'Auto charging is enabled in MiR Fleet, it is recommended to disable it to avoid conflicts with RMF.')
        print('!' * 40)
    if mir_settings['Auto staging'] == 'true':
        print('!' * 40)
        print(f'Auto staging is enabled in MiR Fleet, it is recommended to disable it to avoid conflicts with RMF.')
        print('!' * 40)

    robots_api = RobotsApi(api_client)
    robots_summary = robots_api.robots_get()

    maps_api = MapsApi(api_client)
    mir_maps = {m.guid: m for m in maps_api.maps_get()}

    robots = []
    for summary in robots_summary:
        mir_robot = robots_api.robots_id_get(summary['id'])
        mir_robot_status = mir_robot['status']
        robot_name = mir_robot_status['robot_name']

        prefix = f'http://{mir_robot["ip"]}/api/v2.0.0/'
        headers = {}
        headers['Content-Type'] = 'application/json'
        headers['Authorization'] = mir_auth(config['mirfm']['username'], config['mirfm']['password'])

        # CONFIGURE HANDLE ====================================================
        robot = MiRFleetCommandHandle(
                name=robot_name,
                mir_id=summary['id'],
                node=handle_data['node'],
                rmf_graph=handle_data['graph'],
                robot_traits=handle_data['robot_traits'],
                robot_state_update_frequency=config['mirfm'].get('robot_state_update_frequency', 1),
                dry_run=dry_run
        )
        robots.append(robot)
        robot.mir_api = MirFleetAPI(prefix, headers)
        robot.transforms = handle_data['transforms']
        mir_map = mir_maps.get(mir_robot_status['map_id'])
        if mir_map:
            rmf_map = config['mirfm']['maps'][mir_map['name']]
        else:
            rmf_map = None
        if not rmf_map:
            print(f"Cannot determine starting map for [{robot_name}], robot will NOT be added to RMF")
            continue
        robot.rmf_map_name = rmf_map

        if not dry_run:
        #    with MiRRetryContext(robot):
            robot.mir_name = mir_robot_status["robot_name"]

            robot.load_mir_missions()
            robot.load_mir_positions()
            robot.load_mir_maps(config)

            # Check that the MiR fleet has defined the RMF move mission,
            # that this adapter will use repeatedly with varying parameters.
            rmf_move_mission = 'rmf_move'
            assert rmf_move_mission in robot.mir_missions, \
                (f'RMF move mission [{rmf_move_mission}] not yet defined as a mission in MiR')
            robot.mir_rmf_move_mission = rmf_move_mission

            dock_and_charge_mission = 'rmf_dock_and_charge'
            assert dock_and_charge_mission in robot.mir_missions, \
            (f'Dock and charge mission [{dock_and_charge_mission}] not yet defined as a mission in MiR')
            robot.mir_dock_and_charge_mission = dock_and_charge_mission

            localize_mission = 'rmf_localize'
            assert localize_mission in robot.mir_missions, \
                (f'RMF localize mission [{localize_mission}] not yet defined as a mission in MiR')
            robot.mir_localize_mission = localize_mission

        else:
            robot.mir_name = "DUMMY_ROBOT_FOR_DRY_RUN"

        # OBTAIN PLAN STARTS ==================================================
        starts = plan.compute_plan_starts(
            handle_data['graph'],
            robot.rmf_map_name,
            robot.get_position(rmf=True, as_dimensions=True),
            handle_data['adapter'].now(),
            max_merge_waypoint_distance = config['mirfm']['max_merge_waypoint_distance'],
            max_merge_lane_distance = config['mirfm']['max_merge_lane_distance']
        )
        assert starts, ("Robot %s can't be placed on the nav graph!"
                        % robot_name)
        assert len(starts) != 0, (f'No StartSet found for robot: {robot_name}')

        # Insert start data into robot
        start = starts[0]

        if start.lane is not None:  # If the robot is in a lane
            robot.rmf_current_lane_index = start.lane
            robot.rmf_current_waypoint_index = None
            robot.rmf_target_waypoint_index = None
        else:  # Otherwise, the robot is on a waypoint
            robot.rmf_current_lane_index = None
            robot.rmf_current_waypoint_index = start.waypoint
            robot.rmf_target_waypoint_index = None

        print("MAP_NAME:", robot.rmf_map_name)

        handle_data['fleet_handle'].add_robot(
            robot,
            robot.name,
            handle_data['robot_traits'].profile,
            starts,
            lambda rmf_updater: robot.init_updater(rmf_updater))

        handle_data['node'].get_logger().info(
            f"successfully initialized robot {robot.name}"
            f" (MiR name: {robot.mir_name})"
        )

    return robots


def create_missions(api_client: ApiClient, force_create: bool):
    mission_groups_api = MissionGroupsApi(api_client)
    missions_api = MissionsApi(api_client)

    cur_mgs = mission_groups_api.mission_groups_get()
    rmf_mg = next((x for x in cur_mgs if x['name'] == 'RMF'), None)
    if not rmf_mg:
        rmf_mg = mission_groups_api.mission_groups_post({
            'name': 'RMF',
            'priority': 101,  # This is the default when created from UI
            'feature': 'default',
            'icon': '',
        })
    rmf_missions = {x['name']: x for x in mission_groups_api.mission_groups_group_id_missions_get(rmf_mg['guid'])}

    def create_if_not_exist(mission, actions: list):
        mission_name = mission['name']
        existing_mission = rmf_missions.get(mission_name, None)
        if existing_mission and not force_create:
            print(f'[{mission_name}] already exist')
        else:
            if existing_mission:
                missions_api.missions_guid_delete(existing_mission['guid'])
            new_mission = missions_api.missions_post(mission)
            for action in actions:
                action['mission_id'] = new_mission['guid']
                missions_api.missions_mission_id_actions_post(
                    new_mission['guid'], action)
            print(f'Created [{mission_name}] mission')
            return new_mission

    # create move mission
    create_if_not_exist({
        'name': 'rmf_move',
        'description': 'Variable move mission for RMF',
        'hidden': False,
        'group_id': rmf_mg['guid'],
    }, [{
        'action_type': 'move_to_position',
        'priority': 0,
        'parameters': [
            {'id': 'x', 'input_name': 'X', 'value': 0},
            {'id': 'y', 'input_name': 'Y', 'value': 0},
            {'id': 'orientation', 'input_name': 'Orientation', 'value': 0},
            {'id': 'retries', 'value': 10},
            {'id': 'distance_threshold', 'value': 0.1},
        ],
    }])

    # create dock and charge mission
    create_if_not_exist({
        'name': 'rmf_dock_and_charge',
        'description': 'Dock and charge mission for RMF',
        'hidden': False,
        'group_id': rmf_mg['guid']
    }, [{
        'action_type': 'docking',
        'priority': 0,
        'parameters': [
            {'id': 'marker', 'input_name': 'Marker', 'value': None},
            # not sure if we need to set this dynamically and if so how to get the correct type
            {'id': 'marker_type', 'value': 'mirconst-guid-0000-0001-marker000001'},
            {'id': 'retries', 'value': 10},
            {'id': 'max_linear_speed', 'value': 0.25},
        ]
    }, {
        'action_type': 'charging',
        'priority': 0,
        'parameters': [
            {'id': 'minimum_time', 'value': None},
            {'id': 'minimum_percentage', 'value': None},
            {'id': 'charge_until_new_mission', 'value': True},
        ]
    }])

    # create localization mission
    create_if_not_exist({
        'name': 'rmf_localize',
        'description': 'Localization mission for RMF',
        'hidden': False,
        'group_id': rmf_mg['guid']
    }, [{
        'action_type': 'adjust_localization',
        'priority': 0,
        'parameters': [
        ]
    }])

    print('Successfully created RMF missions')


###############################################################################
# MAIN
###############################################################################
def main(argv=sys.argv):

    # INIT RCL ================================================================
    rclpy.init(args=argv)
    adpt.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="fleet_adapter_mir",
        description="Configure and spin up fleet adapters for MiR 100 robots "
                    "that interface between the "
                    "MiR REST API, ROS2, and rmf_core!")
    parser.add_argument("-c", "--config_file", type=str, required=True,
                        help="Input config.yaml file to process")
    parser.add_argument("-n", "--nav_graph", type=str, required=True,
                    help="Path to the nav_graph for this fleet adapter")
    parser.add_argument("-m", "--mock", action='store_true',
                        help="Init a mock adapter instead "
                             "(does not require a schedule node, "
                             "but can interface with the REST API)")
    parser.add_argument("-d", "--dry-run", action='store_true',
                        help="Run as dry run. For testing only. "
                             "Sets mock to True and disables all REST calls.")
    args = parser.parse_args(args_without_ros[1:])
    print(f"Starting mir fleet adapter...")

    config_path = args.config_file
    nav_graph_path = args.nav_graph
    mock = args.mock
    dry_run = args.dry_run  # For testing

    if dry_run:
        mock = True

    with open(config_path, "r") as f:
        config: Config = yaml.safe_load(f)

    mir_adapter.sanitise_dict(config, inplace=True, recursive=True)

    print("\n== Initialising MiR Robot Command Handles with Config ==")
    pprint(config)
    print()

    api_client = ApiClient(Configuration(
        host=config['mirfm']['base_url'],
        username=config['mirfm']['username'],
        password=config['mirfm']['password'],
    ))

    # INIT TRANSFORMS =========================================================
    transforms = {}
    for level, coords in config['reference_coordinates'].items():
        rmf_coordinates = coords['rmf']
        mir_coordinates = coords['mir']
        transforms[level] = mir_adapter.compute_transforms(rmf_coordinates, mir_coordinates)

    # CREATE MISSIONS =========================================================
    create_missions(api_client, config['mirfm'].get('force_create_missions', False))

    # INIT FLEET ==============================================================
    adapter, fleet, fleet_name, robot_traits, nav_graph = create_fleet(
        config, api_client, nav_graph_path, transforms, mock=mock, dry_run=dry_run)

    # INIT ROBOT HANDLES ======================================================
    cmd_node = rclpy.node.Node(config['node_names']['robot_command_handle'])

    handle_data = {
        'fleet_handle': fleet,
        'fleet_name': fleet_name,
        'adapter': adapter,
        'node': cmd_node,
        'graph': nav_graph,
        'robot_traits': robot_traits,
        'transforms': transforms
    }

    robots = create_robot_command_handles(config, api_client, handle_data, dry_run=dry_run)

    # CREATE NODE EXECUTOR ====================================================
    rclpy_executor = rclpy.executors.SingleThreadedExecutor()
    rclpy_executor.add_node(cmd_node)

    # INIT FLEET STATE PUB ====================================================
    if config['rmf_fleet']['publish_fleet_state']:
        fleet_state_node = rclpy.node.Node(
            config['node_names']['fleet_state_publisher'])
        fleet_state_pub = fleet_state_node.create_publisher(
            FleetState,
            config['rmf_fleet']['fleet_state_topic'],
            1
        )
        rclpy_executor.add_node(fleet_state_node)

        def create_fleet_state_pub_fn(fleet_state_pub, fleet_name, robots):
            def f():
                fleet_state = FleetState()
                fleet_state.name = fleet_name

                for robot in robots:
                    fleet_state.robots.append(robot.robot_state)

                fleet_state_pub.publish(fleet_state)
            return f

        fleet_state_timer = fleet_state_node.create_timer(
            config['rmf_fleet']['fleet_state_publish_frequency'],
            create_fleet_state_pub_fn(fleet_state_pub, fleet_name, robots)
        )

    # GO! =====================================================================
    adapter.start()
    rclpy_executor.spin()

    # CLEANUP =================================================================
    cmd_node.destroy_node()

    if config['rmf_fleet']['publish_fleet_state']:
        fleet_state_node.destroy_timer(fleet_state_timer)
        fleet_state_node.destroy_node()

    rclpy_executor.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)