from fleet_adapter_mir import MiRCommandHandle
from MiRClientAPI import MirAPI

from rmf_fleet_msgs.msg import FleetState
from rmf_task_msgs.msg import TaskProfile, TaskType
import rclpy.node
import rclpy

import rmf_adapter as adpt
import rmf_adapter.vehicletraits as traits
import rmf_adapter.battery as battery
import rmf_adapter.geometry as geometry
import rmf_adapter.graph as graph
import rmf_adapter.plan as plan

from functools import partial
from pprint import pprint
import sys
import argparse
import nudged
import yaml


###############################################################################
# HELPER FUNCTIONS AND CLASSES
###############################################################################
def sanitise_dict(dictionary, inplace=False, recursive=False):
    """Remove dictionary Nones."""
    if type(dictionary) is not dict:
        return dictionary

    output = {}

    if inplace:
        del_keys = set()

        for key, val in dictionary.items():
            if val is None:
                del_keys.add(key)
            elif recursive:
                sanitise_dict(dictionary[key], True, True)

        for key in del_keys:
            del dictionary[key]

        output = dictionary
    else:
        for key, val in dictionary.items():
            if val is None:
                continue
            else:
                if recursive:
                    val = sanitise_dict(val, False, True)
                output[key] = val

    return output


def compute_transforms(rmf_coordinates, mir_coordinates, node=None):
    """Get transforms between RMF and MIR coordinates."""
    transforms = {
        'rmf_to_mir': nudged.estimate(rmf_coordinates, mir_coordinates),
        'mir_to_rmf': nudged.estimate(mir_coordinates, rmf_coordinates)
    }

    if node:
        mse = nudged.estimate_error(transforms['rmf_to_mir'],
                                    rmf_coordinates,
                                    mir_coordinates)

        node.get_logger().info(f"Transformation estimate error: {mse}")
    return transforms


def create_fleet(config,nav_graph_path,task_request_check, mock):
    """Create RMF Adapter, FleetUpdateHandle, and parse navgraph."""
    profile = traits.Profile(
        geometry.make_final_convex_circle(config['rmf_fleet']['profile']['footprint']),
        geometry.make_final_convex_circle(config['rmf_fleet']['profile']['vicinity'])
    )
    robot_traits = traits.VehicleTraits(
        linear=traits.Limits(*config['rmf_fleet']['limits']['linear']),
        angular=traits.Limits(*config['rmf_fleet']['limits']['angular']),
        profile=profile
    )
    robot_traits.differential.reversible = config['rmf_fleet']['reversible']

    voltage = config['rmf_fleet']['battery_system']['voltage']
    capacity = config['rmf_fleet']['battery_system']['capacity']
    charging_current = config['rmf_fleet']['battery_system']['charging_current']
    battery_sys = battery.BatterySystem.make(voltage,capacity,charging_current)

    mass = config['rmf_fleet']['mechanical_system']['mass']
    moment = config['rmf_fleet']['mechanical_system']['moment_of_inertia']
    friction = config['rmf_fleet']['mechanical_system']['friction_coefficient']
    mech_sys = battery.MechanicalSystem.make(mass,moment,friction)

    ambient_power_sys = battery.PowerSystem.make(
        config['rmf_fleet']['ambient_system']['power'])
    tool_power_sys = battery.PowerSystem.make(
        config['rmf_fleet']['cleaning_system']['power'])
    motion_sink = battery.SimpleMotionPowerSink(battery_sys,mech_sys)
    ambient_sink = battery.SimpleDevicePowerSink(battery_sys, ambient_power_sys)
    

    nav_graph = graph.parse_graph(nav_graph_path, robot_traits)

    # RMF_CORE Fleet Adapter: Manages delivery or loop requests
    if mock:
        adapter = adpt.MockAdapter(config['node_names']['rmf_fleet_adapter'])
    else:
        adapter = adpt.Adapter.make(config['node_names']['rmf_fleet_adapter'])

    assert adapter, ("Adapter could not be init! "
                     "Ensure a ROS2 scheduler node is running")

    fleet_name = config['rmf_fleet']['name']
    fleet = adapter.add_fleet(fleet_name, robot_traits, nav_graph)

    if config['rmf_fleet']['publish_fleet_state']:
        fleet.fleet_state_publish_period(None)
    
    drain_battery = config['rmf_fleet']['account_for_battery_drain']
    recharge_threshold = config['rmf_fleet']['recharge_threshold']
    recharge_soc = config['rmf_fleet']['recharge_soc']

    if task_request_check is None:
        # Naively accept all delivery requests
        fleet.accept_task_requests(lambda x: True)
    else:
        fleet.accept_task_requests(task_request_check)

    return adapter, fleet, fleet_name, robot_traits, nav_graph

def create_robot_command_handles(config, handle_data, dry_run=False):
    robots = {}

    for robot_name, robot_config in config['robots'].items():
        # CONFIGURE MIR =======================================================
        mir_config = robot_config['mir_config']
        rmf_config = robot_config['rmf_config']

        prefix = mir_config['base_url']
        headers = {}
        headers['Content-Type']  = mir_config['user']
        headers['Authorization'] = mir_config['password']

        # CONFIGURE HANDLE ====================================================
        robot = MiRCommandHandle(
                name=robot_name,
                node=handle_data['node'],
                rmf_graph=handle_data['graph'],
                robot_traits=handle_data['robot_traits'],
                robot_state_update_frequency=rmf_config.get(
                'robot_state_update_frequency', 1),
            dry_run=dry_run
        )
        robot.mir_api =  MirAPI(prefix,headers)
        robot.transforms = handle_data['transforms']
        robot.rmf_map_name = rmf_config['start']['map_name']

        if not dry_run:
        #    with MiRRetryContext(robot):
            _mir_status = robot.mir_api.status_get()
            robot.mir_name = _mir_status["robot_name"]

            robot.load_mir_missions()
            robot.load_mir_positions()
        else:
            robot.mir_name = "DUMMY_ROBOT_FOR_DRY_RUN"

        robots[robot.name] = robot

        # OBTAIN PLAN STARTS ==================================================
        start_config = rmf_config['start']

        # If the plan start is configured, use it, otherwise, grab it
        waypoint_index = start_config.get('waypoint_index')
        orientation = start_config.get('orientation')
        if (waypoint_index is not None) and (orientation is not None):
            pprint(type(handle_data['adapter']))
            starts = [plan.Start(handle_data['adapter'].now(),
                                 start_config.get('waypoint_index'),
                                 start_config.get('orientation'))]
        else:
            starts = plan.compute_plan_starts(
                handle_data['graph'],
                start_config['map_name'],
                robot.get_position(rmf=True, as_dimensions=True),
                handle_data['adapter'].now()
            )
        assert starts, ("Robot %s can't be placed on the nav graph!"
                        % robot_name)

        # Insert start data into robot
        start = starts[0]

        if start.lane is not None:  # If the robot is in a lane
            robot.rmf_current_lane_index = start.lane.value
            robot.rmf_current_waypoint_index = None
            robot.rmf_target_waypoint_index = None
        else:  # Otherwise, the robot is on a waypoint
            robot.rmf_current_lane_index = None
            robot.rmf_current_waypoint_index = start.waypoint
            robot.rmf_target_waypoint_index = None

        print("MAP_NAME:", start_config['map_name'])
        robot.rmf_map_name = start_config['map_name']

        # INSERT UPDATER ======================================================
        def updater_inserter(handle_obj, rmf_updater):
            """Insert a RobotUpdateHandle."""
            handle_obj.rmf_updater = rmf_updater

        handle_data['fleet_handle'].add_robot(
            robot,
            robot.name,
            handle_data['robot_traits'].profile,
            starts,
            partial(updater_inserter, robot)
        )

        handle_data['node'].get_logger().info(
            f"successfully initialized robot {robot.name}"
            f" (MiR name: {robot.mir_name})"
        )
    return robots

###############################################################################
# MAIN
###############################################################################
def main(argv=sys.argv, task_request_check=None, mock=False):

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
        config = yaml.safe_load(f)

    sanitise_dict(config, inplace=True, recursive=True)

    print("\n== Initialising MiR Robot Command Handles with Config ==")
    pprint(config)
    print()

    # INIT FLEET ==============================================================
    adapter, fleet, fleet_name, robot_traits, nav_graph = create_fleet(config,nav_graph_path,task_request_check=task_request_check,
                                                                  mock=mock)

    # INIT TRANSFORMS =========================================================
    rmf_coordinates = config['reference_coordinates']['rmf']
    mir_coordinates = config['reference_coordinates']['mir']
    transforms = compute_transforms(rmf_coordinates, mir_coordinates)

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

    robots = create_robot_command_handles(config, handle_data, dry_run=dry_run)

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

                for robot in robots.values():
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
    # Configure task condition ============================================
    # Return True if the loop task request should be honoured
    # False otherwise
    def task_request_check(msg: TaskProfile):
        if (msg.description.task_type == TaskType.TYPE_LOOP):
            return True
        else:
            return False

    main(sys.argv,
           task_request_check=task_request_check)
