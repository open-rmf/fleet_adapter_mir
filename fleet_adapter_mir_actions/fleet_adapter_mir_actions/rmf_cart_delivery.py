import math
from icecream import ic
import enum
import numpy as np
from typing import Any
from dataclasses import dataclass
import requests
from urllib.error import HTTPError
from fleet_adapter_mir_actions.mir_action import MirAction, MirActionFactory
from fleet_adapter_mir.mir_api import MirAPI, MirStatus, MiRStateCode


class PickupState(enum.IntEnum):
    PICKUP_INITIALIZED = 0
    PICKUP_ALLOCATED = 1
    MOVE_REQUESTED = 2
    AT_PICKUP = 3
    DOCK_REQUESTED = 4
    DOCK_COMPLETED = 5
    PICKUP_REQUESTED = 6
    PICKUP_SUCCESS = 7
    TASK_CANCELLED = 8


class Mission:
    def __init__(self, queue_id: str, start_time: float):
        self.queue_id = queue_id
        self.start_time = start_time


@dataclass
class Pickup:
    state: PickupState
    pickup_lots: list[str] # contains either a single pickup lot or list of pickup lots
    cart_id: str
    execution: Any
    mission: Mission
    latching: bool


@dataclass
class Dropoff:
    execution: Any
    mission: Mission


class ActionFactory(MirActionFactory):
    def make_action(self,
                    node,
                    name,
                    mir_api,
                    update_handle,
                    fleet_config,
                    action_config) -> MirAction:
        return CartDelivery(node, name, mir_api, update_handle, fleet_config, action_config)


class CartDelivery(MirAction):
    def __init__(
            self,
            node,
            name,
            mir_api,
            update_handle,
            fleet_config,
            action_config
        ):
        MirAction.__init__(self, node, name, mir_api, update_handle, fleet_config, action_config)

        self.pickup: Pickup = None
        self.dropoff: Dropoff = None
        self.search_timeout = self.action_config.get('search_timeout', 60)  # seconds

        # Store the mission names to be used during the action
        self.dock_to_cart_mission = self.action_config['missions']['dock_to_cart']
        self.pickup_mission = self.action_config['missions']['pickup']
        self.dropoff_mission = self.action_config['missions']['dropoff']
        self.exit_mission = self.action_config['missions']['exit_lot']
        self.footprint_mission = self.action_config['missions']['update_footprint']

        # Initialize footprints and marker types
        self.robot_footprint_guid = self.api.footprints_guid_get(self.action_config['footprints']['robot'])
        self.cart_footprint_guid = self.api.footprints_guid_get(self.action_config['footprints']['cart'])
        self.cart_marker_type_guid = self.api.docking_offsets_guid_get(self.action_config['marker_types']['cart'])
        self.update_footprint(self.robot_footprint_guid)

    def perform_action(self, category, description, execution):
        # Check that the perform action category matches
        match category:
            case 'delivery_pickup':
                # Check if the robot's latch is currently open
                if self.is_latch_open():
                    # Latch is open, unable to perform pickup
                    self.node.get_logger().info(f'Robot [{self.name}] latch is open, unable to perform pickup, cancelling task...')
                    self.cancel_task()
                    return

                self.node.get_logger().info(f'New pickup requested for robot [{self.name}]')
                self.pickup = Pickup(
                    state=PickupState.AT_PICKUP,
                    pickup_lots=[description.get('pickup_lot')],  # TODO(XY): Allow multiple pickups?
                    cart_id=description.get('cart_id'),
                    execution=execution,
                    mission=None,
                    latching=False
                )
            case 'delivery_dropoff':
                self.node.get_logger().info(f'Received dropoff request!')
                self.dropoff = Dropoff(
                    execution=execution,
                    mission=None
                )
            case _:
                self.node.get_logger().info(f'Invalid perform action [{category}] passed to CartDelivery, ending action.')
                execution.finished()

    def update_action(self):
        # There should not be a pickup and dropoff being performed simultaneously at any given time, since actions are
        # dispatched sequentially
        if self.pickup:
            return self.update_pickup(self.pickup)
        elif self.dropoff:
            return self.update_dropoff(self.dropoff)

        return False

    def update_pickup(self, pickup: Pickup):
        # If this is a PerformAction pickup, check that the action is underway and valid
        if pickup.execution is not None and not pickup.execution.okay():
            self.node.get_logger().info(f'[delivery_pickup] action is killed/canceled.')
            pickup.state = PickupState.TASK_CANCELLED

        # Start state machine check
        now = self.node.get_clock().now().nanoseconds / 1e9
        self.node.get_logger().debug(f'PickupState: {pickup.state.name}')
        match pickup.state:
            case PickupState.AT_PICKUP:
                # Send rmf_dock_to_cart mission
                assert self.dock_to_cart_mission is not None
                current_wp_name = pickup.pickup_lots[0]
                cart_marker_guid = self.api.known_positions[current_wp_name]['guid']
                cart_marker_param = self.api.get_mission_params_with_value(self.dock_to_cart_mission, 'docking', 'cart_marker', cart_marker_guid)
                marker_type_param = self.api.get_mission_params_with_value(self.dock_to_cart_mission, 'docking', 'cart_marker_type', self.cart_marker_type_guid)
                mission_params = cart_marker_param + marker_type_param
                self.update_footprint(self.robot_footprint_guid)
                mission_queue_id = self.api.queue_mission_by_name(self.dock_to_cart_mission, mission_params)
                if not mission_queue_id:
                    error_str = f'Mission {self.dock_to_cart_mission} not supported, ignoring'
                    self.node.get_logger().error(error_str)
                    return
                pickup.mission = Mission(mission_queue_id, now)
                pickup.state = PickupState.DOCK_REQUESTED
                self.node.get_logger().info(f'Dock to cart mission requested with mission queue id {mission_queue_id}')

            case PickupState.DOCK_REQUESTED:
                # Make sure that there is an rmf_dock_to_cart mission issued
                if not pickup.mission:
                    self.node.get_logger().info(f'Robot [{self.name}] is indicated to be at the DOCK_REQUESTED state but '
                                                f'no mission queue ID stored for this docking mission! Returning to AT_PICKUP state.')
                    pickup.state = PickupState.AT_PICKUP
                    return
                # Mission completed, move onto the next state
                if self.api.mission_completed(pickup.mission.queue_id):
                    self.node.get_logger().info(f'Robot [{self.name}] dock to cart mission {pickup.mission.queue_id} completed or timed out.')
                    pickup.mission = None
                    pickup.state = PickupState.DOCK_COMPLETED
                    return
                # Mission not yet completed, we check the timeout status to decide if we need to publish any alert
                seconds_passed = now - pickup.mission.start_time
                # Publish update every 10 seconds just to monitor
                if round(seconds_passed)%10 == 0:
                    self.node.get_logger().info(f'{round(seconds_passed)} seconds have passed since pickup mission requested.')
                # Mission timeout, cart not found
                if seconds_passed > self.search_timeout:
                    # Delete mission from mir first
                    self.api.mission_queue_id_delete(pickup.mission.queue_id)
                    # Regardless of whether the robot completed docking properly, we move to the next state to check
                    self.node.get_logger().info(f'Robot [{self.name}] dock to cart mission {pickup.mission.queue_id} timed out! '
                                                f'Configured search timeout is {self.search_timeout} seconds.')
                    pickup.mission = None
                    pickup.state = PickupState.DOCK_COMPLETED
                return

            case PickupState.DOCK_COMPLETED:
                # Check if robot docked under the correct cart
                cart_check = self.is_correct_cart(pickup.cart_id)
                if cart_check:
                    # If cart is correct, send pickup mission
                    assert self.pickup_mission is not None
                    mission_queue_id = self.api.queue_mission_by_name(self.pickup_mission)
                    if not mission_queue_id:
                        error_str = f'Mission {self.pickup_mission} not supported, ignoring'
                        self.node.get_logger().error(error_str)
                        return
                    pickup.mission = Mission(mission_queue_id, now)
                    pickup.latching = True
                    self.node.get_logger().info(f'Robot [{self.name}] found the correct cart, pickup mission requested with mission queue id {mission_queue_id}')
                    pickup.state = PickupState.PICKUP_REQUESTED
                elif cart_check is None:
                    # If cart is missing, cancel this task
                    self.node.get_logger().info(f'Robot [{self.name}] was unable to dock under any carts, please check that cart is present. Cancelling task.')
                    self.cancel_task()
                    pickup.state = PickupState.TASK_CANCELLED
                else:
                    # If cart is wrong, cancel this task also but after we exit from the lot
                    self.node.get_logger().info(f'Robot [{self.name}] found the wrong cart, exiting lot and cancelling task.')
                    self.exit_lot()
                    self.cancel_task()
                    pickup.state = PickupState.TASK_CANCELLED

            case PickupState.PICKUP_REQUESTED:
                # Pickup mission completed
                if self.api.mission_completed(pickup.mission.queue_id):
                    pickup.state = PickupState.PICKUP_SUCCESS
                    pickup.mission = None
                    pickup.latching = False
                    # Update robot footprint to accommodate the cart size
                    self.update_footprint(self.cart_footprint_guid)
                # TODO(XY) consider doing a check for whether the latching time exceeds a configurable timeout

            case PickupState.PICKUP_SUCCESS:
                # Correct ID, we can end the delivery now
                self.node.get_logger().info(f'Robot [{self.name}] successfully received cart, exiting lot with cart and ending mission')
                self.exit_lot()
                if pickup.execution is not None:
                    pickup.execution.finished()
                return True

            case PickupState.TASK_CANCELLED:
                self.node.get_logger().info(f'Robot [{self.name}] is in pickup cancelled state.')
                # If some MiR mission is in progress, we abort it unless it is latching
                if pickup.mission and not self.api.mission_completed(pickup.mission.queue_id):
                    if pickup.latching:
                        self.node.get_logger().info(f'Robot [{self.name}] is performing latching, cancelling task after this action is complete.')
                        return False
                    self.api.mission_queue_id_delete(pickup.mission.queue_id)
                    pickup.mission = None
                # Clear any errors
                self.api.clear_error()
                self.api.status_put(state_id=MiRStateCode.READY)
                self.release_cart()
                # Mark pickup session as completed
                return True

        return False

    def update_dropoff(self, dropoff: Dropoff):
        # Check if action is underway or cancelled
        if dropoff.execution is not None and not dropoff.execution.okay():
            self.node.get_logger().info(f'Dropoff action is killed/canceled')

            # If cart release is in progress, we let it finish first
            if dropoff.mission and not self.api.mission_completed(dropoff.mission.queue_id):
                return False

            # Task is cancelled and cart is done dropping off/mission is not yet queued anyway,
            # we can mark it as completed at this point
            self.api.clear_error()
            self.api.status_put(state_id=MiRStateCode.READY)
            # Mark dropoff session as completed
            return True

        # No mission queued yet
        if dropoff.mission is None:
            assert self.dropoff_mission is not None
            mission_queue_id = self.api.queue_mission_by_name(self.dropoff_mission)
            if not mission_queue_id:
                error_str = f'Mission {self.dropoff_mission} not supported, ignoring'
                self.node.get_logger().error(error_str)
                return True
            self.node.get_logger().info(f'Mission {self.dropoff_mission} added to queue for robot [{self.name}].')
            now = self.node.get_clock().now().nanoseconds / 1e9
            dropoff.mission = Mission(mission_queue_id,
                                      now)
            self.node.get_logger().info(f'[{self.name}] Dropoff mission requested with mission queue id {mission_queue_id}')

        # Mission queued, check completion
        else:
            if self.api.mission_completed(dropoff.mission.queue_id):
                self.node.get_logger().info(f'[{self.name}] Dropoff mission completed!')
                # Update robot footprint
                self.update_footprint(self.robot_footprint_guid)
                if dropoff.execution is not None:
                    dropoff.execution.finished()
                return True
            else:
                self.node.get_logger().info(f'[{self.name}] Dropoff mission in progress...')
        return

    def cancel_task(self, label: str = None):
        def _cancel_success():
            if self.pickup:
                self.pickup.state = PickupState.TASK_CANCELLED
        def _cancel_fail():
            pass
        self.cancel_current_task(_cancel_success, _cancel_fail, label)

    def is_latch_open(self):
        # Checks if the robot's latch is open and carrying a cart
        # Return True if latch is open, else False
        # ------------------------
        # IMPLEMENT YOUR CODE HERE
        # ------------------------
        return False

    def is_under_cart(self):
        # Checks if the robot is docked under a cart
        # Return True if robot is under any carts, else False
        # ------------------------
        # IMPLEMENT YOUR CODE HERE
        # ------------------------
        return False

    def is_correct_cart(self, cart_id: str):
        # Checks if the detected cart identifier matches the target cart_id
        # Return True if cart is correct, False if cart is wrong, None if no cart detected
        # ------------------------
        # IMPLEMENT YOUR CODE HERE
        # ------------------------
        return None

    def exit_lot(self):
        if not self.api.connected:
            return None
        # Set footprint to robot footprint before exiting lot
        self.update_footprint(self.robot_footprint_guid)
        return self.api.queue_mission_by_name(self.exit_mission)

    def release_cart(self):
        # If robot latch is open, close it
        if self.is_latch_open():
            self.node.get_logger().info(f'Robot [{self.name}] detected to have latch open, dropping off cart...')
            self.dropoff = Dropoff(
                execution=None,
                mission_queue_id=None)
            # Dropoff mission will take care of the lot exit, so we can return here
            return
        # If robot is under a cart, exit lot
        if self.is_under_cart():
            self.node.get_logger().info(f'Cart detected above robot [{self.name}] during a release call, exiting lot...')
            self.exit_lot()

    def update_footprint(self, ft_guid: str):
        ft_params = self.api.get_mission_params_with_value(self.footprint_mission,
                                                           'set_footprint',
                                                           'footprint',
                                                           ft_guid)
        return self.api.queue_mission_by_name(self.footprint_mission, ft_params)

    def retrieve_mir_coordinates(self, waypoint_name: str):
        transform = self.fleet_config.transformations_to_robot_coordinates
        transform_current_map = transform.get(self.current_map)
        rmf_pose = self.fleet_config.graph.find_waypoint(waypoint_name).location
        new_rmf_pose = np.array([rmf_pose[0], rmf_pose[1], 0.0])
        mir_pose = transform_current_map.apply(new_rmf_pose)
        return mir_pose

    def dist(self, A, B):
        assert(len(A) > 1)
        assert(len(B) > 1)
        return math.sqrt((A[0] - B[0])**2 + (A[1] - B[1])**2)


    # ------------------------------------------------------------------------------------------------------------------
    # MIR API FOR CART RELATED MISSIONS
    # ------------------------------------------------------------------------------------------------------------------

    def register_get(self, register: int):
        if not self.api.connected:
            return None
        try:
            response = requests.get(self.api.prefix + f'registers/{register}', headers = self.api.headers, timeout = self.api.timeout)
            if self.api.debug:
                print(f"Response: {response.headers}")
            # Response value is string, return integer of value
            return int(response.json().get('value', 0))
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
            return None
        except Exception as err:
            print(f"Other  error: {err}")
            return None

    def io_module_guid_status_get(self, io_guid: str):
        if not self.api.connected:
            return None
        if io_guid is None:
            return None
        try:
            response = requests.get(self.api.prefix + f'io_modules/{io_guid}/status', headers = self.api.headers, timeout = self.api.timeout)
            if self.api.debug:
                print(f"Response: {response.headers}")
            if 'input_state' not in response.json():
                return None
            return response.json()['input_state']
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
            return None
        except Exception as err:
            print(f"Other  error: {err}")
            return None

    def io_modules_get(self):
        if not self.api.connected:
            return None
        try:
            response = requests.get(self.api.prefix + f'io_modules', headers = self.api.headers, timeout = self.api.timeout)
            if self.api.debug:
                print(f"Response: {response.headers}")
            # Response value is string, return integer of value
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
            return None
        except Exception as err:
            print(f"Other  error: {err}")
            return None
