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

import enum
from dataclasses import dataclass
import importlib
from fleet_adapter_mir_actions.mir_action import MirAction, MirActionFactory
from fleet_adapter_mir.robot_adapter_mir import ActionContext
from fleet_adapter_mir.mir_api import MiRStateCode


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
    pickup_lots: list[str]  # contains list of pickup lots
    cart_id: str
    mission: Mission
    latching: bool


@dataclass
class Dropoff:
    mission: Mission


class ActionFactory(MirActionFactory):
    def __init__(self, action_config: dict):
        MirActionFactory.__init__(action_config)
        # Raise error if config file is invalid
        # Note(@xiyuoh) Using if-else to check for valid keys in the action
        #               config is not the most scalable. Consider other ways to
        #               do this.
        if 'cart_detection_module' not in action_config:
            raise KeyError(
                f'CartDelivery MirAction requires a cart detection module, but '
                f'path to [cart_detection_module] is not provided in the '
                f'action config! Unable to instantiate an ActionFactory.')
        elif 'marker_types' not in action_config:
            raise KeyError(
                f'CartDelivery MirAction requires configured marker types, but '
                f'[marker_types] not provided in the action config! '
                f'Unable to instantiate an ActionFactory.')
        elif 'missions' not in action_config:
            raise KeyError(
                f'CartDelivery MirAction requires configured missions, but '
                f'[missions] not provided in the action config! '
                f'Unable to instantiate an ActionFactory.')
        # Check if the required mission names have been configured
        # TODO(@xiyuoh) Remove requirement for configuring dock_to_cart and
        #               exit lot as they will be created on the robot by the
        #               fleet adapter, hence will not need to be configured
        elif 'dock_to_cart' not in action_config['missions']:
            raise KeyError(
                f'CartDelivery MirAction requires the configured MiR mission '
                f'name for [dock_to_cart], but it is not provided in the action '
                f'config! Unable to instantiate an ActionFactory.')
        elif 'pickup' not in action_config['missions']:
            raise KeyError(
                f'CartDelivery MirAction requires the configured MiR mission '
                f'name for [pickup], but it is not provided in the action '
                f'config! Unable to instantiate an ActionFactory.')
        elif 'dropoff' not in action_config['missions']:
            raise KeyError(
                f'CartDelivery MirAction requires the configured MiR mission '
                f'name for [dropoff], but it is not provided in the action '
                f'config! Unable to instantiate an ActionFactory.')
        elif 'exit_lot' not in action_config['missions']:
            raise KeyError(
                f'CartDelivery MirAction requires the configured MiR mission '
                f'name for [exit_lot], but it is not provided in the action '
                f'config! Unable to instantiate an ActionFactory.')

    def perform_action(
        self,
        category: str,
        description: dict,
        context: ActionContext,
    ) -> MirAction:
        match category:
            case 'delivery_pickup':
                return CartPickup(description, context)
            case 'delivery_dropoff':
                return CartDropoff(description, context)


class CartPickup(MirAction):
    def __init__(self, description: dict, context: ActionContext):
        MirAction.__init__(self, context)

        # Import CartDetection module
        # TODO(@xiyuoh) Import this only once during ActionFactory instantiation
        detection_module = context.action_config['cart_detection_module']
        detection_plugin = importlib.import_module(detection_module)
        self.cart_detection = detection_plugin.CartDetection(context)

        # Mission names to be used during pickup
        self.dock_to_cart_mission = \
            self.context.action_config['missions']['dock_to_cart']
        self.pickup_mission = \
            self.context.action_config['missions']['pickup']
        self.exit_mission = \
            self.context.action_config['missions']['exit_lot']

        self.search_timeout = \
            self.context.action_config.get('search_timeout', 60)  # seconds
        self.cart_marker_type_guid = \
            self.context.api.docking_offsets_guid_get(
                self.context.action_config['marker_types']['cart'])

        # Begin action
        # Check if the robot's latch is currently open
        if self.is_latch_open():
            # Latch is open, unable to perform pickup
            self.context.node.get_logger().info(
                f'Robot [{self.context.name}] latch is open, unable to '
                f'perform pickup, cancelling task...')
            self.cancel_task()
            return

        self.context.node.get_logger().info(
            f'New pickup requested for robot [{self.context.name}]')
        self.pickup = Pickup(
            state=PickupState.AT_PICKUP,
            pickup_lots=[description.get('pickup_lot')],
            cart_id=description.get('cart_id'),
            mission=None,
            latching=False
        )

    def update_action(self):
        if self.update_pickup(self.pickup):
            if self.context.execution is not None:
                self.context.execution.finished()
            return True
        return False

    def update_pickup(self, pickup: Pickup):
        # If this is a PerformAction pickup, check that the action is underway
        # and valid
        if self.context.execution is not None and not self.context.execution.okay():
            self.context.node.get_logger().info(
                f'[delivery_pickup] action is killed/canceled.')
            pickup.state = PickupState.TASK_CANCELLED

        # Start state machine check
        now = self.context.node.get_clock().now().nanoseconds / 1e9
        self.context.node.get_logger().debug(f'PickupState: {pickup.state.name}')
        match pickup.state:
            case PickupState.AT_PICKUP:
                # Send rmf_dock_to_cart mission
                assert self.dock_to_cart_mission is not None
                current_wp_name = pickup.pickup_lots[0]
                mir_pos = self.context.api.known_positions.get(current_wp_name)
                if not mir_pos:
                    self.context.node.get_logger().info(
                        f'No shelf position [{mir_pos}] found on robot '
                        f'[{self.context.name}], cancelling task')
                    self.cancel_task()
                    pickup.state = PickupState.TASK_CANCELLED
                    return
                cart_marker_guid = mir_pos['guid']
                cart_marker_param = self.context.api.get_mission_params_with_value(
                    self.dock_to_cart_mission,
                    'docking',
                    'cart_marker',
                    cart_marker_guid)
                marker_type_param = self.context.api.get_mission_params_with_value(
                    self.dock_to_cart_mission,
                    'docking',
                    'cart_marker_type',
                    self.cart_marker_type_guid)
                mission_params = cart_marker_param + marker_type_param
                mission_queue_id = self.context.api.queue_mission_by_name(
                    self.dock_to_cart_mission, mission_params)
                if not mission_queue_id:
                    error_str = \
                        f'Mission {self.dock_to_cart_mission} not ' + \
                        f'supported, ignoring'
                    self.context.node.get_logger().error(error_str)
                    return
                pickup.mission = Mission(mission_queue_id, now)
                pickup.state = PickupState.DOCK_REQUESTED
                self.context.node.get_logger().info(
                    f'[{self.context.name}] Dock to cart mission requested with '
                    f'mission queue id {mission_queue_id}')

            case PickupState.DOCK_REQUESTED:
                # Make sure that there is an rmf_dock_to_cart mission issued
                if not pickup.mission:
                    self.context.node.get_logger().info(
                        f'Robot [{self.context.name}] is indicated to be at the '
                        f'DOCK_REQUESTED state but no mission queue ID stored '
                        f'for this docking mission! Returning to AT_PICKUP '
                        f'state.')
                    pickup.state = PickupState.AT_PICKUP
                    return
                # Mission completed, move onto the next state
                if self.context.api.mission_completed(pickup.mission.queue_id):
                    self.context.node.get_logger().info(
                        f'Robot [{self.context.name}] dock to cart mission '
                        f'{pickup.mission.queue_id} completed or timed out.')
                    pickup.mission = None
                    pickup.state = PickupState.DOCK_COMPLETED
                    return
                # Mission not yet completed, we check the timeout status to
                # decide if we need to publish any alert
                seconds_passed = now - pickup.mission.start_time
                # Publish update every 10 seconds just to monitor
                if round(seconds_passed) % 10 == 0:
                    self.context.node.get_logger().info(
                        f'{round(seconds_passed)} seconds have passed since '
                        f'pickup mission requested.')
                # Mission timeout, cart not found
                if seconds_passed > self.search_timeout:
                    # Delete mission from mir first
                    self.context.api.mission_queue_id_delete(pickup.mission.queue_id)
                    # Regardless of whether the robot completed docking
                    # properly, we move to the next state to check
                    self.context.node.get_logger().info(
                        f'Robot [{self.context.name}] dock to cart mission '
                        f'{pickup.mission.queue_id} timed out! Configured '
                        f'search timeout is {self.search_timeout} seconds.')
                    pickup.mission = None
                    pickup.state = PickupState.DOCK_COMPLETED
                return

            case PickupState.DOCK_COMPLETED:
                # Check if robot docked under the correct cart
                cart_check = self.is_correct_cart(pickup.cart_id)
                if cart_check:
                    # If cart is correct, send pickup mission
                    assert self.pickup_mission is not None
                    mission_queue_id = self.context.api.queue_mission_by_name(
                        self.pickup_mission)
                    if not mission_queue_id:
                        error_str = \
                            f'Mission {self.pickup_mission} not supported,' + \
                            f' ignoring'
                        self.context.node.get_logger().error(error_str)
                        return
                    pickup.mission = Mission(mission_queue_id, now)
                    pickup.latching = True
                    self.context.node.get_logger().info(
                        f'Robot [{self.context.name}] found the correct cart, pickup '
                        f'mission requested with mission queue id '
                        f'{mission_queue_id}')
                    pickup.state = PickupState.PICKUP_REQUESTED
                elif cart_check is None:
                    # If cart is missing, cancel this task
                    self.context.node.get_logger().info(
                        f'Robot [{self.context.name}] was unable to dock under any '
                        f'carts, please check that cart is present. '
                        f'Cancelling task.')
                    self.cancel_task()
                    pickup.state = PickupState.TASK_CANCELLED
                else:
                    # If cart is wrong, cancel this task also but after we
                    # exit from the lot
                    self.context.node.get_logger().info(
                        f'Robot [{self.context.name}] found the wrong cart, exiting '
                        f'lot and cancelling task.')
                    self.exit_lot()
                    self.cancel_task()
                    pickup.state = PickupState.TASK_CANCELLED

            case PickupState.PICKUP_REQUESTED:
                # Pickup mission completed
                if self.context.api.mission_completed(pickup.mission.queue_id):
                    pickup.state = PickupState.PICKUP_SUCCESS
                    pickup.mission = None
                    pickup.latching = False

            case PickupState.PICKUP_SUCCESS:
                # Correct ID, we can end the delivery now
                self.context.node.get_logger().info(
                    f'Robot [{self.context.name}] successfully received cart, '
                    f'exiting lot with cart and ending mission')
                self.exit_lot()
                return True

            case PickupState.TASK_CANCELLED:
                self.context.node.get_logger().info(
                    f'Robot [{self.context.name}] is in pickup cancelled state.')
                # If some MiR mission is in progress, we abort it unless
                # it is latching
                if pickup.mission and not self.context.api.mission_completed(
                        pickup.mission.queue_id):
                    if pickup.latching:
                        self.context.node.get_logger().info(
                            f'Robot [{self.context.name}] is performing latching, '
                            f'cancelling task after this action is complete.')
                        return False
                    self.context.api.mission_queue_id_delete(pickup.mission.queue_id)
                    pickup.mission = None
                # Clear any errors
                self.context.api.clear_error()
                self.context.api.status_put(state_id=MiRStateCode.READY)
                # NOTE(@xiyuoh) If this task is cancelled in the middle of a
                # cart pickup (latching), the users should implement a
                # cancellation behavior in the task JSON to ensure that this
                # robot can continue with its next task without having a cart
                # latched on.
                return True

        return False

    def is_latch_open(self):
        return self.cart_detection.is_latch_open()

    def is_correct_cart(self, cart_id: str):
        return self.cart_detection.is_correct_cart(cart_id)

    def is_under_cart(self):
        return self.cart_detection.is_under_cart()

    def exit_lot(self):
        if not self.context.api.connected:
            return None
        # Set footprint to robot footprint before exiting lot
        return self.context.api.queue_mission_by_name(self.exit_mission)

    def cancel_task(self, label: str = ''):
        def _cancel_success():
            if self.pickup:
                self.pickup.state = PickupState.TASK_CANCELLED
        def _cancel_fail():
            pass
        self.cancel_current_task(_cancel_success, _cancel_fail, label)


class CartDropoff(MirAction):
    def __init__(self, description: dict, context: ActionContext):
        MirAction.__init__(self, context)

        self.dropoff: Dropoff = None
        self.dropoff_mission = \
            self.context.action_config['missions']['dropoff']

    def update_action(self):
        if self.update_dropoff(self.dropoff):
            if self.context.execution is not None:
                self.context.execution.finished()
            return True
        return False

    def update_dropoff(self, dropoff: Dropoff):
        # Check if action is underway or cancelled
        if self.context.execution is not None and not self.context.execution.okay():
            self.context.node.get_logger().info(
                f'[delivery_dropoff] action is killed/canceled')

            # If cart release is in progress, we let it finish first
            if dropoff.mission and not self.context.api.mission_completed(
                    dropoff.mission.queue_id):
                return False

            # Task is cancelled and cart is done dropping off/mission is not
            # yet queued anyway, we can mark it as completed at this point
            self.context.api.clear_error()
            self.context.api.status_put(state_id=MiRStateCode.READY)
            # Mark dropoff session as completed
            return True

        # No mission queued yet
        if dropoff.mission is None:
            assert self.dropoff_mission is not None
            mission_queue_id = self.context.api.queue_mission_by_name(
                self.dropoff_mission)
            if not mission_queue_id:
                error_str = \
                    f'Mission {self.dropoff_mission} not supported, ignoring'
                self.context.node.get_logger().error(error_str)
                return True
            self.context.node.get_logger().info(
                f'Mission {self.dropoff_mission} added to queue for robot '
                f'[{self.context.name}].')
            now = self.context.node.get_clock().now().nanoseconds / 1e9
            dropoff.mission = Mission(mission_queue_id, now)
            self.context.node.get_logger().info(
                f'[{self.context.name}] Dropoff mission requested with mission queue '
                f'id {mission_queue_id}')

        # Mission queued, check completion
        else:
            if self.context.api.mission_completed(dropoff.mission.queue_id):
                self.context.node.get_logger().info(
                    f'[{self.context.name}] Dropoff mission completed!')
                return True
            else:
                self.context.node.get_logger().info(
                    f'[{self.context.name}] Dropoff mission in progress...')
        return
