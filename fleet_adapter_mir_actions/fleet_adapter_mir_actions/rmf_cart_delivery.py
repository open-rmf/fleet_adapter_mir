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
    cart_id: str | None
    mission: Mission
    latching: bool


@dataclass
class Dropoff:
    mission: Mission


class ActionFactory(MirActionFactory):
    def __init__(self, context: ActionContext):
        MirActionFactory.__init__(self, context)
        # Raise error if config file is invalid
        # NOTE(@xiyuoh) Using if-else to check for valid keys in the action
        # config is not the most scalable. Consider other ways to do this.
        if 'cart_detection_module' not in context.action_config:
            raise KeyError(
                f'CartDelivery MirAction requires a cart detection module, '
                f'but path to [cart_detection_module] is not provided in the '
                f'action config! Unable to instantiate an ActionFactory.')
        elif 'marker_types' not in context.action_config:
            raise KeyError(
                f'CartDelivery MirAction requires configured marker types, '
                f'but [marker_types] not provided in the action config! '
                f'Unable to instantiate an ActionFactory.')
        elif 'missions' not in context.action_config:
            raise KeyError(
                f'CartDelivery MirAction requires configured missions, but '
                f'[missions] not provided in the action config! '
                f'Unable to instantiate an ActionFactory.')
        # Check if the required mission names have been configured
        elif ('dock_to_cart' not in context.action_config['missions'] or
                context.action_config['missions']['dock_to_cart'] is None):
            raise KeyError(
                f'CartDelivery MirAction requires the configured MiR mission '
                f'name for [dock_to_cart], but it is not provided in the '
                f'action config! Unable to instantiate an ActionFactory.')
        elif ('pickup' not in context.action_config['missions'] or
                context.action_config['missions']['pickup'] is None):
            raise KeyError(
                f'CartDelivery MirAction requires the configured MiR mission '
                f'name for [pickup], but it is not provided in the action '
                f'config! Unable to instantiate an ActionFactory.')
        elif ('dropoff' not in context.action_config['missions'] or
                context.action_config['missions']['dropoff'] is None):
            raise KeyError(
                f'CartDelivery MirAction requires the configured MiR mission '
                f'name for [dropoff], but it is not provided in the action '
                f'config! Unable to instantiate an ActionFactory.')
        elif ('exit_lot' not in context.action_config['missions'] or
                context.action_config['missions']['exit_lot'] is None):
            raise KeyError(
                f'CartDelivery MirAction requires the configured MiR mission '
                f'name for [exit_lot], but it is not provided in the action '
                f'config! Unable to instantiate an ActionFactory.')

        # Import CartDetection module
        detection_module = context.action_config['cart_detection_module']
        detection_plugin = importlib.import_module(detection_module)
        self.cart_detection = detection_plugin.CartDetection(self.context)

    def supports_action(self, category: str) -> bool:
        match category:
            case 'delivery_pickup':
                return True
            case 'delivery_dropoff':
                return True
            case _:
                return False

    def perform_action(
        self,
        category: str,
        description: dict,
        execution
    ) -> MirAction:
        execution.set_automatic_cancel(False)
        match category:
            case 'delivery_pickup':
                return CartPickup(
                    description, execution, self.context, self.cart_detection)
            case 'delivery_dropoff':
                return CartDropoff(
                    description, execution, self.context, self.cart_detection)
        raise ValueError(
            f'Action [{category}] has been called for rmf_cart_delivery, '
            f'but it is not a supported action!'
        )


class CartPickup(MirAction):
    def __init__(
        self,
        description: dict,
        execution,
        context: ActionContext,
        cart_detection
    ):
        MirAction.__init__(self, context, execution)

        # Mission names to be used during pickup
        self.dock_to_cart_mission = \
            context.action_config['missions']['dock_to_cart']
        self.pickup_mission = \
            context.action_config['missions']['pickup']
        self.exit_mission = \
            context.action_config['missions']['exit_lot']

        self.cart_detection = cart_detection
        self.search_timeout = \
            context.action_config.get('search_timeout', 60)  # seconds
        self.cart_marker_type_guid = \
            context.api.docking_offsets_guid_get(
                context.action_config['marker_types']['cart'])

        # Begin action
        self.context.node.get_logger().info(
            f'New pickup requested for robot [{self.context.name}]')
        self.pickup = Pickup(
            state=PickupState.AT_PICKUP,
            pickup_lots=[description.get('pickup_lot')],
            cart_id=description.get('cart_id'),
            mission=None,
            latching=False
        )

        # Check if the robot's latch is currently open
        if self.cart_detection.is_latch_open():
            # Latch is open, unable to perform pickup
            self.context.node.get_logger().info(
                f'Robot [{self.context.name}] latch is open, unable to '
                f'perform pickup, cancelling task...')
            self.cancel_task(
                label='Robot latch is still open, unable to perform pickup.'
            )

    def update_action(self):
        return self.update_pickup(self.pickup)

    def update_pickup(self, pickup: Pickup):
        # If the action is no longer active, perform cleanup by updating
        # PickupState to TASK_CANCELLED.
        if self.execution is not None and not self.execution.okay():
            self.context.node.get_logger().info(
                f'[delivery_pickup] action is no longer underway and valid, '
                f'setting PickupState to TASK_CANCELLED.')
            pickup.state = PickupState.TASK_CANCELLED

        # Start state machine check
        now = self.context.node.get_clock().now().nanoseconds / 1e9
        self.context.node.get_logger().debug(
            f'PickupState: {pickup.state.name}')
        match pickup.state:
            case PickupState.AT_PICKUP:
                # Send rmf_dock_to_cart mission
                current_wp_name = pickup.pickup_lots[0]
                mir_pos = self.context.api.known_positions.get(current_wp_name)
                if not mir_pos:
                    self.context.node.get_logger().info(
                        f'No shelf position [{mir_pos}] found on robot '
                        f'[{self.context.name}], cancelling task')
                    self.cancel_task(
                        label=f'MiR position is not found on the robot, '
                        f'unable to pickup.'
                    )
                    pickup.state = PickupState.TASK_CANCELLED
                    return
                cart_marker_guid = mir_pos['guid']
                cart_marker_param = \
                    self.context.api.get_mission_params_with_value(
                        self.dock_to_cart_mission,
                        'docking',
                        'cart_marker',
                        cart_marker_guid)
                marker_type_param = \
                    self.context.api.get_mission_params_with_value(
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
                    f'[{self.context.name}] Dock to cart mission requested '
                    f'with mission queue id {mission_queue_id}')

            case PickupState.DOCK_REQUESTED:
                # Make sure that there is an rmf_dock_to_cart mission issued
                if not pickup.mission:
                    self.context.node.get_logger().info(
                        f'Robot [{self.context.name}] is indicated to be at '
                        f'the DOCK_REQUESTED state but no mission queue ID '
                        f'stored for this docking mission! Returning to '
                        f'AT_PICKUP state.')
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
                    self.context.api.mission_queue_id_delete(
                        pickup.mission.queue_id)
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
                cart_check = \
                    self.cart_detection.is_correct_cart(pickup.cart_id)
                if cart_check:
                    # If cart is correct, send pickup mission
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
                        f'Robot [{self.context.name}] found the correct cart, '
                        f'pickup mission requested with mission queue id '
                        f'{mission_queue_id}')
                    pickup.state = PickupState.PICKUP_REQUESTED
                elif cart_check is None:
                    # If cart is missing, cancel this task
                    self.context.node.get_logger().info(
                        f'Robot [{self.context.name}] was unable to dock '
                        f'under any carts, please check that cart is present. '
                        f'Cancelling task.')
                    self.cancel_task(
                        label=f'Dock to cart failed, unable to perform pickup.'
                    )
                    pickup.state = PickupState.TASK_CANCELLED
                else:
                    # If cart is wrong, cancel this task also but after we
                    # exit from the lot
                    self.context.node.get_logger().info(
                        f'Robot [{self.context.name}] found the wrong cart, '
                        f'exiting lot and cancelling task.')
                    if not self.exit_lot():
                        return
                    self.cancel_task(
                        label=f'Wrong cart found, unable to perform pickup.'
                    )
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
                if not self.exit_lot():
                    return
                return True

            case PickupState.TASK_CANCELLED:
                self.context.node.get_logger().info(
                    f'Robot [{self.context.name}] is in pickup cancelled '
                    f'state.')
                # If some MiR mission is in progress, we abort it unless
                # it is latching
                if pickup.mission and not self.context.api.mission_completed(
                        pickup.mission.queue_id):
                    if pickup.latching:
                        self.context.node.get_logger().info(
                            f'Robot [{self.context.name}] is performing '
                            f'latching, cancelling task after this action is '
                            f'complete.')
                        return False
                    self.context.api.mission_queue_id_delete(
                        pickup.mission.queue_id)
                    pickup.mission = None
                # Clear any errors
                self.context.api.clear_error()
                self.context.api.status_put(state_id=MiRStateCode.READY)

                # NOTE(@xiyuoh) If this task is cancelled in the middle of a
                # cart pickup, the dispatch_delivery task is built with a
                # cancellation behavior to ensure that this robot drops the
                # cart off and is able to continue with its next task safely
                return True

        return False

    def exit_lot(self):
        mission_queue_id = \
            self.context.api.queue_mission_by_name(self.exit_mission)
        if not mission_queue_id:
            self.context.node.get_logger().info(
                f'Unable to queue exit lot mission for robot '
                f'[{self.context.name}], retrying in the next '
                f'action loop'
            )
        return mission_queue_id

    def cancel_task(self, label: str = ''):
        def _cancel_success():
            if self.pickup:
                self.pickup.state = PickupState.TASK_CANCELLED

        def _cancel_fail():
            pass
        self.cancel_task_of_action(_cancel_success, _cancel_fail, label)


class CartDropoff(MirAction):
    def __init__(
        self,
        description: dict,
        execution,
        context: ActionContext,
        cart_detection
    ):
        MirAction.__init__(self, context, execution)

        self.cart_detection = cart_detection
        self.dropoff_mission = \
            context.action_config['missions']['dropoff']
        self.exit_mission = \
            context.action_config['missions']['exit_lot']

        # Check if robot is currently latching onto a cart. If the latch is
        # released before triggering a dropoff mission, there is no need to
        # perform a full dropoff. We can skip queueing the dropoff mission in
        # the update loop and end action after ensuring that the robot is no
        # longer under a cart.
        self.skip_dropoff = False
        if not self.cart_detection.is_latch_open():
            self.context.node.get_logger().info(
                f'Robot [{self.context.name}] latch is not open, dropoff '
                f'mission will not be queued')
            self.skip_dropoff = True

        # Begin action
        self.context.node.get_logger().info(
            f'New dropoff requested for robot [{self.context.name}]')
        self.dropoff = Dropoff(mission=None)

    def update_action(self):
        return self.update_dropoff(self.dropoff)

    def update_dropoff(self, dropoff: Dropoff):
        # If action is no longer active, log for awareness. No cleanup required
        if self.execution is not None and not self.execution.okay():
            self.context.node.get_logger().info(
                f'[delivery_dropoff] action is no longer underway and valid!')

        # If skip_dropoff flag is raised, check if the robot is under a cart
        # and queue exit lot mission accordingly
        if self.skip_dropoff:
            if self.cart_detection.is_under_cart():
                self.context.node.get_logger().info(
                    f'Robot [{self.context.name}] is detected to be under a '
                    f'cart, exiting lot...'
                )
                if not self.context.api.queue_mission_by_name(
                        self.exit_mission):
                    self.context.node.get_logger().info(
                        f'Unable to queue exit lot mission for robot '
                        f'[{self.context.name}], retrying on the next update'
                    )
                    return
            return True

        # No mission queued yet, queue dropoff mission
        if dropoff.mission is None:
            mission_queue_id = self.context.api.queue_mission_by_name(
                self.dropoff_mission)
            if not mission_queue_id:
                self.context.node.get_logger().error(
                    f'Unable to queue mission [{self.dropoff_mission}], '
                    f'retrying on next update'
                )
                return
            now = self.context.node.get_clock().now().nanoseconds / 1e9
            dropoff.mission = Mission(mission_queue_id, now)
            self.context.node.get_logger().info(
                f'[{self.context.name}] Dropoff mission requested with '
                f'mission queue id {mission_queue_id}')

        # Mission queued, check completion
        else:
            if self.context.api.mission_completed(dropoff.mission.queue_id):
                self.context.node.get_logger().info(
                    f'[{self.context.name}] Dropoff mission completed!')
                self.context.api.clear_error()
                self.context.api.status_put(state_id=MiRStateCode.READY)
                return True
            else:
                self.context.node.get_logger().info(
                    f'[{self.context.name}] Dropoff mission in progress...')
        return
