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

from fleet_adapter_mir.robot_adapter_mir import ActionContext
from .rmf_cart_detection import BaseCartDetection


class CartDetection(BaseCartDetection):
    def __init__(self, context: ActionContext):
        BaseCartDetection.__init__(self, context)

    '''
    This method checks if the robot's latch is open and carrying a cart.
    Return True if latch is open, else False.
    '''
    def is_latch_open(self):
        return self.register_get(20)

    '''
    This method checks if the robot is currently docked under a cart.
    Return True if robot is under any carts, else False.
    '''
    def is_under_cart(self):
        detected_cart_id = self.register_get(30)
        if (detected_cart_id is None or detected_cart_id == 0):
            return False
        return True

    '''
    This method checks if the detected cart identifier matches the target
    cart_id, if any.
    Return True if cart is correct, False if cart is wrong, None if no cart is
    detected.
    '''
    def is_correct_cart(self, cart_id: str | None):
        if cart_id is None or cart_id == '':
            return True
        detected_cart_id = self.register_get(30)
        return detected_cart_id == int(cart_id)
