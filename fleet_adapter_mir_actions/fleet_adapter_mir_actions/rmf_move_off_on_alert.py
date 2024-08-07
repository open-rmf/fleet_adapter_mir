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

import datetime
from threading import Lock

from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy as Reliability
from rmf_task_msgs.msg import Alert
from rmf_task_msgs.msg import AlertResponse
from rmf_task_msgs.msg import AlertParameter


class MoveOff:
    def __init__(
            self,
            action_handle,
            signal_config,
            task_description
    ):
        self.action = action_handle
        self.config = signal_config
        self.task_desc = task_description

        '''
        This example demonstrates how we can notify the robot to move off when
        it receives an Alert via ROS 2.
        '''
        self.mutex = Lock()
        self.alert = None
        self.move_off = False

        # Create alert related publisher and subscribers
        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL,
        )
        self.alert_response_sub = self.action.node.create_subscription(
            AlertResponse,
            'alert_response',
            self.alert_response_cb,
            qos_profile=transient_qos
        )
        self.alert_pub = self.action.node.create_publisher(
            Alert,
            'alert',
            qos_profile=transient_qos
        )

    def begin_waiting(self, description):
        # Publish an alert to signal that the robot has begun waiting
        msg = Alert()
        msg.id = datetime.datetime.now().strftime(
            "alert-%Y-%m-%d-%H-%M-%S")
        msg.title = f'Robot [{self.action.name}] has begun waiting.'
        msg.tier = Alert.TIER_INFO
        msg.responses_available = ['ready']
        msg.task_id = self.action.update_handle.more().current_task_id()
        self.alert_pub.publish(msg)
        self.action.node.get_logger().info(
            f'Robot [{self.action.name}] published alert [{msg.id}] to signal '
            f'that it has started waiting.'
        )
        self.alert = msg

    def is_move_off_ready(self):
        with self.mutex:
            if self.move_off:
                # This move-off object is created everytime the robot begins a
                # wait_until action, so there is no need to toggle it back to
                # False
                return True
        return False

    def alert_response_cb(self, msg):
        if not self.alert:
            return
        if msg.id != self.alert.id:
            return

        if msg.response != 'ready':
            self.node.get_logger().info(
                f'Robot [{self.name}] received invalid response inside alert '
                f'response: [{msg.response}], ignoring...'
            )
            return

        self.node.get_logger().info(
            f'Robot [{self.name}] received move-off signal, delivery is '
            f'complete, marking action as completed.'
        )
        with self.mutex:
            self.move_off = True
