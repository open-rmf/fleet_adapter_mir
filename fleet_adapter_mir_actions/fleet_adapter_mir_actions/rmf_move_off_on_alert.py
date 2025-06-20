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

from .rmf_move_off import BaseMoveOff
from fleet_adapter_mir.robot_adapter_mir import ActionContext


class MoveOff(BaseMoveOff):
    def __init__(self, context: ActionContext):
        BaseMoveOff.__init__(self, context)

        '''
        This example demonstrates how we can notify the robot to move off when
        it receives an Alert via ROS 2.
        '''
        self.mutex = Lock()
        self.alert = None
        self.ready = False

        # Create alert related publisher and subscribers
        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL,
        )
        self.alert_response_sub = self.context.node.create_subscription(
            AlertResponse,
            'alert_response',
            self.alert_response_cb,
            qos_profile=transient_qos
        )
        self.alert_pub = self.context.node.create_publisher(
            Alert,
            'alert',
            qos_profile=transient_qos
        )

    def verify_signal(self, signal_config: dict) -> bool:
        return True

    def begin_waiting(self, signal_config: dict) -> bool:
        # Publish an alert to signal that the robot has begun waiting
        msg = Alert()
        msg.id = datetime.datetime.now().strftime(
            "alert-%Y-%m-%d-%H-%M-%S")
        msg.title = f'Robot [{self.context.name}] has begun waiting.'
        msg.tier = Alert.TIER_INFO
        msg.responses_available = ['ready']
        msg.task_id = self.context.update_handle.more().current_task_id()
        self.alert_pub.publish(msg)
        self.context.node.get_logger().info(
            f'Robot [{self.context.name}] published alert [{msg.id}] to '
            f'signal that it has started waiting.'
        )
        self.alert = msg
        return True

    def is_move_off_ready(self) -> bool:
        with self.mutex:
            if self.ready:
                return True
        return False

    def alert_response_cb(self, msg):
        if not self.alert:
            return
        if msg.id != self.alert.id:
            return

        if msg.response != 'ready':
            self.context.node.get_logger().info(
                f'Robot [{self.context.name}] received invalid response '
                f'inside alert response: [{msg.response}], ignoring...'
            )
            return

        self.context.node.get_logger().info(
            f'Received move off signal, robot [{self.context.name}] is done '
            f'waiting, marking action as completed.'
        )
        with self.mutex:
            self.ready = True
