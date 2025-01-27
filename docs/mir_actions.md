## MiR Action Plugins

This section lists and elaborates on the MiR actions provided out-of-the-box in this repo.

## Available Action Plugins

* [rmf_cart_delivery](#rmf_cart_delivery)
* [rmf_wait_until](#rmf_wait_until)


## rmf_cart_delivery

### Overview

The `rmf_cart_delivery` plugin allows users to submit pickup and dropoff tasks for MiR from point A to point B via RMF. The intended workflow of a delivery task is as follows:
1. RMF will send the robot to the pickup lot.
2. The robot will attempt to dock under a cart in the pickup lot. If the cart is missing or is not the desired cart, RMF will cancel the task.
3. If the robot successfully docks under the correct cart, it will latch onto the cart.
4. With the cart attached, the robot will move to the designated dropoff point.
5. Upon reaching the dropoff point, the robot will release and exit from under the cart. The task ends once the robot has safely exited.

### Setup

Some relevant MiR missions are pre-defined and can be automatically created on the MiR on startup. These missions are used to facilitate the pickup and dropoff activities. They are defined and stored in the `rmf_cart_missions.json` file and do not require any further configuration.

However, since there are various types of latching methods available for different MiR models, users will need to set up their custom pickup and dropoff missions on the MiR, as well as implement their own `CartDetection` plugin module with the appropriate APIs to detect latching states. A `BaseCartDetection` abstract class is provided the methods to be implemented.

Before setting up, you may want to refer to the [list of RMF missions](https://github.com/open-rmf/fleet_adapter_mir/blob/main/docs/mir_missions.md#RMF-missions-for-rmf_cart_delivery) required for this MiR Action.

Steps for setup:

1. Create 2 missions on the MiR:
   - `rmf_pickup_cart`: Triggers the robot's latching module to open
   - `rmf_dropoff_cart`: Triggers the robot's latching module to close and release the cart, then exit from under the cart (relative move in the negative X-direction)
2. Fill in the MiR mission names in the plugin config under `missions`.
   - This helps the fleet adapter identify and map the action to the missions you have created earlier.
   - The recommended mission names are `rmf_pickup_cart` and `rmf_dropoff_cart`, per the instructions in Step 1. However, it is possible to use a different mission name as long as it is indicated accordingly under `missions`.
3. Fill in the appropriate cart marker type in the plugin config under `marker_type`.
4. Create your own `CartDetection` plugin.
   - You are encouraged to use the `BaseCartDetection` class in `rmf_cart_detection.py` as a base for your own module implementation. The class methods will be used by the `CartPickup` and `CartDropoff` Mir Actions. Some API calls to check the MiR's PLC registers and IO modules are provided in case you may want to use them.
   - In the plugin config, update the `cart_detection_module` field to point to your own written module.
6. Enable RMF cart missions creation.
   - If this is your first time setting up the action, and the pre-defined RMF cart missions have not been created on your robot, you will need to provide the filepath to `rmf_cart_missions.json` under `missions_json`.

You can refer to `mir_config.yaml` under the `configs` folder for an example of a filled-in plugin configuration.


### Usage

To submit a cart delivery task, you may use the `dispatch_delivery` task script found in the `fleet_adapter_mir_tasks` package:
```bash
ros2 run fleet_adapter_mir_tasks dispatch_delivery -g go_to_waypoint -p pickup_lot -d dropoff_lot -c some_cart_id
```
- `-g`: Takes in an existing waypoint name for the robot to travel to before performing the pickup. The robot will begin docking into the pickup lot from this waypoint.
- `-p`: Name of the pickup lot. This name should be identical to the shelf position configured on the MiR.
- `-d`: Name of the dropoff lot. This name should be identical to the robot or shelf position configured on the MiR.
- `-c`: Optional cart identifier for the fleet adapter to assess whether the cart is correct for pickup. 


## rmf_wait_until

### Overview

The `rmf_wait_until` plugin introduces the `wait_until` action. It allows users to command a robot to wait at a specified location until it receives a move off signal or until a configured timeout. The robot would then be free to move on to carry out the remainder of its task, or complete the task and proceed to its idle behavior. During this waiting period, the user may command the robot to perform missions or any customized behavior with user-defined move off signals to trigger completion.

This action can come in handy for various use cases, for example:
- The robot has to perform a delivery where it travels between pick up and drop off locations, and wait at each location for an external device to load or unload items on itself.
- The robot is performing a MiR mission at a specific location, and is only ready to move off when the mission is completed.

Here is the workflow of a multi-stop task, demonstrating how the `wait_until` action can be used:
1. Users submit a task with a list of waypoints, RMF will send the robot to the first waypoint
2. The robot will stop at the waypoint and do nothing until it receives a move off signal or until a configured timeout.
3. Repeat Steps 1. and 2. until the robot has travelled to all the waypoints in the task.

Move off signals are plugin-based. Users can specify the MoveOff plugin they would like to use for their `wait_until` action. Multiple move off signals may be configured in the fleet config used at runtime.

The following lists some move off signal plugins currently available:
1. Move off when a MiR mission completes.
   - Users may create/select a MiR mission on the robot and provide the mission name in the fleet config or task description. This mission will be submitted to the robot when the waiting action begins. The waiting action will end when the robot receives the move off signal, which in this case happens when the robot completes the mission.
2. Move off when a PLC register returns `True`.
   - When the waiting action begins, the fleet adapter will monitor the state of the PLC register specified in the fleet config or task description. When the register returns a non-zero integer or `True`, the waiting action will end and the robot will move on to its next waypoint or task. Numeric strings convertible to integers are also accepted, e.g. "1", "5".
3. Move off on alerts received via ROS 2.
   - The fleet adapter will subscribe to the `alert_response` topic and trigger a move off when it receives a ready message.

Users can customize their own signal type by implementing a `MoveOff` plugin and include it in the fleet config.

The plugin also supports task-specific move off signals, such that users can trigger different move off signals with different signal config in each task. These move off signals should use plugins that are pre-defined in the fleet config. It is up to the user to ensure that these signal types are valid and compatible with the MiR.

### Setup

Users can configure multiple move off signals for their robot fleet in the fleet's plugin config, as long as they are supported by the WaitUntil action. If signal types are not configured in the plugin config nor the task description, the robot will not have a move off signal set up, and simply wait for the full duration of the timeout during the action.

Steps for setup:

1. Fill in the appropriate fields under the `rmf_wait_until` plugin in the fleet config, with reference to the example provided in `mir_config.yaml`. It allows users to customize the behavior of their robots during the waiting action and the type of signal to trigger move off.
   - `timeout`: Optional, the default timeout of the waiting action. At timeout, the robot will move off even if it did not receive the move off signal. If not specified, the timeout will default to 60 seconds. This value can be overridden by the task description.
   - `update_gap`: Optional, the update interval for logging purposes. If not specified, the update gap will default to 30 seconds. This value can be overridden by the task description.
   - `move_off_plugins`: Provides a plugin name and path to the module. The fleet adapter will import these modules at startup. If no plugin is provided, the robot will wait the full duration of the default timeout for every `wait_until` action. This action does not support importing move off plugins at runtime.
   - `signals`: List the signals to be pre-configured for the fleet.
      - Provide a unique name for each of your signal, with a mandatory field `plugin` pointing to a configured move off plugin listed above.
      - Any additional configuration required for each move off signal plugin goes here.
      - The following elaborates on each currently available plugins and their respective config:
         - `rmf_move_off_on_mission`
            - `mission_name` is required for the robot to trigger the relevant MiR mission when the waiting action starts. This is a compulsory field.
            - `retry_count` is an integer and an optional field, used to configure the number of times the fleet adapter should re-attempt posting a mission at the start of the waiting action. If the mission cannot be successfully queued on the robot beyond the number of retries, the task will be cancelled. Default to 10.
            - `resubmit_on_abort` is a boolean and an optional field, used to configure the action behavior in cases where the MiR mission cannot be successfully completed and gets aborted by the robot. When set to `True`, the fleet adapter will submit the same MiR mission to the robot if the previous attempt has been aborted. The move off signal will come only when the mission has been successfully completed. If set to `False`, the fleet adapter will treat aborted missions as completed and end the waiting action. Default to False.
         - `rmf_move_off_on_mission`
            - `register`: a PLC register number that is an integer and available on the MiR. This is a compulsory field.
         - `rmf_move_off_on_alert`: No signal config required.
   - The `default_signal` field is optional but encouraged to be added. It should point to any one of the configured signals. If no signal is provided in the task description, the fleet adapter will select the default signal as the move off trigger. If neither is provided, the robot will simply wait for the full duration of the timeout since it does not have a move off signal configured.
2. [Optional] Create your own `MoveOff` plugin.
   - You are encouraged to use the `BaseMoveOff` class in `rmf_move_off.py` as a base for your own plugin implementation. The class methods will be used by the `WaitUntil` Mir Action.


### Usage

To submit a multi-stop waiting task, you may use the `dispatch_multistop` task script found in the `fleet_adapter_mir_tasks` package:
```bash
# Trigger a task with a configured signal custom_2
ros2 run fleet_adapter_mir_tasks dispatch_multistop -g waypoint_1 waypoint_2 waypoint_3 -n sample_alert_1

# Trigger a task with a new mission signal and a timeout of 120 seconds
ros2 run fleet_adapter_mir_tasks dispatch_multistop -g waypoint_1 waypoint_2 waypoint_3 -t 120 -p rmf_move_off_on_mission -m some_mission_name

# Trigger a task with a new PLC signal
ros2 run fleet_adapter_mir_tasks dispatch_multistop -g waypoint_1 waypoint_2 waypoint_3 -p rmf_move_off_on_plc -r 30

# Trigger the default signal configured in plugin config
ros2 run fleet_adapter_mir_tasks dispatch_multistop -g waypoint_1 waypoint_2 waypoint_3
```
- `-g`: Takes in the waypoints the robots should travel to for each waiting action.
- `-t`: Optional timeout of the action in seconds. Default to 60 seconds.
- `-u`: Optional update gap of the action in seconds. Default to 30 seconds.
- `-n`: Signal name for this `wait_until` action. This signal name has to be pre-configured in the fleet config. If not provided, the task script will refer to the `plugin` and relevant `signal_config` provided instead.
- `-p`: Move off plugin for this `wait_until` action. The plugin has to be pre-configured in the fleet config.
- `-m`: Further specifies the mission name for signal plugin `rmf_move_off_on_mission`.
- `-a`: A boolean determining whether to resubmit missions if they are aborted by the robot. Used for signal plugin `rmf_move_off_on_mission`.
- `-rc`: An integer indicating the number of times to reattempt queueing a mission. Used for signal plugin `rmf_move_off_on_mission`.
- `-r`: Further specifies the PLC register number for signal type `rmf_move_off_on_plc`.

Do note that this task involves using the same move off signal for every waypoint the robot travels to. This task script only support populating signal config for `rmf_move_off_on_plc` and `rmf_move_off_on_mission`. For any custom plugins that require task-specific signal config, users will need to create their own task script.
