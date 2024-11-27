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

However, since there are various types of latching methods available for different MiR models, users will need to set up their custom pickup and dropoff missions on the MiR, as well as implement their own `CartDetection` plugin module with the appropriate APIs to detect latching states.

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
   - You are encouraged to use the `CartDetection` class in `rmf_cart_detection.py` as a base for your own module implementation. The class methods will be used by the `CartPickup` and `CartDropoff` Mir Actions. Some API calls to check the MiR's PLC registers and IO modules are provided in case you may want to use them.
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

The `rmf_wait_until` plugin allows users to create a multistop task for their MiR integrated with RMF. The action would consist of the robot travelling to multiple waypoints, stop and wait there for a configured period of time, then complete the task by performing its finishing request. Users may wish to use the move-off behavior provided in the plugin or customize their own to end the waiting process earlier.

This action can come in handy for various use cases, for example:
- The robot has to perform a multistop delivery, i.e. travel to several waypoints to pickup or dropoff some items that it is carrying
- The robot is performing the same custom MiR mission at different locations, and is ready to move off to the next stop when the mission is completed.

The workflow of the task is as follows:
1. After users submit a task with a list of waypoints for the robot to travel to, RMF will send the robot to the first waypoint
2. The robot will stop at the waypoint and do nothing until it receives a move-off signal or timeout (both are configurable)
3. Steps 1. and 2. repeat until the robots has travelled to all the waypoints in the task.


There are 2 move-off behavior provided in the plugin.
1. Move-off when a MiR mission completes.
   - Users may create/pick a MiR mission on the robot and provide the mission name in the fleet config or task description. This mission will be submitted to the robot when the waiting action begins. The waiting action will end when the robot receives the move-off signal, which in this case happens when the robot completes the mission.
2. Move-off when a PLC register returns `True`.
   - When the waiting action begins, the fleet adapter will monitor the state of the PLC register specified in the fleet config or task description. When the register returns a value of `1` or `True`, the waiting action will end and the robot will move on to its next waypoint or task.


It is important to fill in the configuration relevant to the `wait_until` action following the example provided in `mir_config.yaml`. It allows users to customize the behavior of their robots during the waiting action:
- `timeout`: The default timeout of the waiting action. At timeout, the robot will move off even if it did not receive the move off signal.
- `signal_type`: The type of move off signal for the robot. We currently support `mission`, `plc` and `custom`.
   - If `mission` signal is chosen, users will need to provide the `mission_name` for the robot to trigger the MiR mission when the waiting action starts.

     An optional field `resubmit_on_abort` can be used to configure the action behavior in cases where the MiR mission cannot be successfully completed and gets aborted by the robot. When set to `True`, the fleet adapter will submit the same MiR mission to the robot if the previous attempt has been aborted. The move-off signal will come only when the mission has been successfully completed. If `resubmit_on_abort` is set to `False`, the fleet adapter will accept abort missions as a move-off signal and end the waiting action.
   - If `plc` signal is chosen, users will need to provide the relevant `plc_register`.
   - If `custom` is chosen, users will need to provide a path their custom `MoveOff` module. This will require your own `MoveOff` plugin. A skeleton of the plugin is provided in `rmf_move_off.py`, with `# IMPLEMENT YOUR CODE HERE` indicating where users should fill in their code logic. An example demonstrating how a move-off signal can be called with ROS 2 `Alert` msg is provided in `rmf_move_off_on_alert.py`.
   - If the `signal_type` is not provided, the robot will carry out the waiting action for the full duration of the default timeout.

For task-specific waiting behavior, users could provide the same signal config in their task description instead. The signal behavior provided in the task description will override the default behavior in the fleet config.


To submit a multistop waiting task, you may use the `dispatch_multistop` task script found in the `fleet_adapter_mir_tasks` package:
```bash
ros2 run fleet_adapter_mir_tasks dispatch_multistop -g waypoint_1 waypoint_2 waypoint_3 -t 1800
```
- `-g`: Takes in the waypoints the robots should travel to for each waiting action.
- `-t`: Default timeout of the action in seconds.
