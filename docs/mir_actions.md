## MiR Action Plugins

This section lists and elaborates on the MiR actions provided out-of-the-box in this repo.

## Available Action Plugins

* [rmf_cart_delivery](#rmf_cart_delivery)


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
