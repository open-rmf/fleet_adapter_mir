## Usage

### Pre-defined RMF variable move mission

Users are required to define a custom mission with a single `Move` action, which has the variables `x`, `y` and `yaw`, attached to the coordinates `x`, `y` and `yaw` respectively. This is the mission that the fleet adapter will use to send move commands to the robot, while modifying the desired `x`, `y` and `yaw` values.

For more information on how to set up missions, actions and variables, please refer to [official guide documents](https://www.manualslib.com/manual/1941073/Mir-Mir250.html?page=150#manual) (this is for the MiR250).

The name of this custom mission, will need to be passed to the fleet adapter through the configuration file, under `rmf_move_mission`, for each robot's `mir_config`.

```yaml
robots:
  ROBOT_NAME:
    mir_config:
      ...
      rmf_move_mission: "CUSTOM_MISSION_NAME"
      ...
```

### Example Entry Point

An example usage of the implemented adapter can be found in the `fleet_adapter_mir/fleet_adapter_mir.py` file. It takes in a configuration file and navigation graph, sets everything up, and spins up the fleet adapter.

Call it like so:

```bash
# Source the workspace
source ~/mir_ws/install/setup.bash

# Get help
ros2 run fleet_adapter_mir fleet_adapter_mir -h

# Run in dry-run mode. Disables ROS 2 publishing and MiR REST calls
cd ~/mir_ws/src/fleet_adapter_mir/configs
ros2 run fleet_adapter_mir fleet_adapter_mir -c mir_config.yaml -n nav_graph.yaml -d
```

Alternatively, if you want to run everything with full capabilities, (though note that it will require an `rmf_traffic_ros2` schedule node to be active, and the MiR REST server to be available)

```bash
ros2 run fleet_adapter_mir fleet_adapter_mir -c mir_config.yaml -n nav_graph.yaml
```

If you have not configured the necessary RMF missions on the MiR, you may parse the relevant JSON filepaths when launching the fleet adapter node. On startup, the fleet adapter will create these missions via MiR REST API

```bash
ros2 run fleet_adapter_mir fleet_adapter_mir -c mir_config.yaml -n nav_graph.yaml -a ../missions/rmf_missions.json
```



### Configuration

An example configuration file, `mir_config.yaml` has been provided. It has been generously commented, and in the cases where it has not, the parameter names are self-explanatory enough.



### Setting up building maps

**Chargers**

To use the `rmf_dock_and_charge` mission for charging, use the traffic-editor to set the `dock_name` property of your charging point to a json description like the following:
```json
{"description": {"end_waypoint": "charger_name"}, "mission_name": "rmf_dock_and_charge"}
```
Where you replace `end_waypoint` with the name of your MiR charger.


**MiR positions**

Upon launch, the MiR fleet adapter recognizes MiR positions with identical names to RMF waypoints to be the same location. Hence, when a navigation command is submitted for the robot to a specific waypoint, if this waypoint name also exists as a robot position on the MiR, the fleet adapter would send it directly to the MiR position even if the coordinates are different.


### Plugins

The MiR is capable of performing various types of custom missions and tasks. You can now easily set up plugins offered in this repo instead of writing your own perform action for common use cases.

For cart deliveries from point A to B:

**rmf_cart_delivery**

The `rmf_cart_delivery` plugin allows users to submit pickup and dropoff tasks to MiR integrated with RMF. The workflow of the task is as follows:
1. RMF will send the robot to the pickup lot
2. The robot will attempt to dock under a cart in the pickup lot
3. If the robot successfully docks under the correct cart, it will proceed to deliver it to a dropoff point. If the cart is missing or is not the desired cart, RMF will cancel the task.

Some relevant MiR missions (docking, exit, update footprint) will be automatically created on the MiR on startup. These missions are used to facilitate the pickup and dropoff activities and can be found in the plugin config under `missions`. They are:
- `rmf_dock_to_cart`: Docks robot under the cart
- `rmf_exit_lot`: Calls the robot to exit from under the cart
- `rmf_update_footprint`: Updates the robot footprint

They are defined and stored in the `rmf_cart_missions.json` file and do not require any further configuration.

However, since there are various types of latching methods available for different MiR models, users will need to set up their custom pickup and dropoff missions on the MiR:
1. Create 2 missions on the MiR:
   - `rmf_pickup_cart`: Triggers the robot's latching module to open
   - `rmf_dropoff_cart`: Triggers the robot's latching module to close and release the cart, then exit from under the cart (relative move -1 metre in the X-direction)
2. Fill in the MiR mission names in the plugin config under `missions`. The default mission names are `rmf_pickup_cart` and `rmf_dropoff_cart`.
3. Fill in the footprints and marker types to be used for your specific robot and cart in the plugin config.
4. In `rmf_cart_delivery.py`, fill in the logic to check whether the robot's latching module is open in blanks marked `# IMPLEMENT YOUR CODE HERE`. Some API calls to check the MiR's PLC registers and IO modules are provided in case you may want to use them.

To submit a cart delivery task, you may use the `dispatch_pickup` task script:
```bash
ros2 run fleet_adapter_mir_tasks dispatch_pickup -g go_to_waypoint -p pickup_lot -d dropoff_lot -c some_cart_id
```
- `-g`: Takes in an existing waypoint name for the robot to travel to before performing the pickup
- `-p`: Name of the pickup lot. This name should be identical to the shelf position configured on the MiR.
- `-d`: Name of the dropoff lot. This name should be identical to the robot or shelf position configured on the MiR.
- `-c`: Optional cart identifier for the fleet adapter to assess whether the cart is correct for pickup. 
