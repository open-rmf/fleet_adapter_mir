## Usage

### Creating MiR missions for RMF

RMF relies on a number of MiR missions  to command the robot to move, perform map switches, and dock into chargers. These missions are pre-defined in `rmf_missions.json` and can be created automatically on the robots at startup.

For more details about the missions needed and created for RMF, refer to the `mir_missions` documentation found [here](https://github.com/open-rmf/fleet_adapter_mir/blob/main/docs/mir_missions.md). It will be helpful to go through the missions mentioned in the section before setting up your building map.

The next section will demonstrate how to parse this JSON file to the fleet adapter for MiR mission creation.


### Example Entry Point

To run the fleet adapter with full capabilities, (it will requirqe an `rmf_traffic_ros2` schedule node to be active, and the MiR REST server to be available):

```bash
# Source the workspace
source ~/mir_ws/install/setup.bash

# Get help
ros2 run fleet_adapter_mir fleet_adapter_mir -h

# Spin up the fleet adapter
cd ~/mir_ws/src/fleet_adapter_mir/configs
ros2 run fleet_adapter_mir fleet_adapter_mir -c mir_config.yaml -n nav_graph.yaml
```

If you have not configured the necessary RMF missions on the MiR, you may parse in the filepath to `rmf_missions.json` when launching the fleet adapter node. On startup, the fleet adapter will create these missions via MiR REST API. If a mission with the same Mission Group and Name already exists on the robot, the fleet adapter will skip creating it.

```bash
ros2 run fleet_adapter_mir fleet_adapter_mir -c mir_config.yaml -n nav_graph.yaml -r ../missions/rmf_missions.json
```


### Configuration

An example configuration file, `mir_config.yaml` has been provided. It has been generously commented, and in the cases where it has not, the parameter names are meant to be self-explanatory.



### Setting up building maps

**Chargers**

To use the `rmf_dock_and_charge` mission for charging, use the traffic-editor to set the `dock_name` property of your charging point to a json description like the following:
```json
{"description": {"end_waypoint": "charger_name"}, "mission_name": "rmf_dock_and_charge"}
```
Where you replace `charger_name` with the name of your MiR charger.


**MiR Positions**

Upon launch, the MiR fleet adapter recognizes MiR Positions with identical names to RMF waypoints to be the same location. Hence, when a navigation command is submitted for the robot to a specific waypoint, if this waypoint name also exists as a robot position on the MiR, the fleet adapter would send it directly to the MiR Position even if the coordinates are different.

Integrators should assume responsibility of making sure that the MiR Positions created on the robots and RMF waypoints with identical names are located reasonably close to one another.


**Guided MiR movement**

Robot travel through tight spaces can be especially tricky. Examples include lift entry, lift exit, or any situation where you might want the robot to move in a straight line over a short distance. One of the default MiR missions created with `rmf_missions.json`, `rmf_follow_line`, helps to ensure that the robot moves in a straight line without the need to create multiple RMF waypoints.

Using lift entry as an example scenario, you can make use of `rmf_follow_line` this way:
1. Create a MiR robot position right outside the lift, facing inwards. Let's call this MiR Position `Outside_Lift`.
2. Create a MiR robot position inside the lift, facing inwards. This position corresponds to the RMF waypoint inside this lift on this level. Let's call this MiR Position `Inside_Lift`. It helps immensely for both MiR Positions to form a straight line with their `Orientation` values facing the same direction as this line.
3. On the traffic-editor, set the `dock_name` property of the lift waypoint to a json description like this:
   ```json
   {"description": {"start_waypoint": "Outside_Lift", "end_waypoint": "Inside_Lift"}, "mission_name": "rmf_follow_line", "dock_name": "enter_lift"}
   ```
When the robot is called to enter the lift, regardless of where the robot was located before the mission starts, it will first move to `Outside_Lift`, facing directly towards the lift entry position, and then move to `Inside_Lift`.

With this mission, integrators have a little more control over the robot movement during travel through tight spaces.


### `MirAction` Plugins

The MiR is capable of performing various types of custom missions and tasks. You can now easily set up plugins offered in this repo instead of writing your own perform action for common use cases. The plugins offered are available under the `fleet_adapter_mir_actions` package.

You may refer to the `mir_actions` documentation found [here](https://github.com/open-rmf/fleet_adapter_mir/blob/main/docs/mir_actions.md) for a list of action plugins offered and how to set them up.
