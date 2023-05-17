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



### Configuration

An example configuration file, `mir_config.yaml` has been provided. It has been generously commented, and in the cases where it has not, the parameter names are self-explanatory enough.