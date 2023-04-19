## Usage

### Example Entry Point

An example usage of the implemented adapter can be found in the `fleet_adapter_mirfm/fleet_adapter_mirfm.py` file. It takes in a configuration file and navigation graph, sets everything up, and spins up the fleet adapter.

To test:
```bash
# Source the workspace
source ~/mir_ws/install/setup.bash

# Get help
ros2 run fleet_adapter_mirfm fleet_adapter_mirfm -h

# Run in dry-run mode. Disables ROS 2 publishing and MiR REST calls
cd ~/mir_ws/src/fleet_adapter_mir/configs
ros2 run fleet_adapter_mirfm fleet_adapter_mirfm -c mirfm_config.yaml -n nav_graph.yaml -d
```

Alternatively, if you want to run everything with full capabilities, (though note that it will require an `rmf_traffic_ros2` schedule node to be active, and the MiRFleet REST server to be available)

```bash
ros2 run fleet_adapter_mirfm fleet_adapter_mirfm -c mir_config.yaml -n nav_graph.yaml
```



### Configuration

An example configuration file, `mirfm_config.yaml` has been provided under the `configs/` folder in the root directory.