# fleet_adapter_mir
MiR100 Fleet Adapter using the https://github.com/osrf/rmf_fleet_adapter_python

 

## Pre-Requisites

- ROS2
- [rmf_core](https://github.com/osrf/rmf_core)
- [rmf_fleet_adapter_python](https://github.com/osrf/rmf_fleet_adapter_python)



## Installation

Place the pre-requisite packages into your ROS2 workspace, and run `colcon build` as per normal, remembering to source the workspace.



## Description

This package implements a MiR robot command handle that is managed by a fleet adapter in Python. (Along with some helpers.) It can be used to command and manage a fleet of MiR 100 robots using RMF!

It uses the `rmf_fleet_adapter_python` bindings, which allows for communication with `rmf_core` libraries and ROS2 nodes.

In effect, it interfaces the MiR REST API with `rmf_core`, all without needing to directly use ROS2 messages to communicate with `rmf_core`!



## Usage

### Example Entry Point

An example usage of the implemented adapter can be found in the `main.py` file. It takes in a configuration file, sets everything up, and spins up the fleet adapter.

Call it like so:

```shell
# Remember to source the workspace you've installed the pre-requisites in!

# Get help
$ python3 main.py -h

# Run in dry-run mode. Disables ROS2 publishing and MiR REST calls
$ python3 main.py mir_config.yaml -d
```

Alternatively, if you want to run everything with full capabilities, (though note that it will require an `rmf_core` schedule node to be active, and the MiR REST server to be available)

```shell
$ python3 main.py mir_config.yaml
```



### Configuration

An example configuration file, `mir_config.yaml` has been provided. It has been generously commented, and in the cases where it has not, the parameter names are self-explanatory enough.



## Additional Notes

### Different Units for Angles

MiR and RMF use different units for angles.

- MiR uses **degrees**
- RMF uses **radians**.

You might also have to clamp your angles to be within +-180 or +-3.14.



### Task IDs

Task IDs are kept as a monotonically increasing integer, with each robot having its own counter. They will be reflected as such in any `RobotState` messages that get published.



### Fleet States

`FleetState` publishing is done manually and not under the purview of the individual robot command handle instances. Each robot handle will internally update its corresponding `RobotState` message, so it is fairly trivial to aggregate them all into a `FleetState` message externally.

The example script `main.py` does this.



### Robot Transforms: Two Coordinate Systems

Since we are interfacing MiR and `rmf_core`, we need to deal with two coordinate systems and two forms of 'maps'.

We have to deal with the `rmf_core` navgraph and the MiR map, with their own individual coordinate systems, waypoints/positions, and waypoint keys/position names.

Luckily, we can leverage the `nudged` Python library to compute coordinate transforms between the two sets of coordinates. You just have to make sure to provide equivalent reference points within the two maps so transforms can be computed.

The transform objects are stored in the robot command handle's `self.transforms` member.



### Very Involved: Location Tracking

> **Note**: You do **not** need to know this if you just want to use this package to control robots. This is here to help any developers that want to look into the source code's guts to understand why the code is written the way it is (and why there seem to be weird variables strewn about.)
>
> This will be helpful if you intend on writing other fleet adapters for other robots that communicate with REST APIs.

In order for the appropriate `RobotUpdateHandle.update_position()` method calls to be used, we need some way to track the location of the robot on the `rmf_core` navigation graph.

The trouble is, since we are using the MiR REST API, the only way we can obtain information about the robot's state is by polling it via REST calls.

Even harder than that, the MiR REST API does not care to store information about the `rmf_core` navigation graph, and any goals that are set with respect ot that. In fact, the corresponding path following calls that `rmf_core` issues gets translated into move-to-coordinate missions for the MiR to execute.

This is **very messy**. Mainly because we cannot immediately infer the semantic state of the robot (with respect to RMF.) Instead, we have to infer it based off of what we know, from the tracking variables we are using, and the robot's mode.



#### **Trackers**

There are a couple of semantic states we need to be aware of, characterised by the state of the following tracker variables:

Three variables that keep track of location:

1. `current_waypoint`
2. `target_waypoint`
3. `current_lane`

The `RobotState` with the following robot modes:

- `MODE_MOVING` : Moving
- `MODE_CHARGING` : Charging
- `MODE_IDLE` : Ready
- `MODE_PAUSED`: Interrupted
- `MODE_DOCKING`: Docking

And a flag that gets set whenever the robot is executing a path following request from `rmf_core`:

- `rmf_path_requested`: `True` if executing a path request, `False` once complete



#### **Updating Position**

Additionally, there are four ways to update robot position. Listed in order of priority, or preference:

1. `update_position(waypoint, orientation)`
   - When the robot is at a waypoint
2. `update_position(position, lane)`
   - When the robot is in transit
3. `update_position(position, target_waypoint)`
   - When the robot is in transit but has lost track of the lane it is in
4. `update_position(map_name, position)`
   - When the robot is **completely lost**
   - Could potentially cause the robot to 'hop' around on the navigation graph, use as a last resort

> **Note:** Notice that this is a cool instance of method overloading that is invokable in Python! This is because of how the Python bindings were created! Different method calls are executed at the lower level!!



#### **Semantic States**

With this information, we can then infer the robot's meta/semantic state with respect to RMF, which will help inform us on what information is available to us, and hence what `update_position` call to use.

Then, we track how the semantic states transition as the robot goes about doing its business (changing the state of its tracking variables.)

Note, that in this case, all semantic states are **implicit** and are characterised and defined **purely by the state of the tracker variables listed above**!



#### **State Diagram**

Here's a state diagram to show how the states transition between themselves. But before that, here's the legend and some notes.



**State Diagram Legend**

```
<RobotMode> (current_lane, current_waypoint, target_waypoint)

For example, IDLE (0, 1, 0) implies:
- RobotMode: MODE_IDLE
- current_lane: None
- current_waypoint: Populated
- target_waypoint: None

If a state has multiple lines, the state might be in EITHER of those listed.

---

Additionally, any extra notes will be included after a --- in the description box
```



**Notes**

> **State Names**
>
> The state names are just there so we know intuitively what they mean. They do not represent actual variable names or values in the source code. Though they will be referenced in the comments so you know what they mean (hence semantic/meta states.)
>
> The states are characterised **ONLY** by the `RobotMode` and the status of the three tracking variables (which will be `None` or some `int`) and the `rmf_path_requested` variable (which will be `True` or `False`).
>
> 
>
> **Robot Modes**
>
> Robot modes will change independently as the command handle keeps running its internal `update_internal_robot_state()` update method. We only need to keep track of how our states are changing and adjust the internal trackers accordingly.
>
> 
>
> **Paused States**
>
> Paused states (interrupted) not included for simplicity.
>
> For those, just assume that any `target_waypoint` trackers will be `None`
>



**Diagram Time!**

![image-20200702030242372](_images/README/state_diagram.png)