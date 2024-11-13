## MiR Missions for RMF

RMF relies on MiR missions to command MiR robots to move and perform various actions. To standardize the missions used for RMF, users have the option to automatically create these missions required by RMF via the fleet adapter by parsing in the JSON filepath. This section lists and explains what these missions do.

Missions may be used for default behavior, which mostly involves the robot patrolling across the same or different maps. Additional missions can be optionally created for custom actions, such as pickup and dropoff missions for delivery tasks.

For more information on how to set up missions, actions and variables, please refer to [official guide documents](https://www.manualslib.com/manual/1941073/Mir-Mir250.html?page=150#manual) (this is for the MiR250).


### List of standardized RMF missions

* [Default RMF missions](#Default-RMF-missions)
* [RMF missions for `rmf_cart_delivery`](#RMF-missions-for-rmf_cart_delivery)


### Default RMF missions

Definition can be found in `rmf_missions.json`.

| Mission name   | Use case    | Created by RMF   |
| -------------- | ----------- | :--------------: |
| `rmf_dock_and_charge` | Enables the robot to dock into its charger and begin charging | Yes |
| `rmf_localize` | Switches MiR map once the robot reaches another level | Yes |
| `rmf_move` | Commands the robot to move to the MiR coordinates `[X, Y, Orientation]` | Yes |
| `rmf_move_to_position` | Commands the robot to move to the specified MiR Position | Yes |
| `rmf_follow_line` | Commands the robot to move to two specified MiR Positions in sequence | Yes |


### RMF missions for `rmf_cart_delivery`

| Mission name   | Use case    | Created by RMF   |
| -------------- | ----------- | :--------------: |
| `rmf_dock_to_cart` | Enables the robot to dock under a cart | Yes |
| `rmf_exit_lot` | Commands the robot to move backwards by 1 metre, for exiting from under a cart | Yes |
| `rmf_pickup_cart` | Commands the robot open its latch to attach to a cart | No |
| `rmf_dropoff_cart` | Commands the robot unlatch from a cart and move backwards to exit from under the cart | No |

**Important note:**
In order to ensure that your robot has the correct footprint after pickup/dropoff, do remember to add in the appropriate MiR actions to update robot footprint in your `rmf_pickup_cart` and `rmf_dropoff_cart` missions.
