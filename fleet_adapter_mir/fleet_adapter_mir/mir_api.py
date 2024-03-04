from collections import namedtuple
import copy
import enum
import math
import requests
import json
from icecream import ic
from urllib.error import HTTPError

from rmf_adapter.easy_full_control import RobotState

__all__ = [
    "MirAPI"
]

MiRLocation = namedtuple("MiRLocation", ['x', 'y', 'yaw'])


class MiRStateCode(enum.IntEnum):
    READY = 3
    PAUSE = 4
    EXECUTING = 5
    MANUAL_CONTROL = 11
    ERROR = 12


class MiRPositionTypes(enum.IntEnum):
    ROBOT = 0
    SHELF = 5
    CHARGING_STATION = 20
    CHARGING_STATION_ENTRY = 21
    CART = 22
    LIFT = 25
    LIFT_ENTRY = 26


LocalizationParamPosition = 'position_estimate'

class MapConversions:
    def __init__(self, rmf_to_mir: dict):
        self.rmf_to_mir = rmf_to_mir
        self.mir_to_rmf = {v: k for k, v in rmf_to_mir.items()}


class MirStatus:
    def __init__(self, response: dict, map_conversions: MapConversions, map_name: str):
        self.response = response
        p = response['position']
        self.state = RobotState(
            map_conversions.mir_to_rmf[map_name],
            [p['x'], p['y'], math.radians(p['orientation'])],
            response['battery_percentage']/100,
        )


class MirAPI:
    def __init__(self, prefix, headers, conversions, rmf_missions, timeout=10.0, debug=False):
        #HTTP connection
        self.prefix =  prefix
        self.debug = False
        self.headers = headers
        self.timeout = timeout
        self.connected = False
        self.rmf_missions = rmf_missions
        self.map_conversions = MapConversions(conversions['maps'])
        self.known_missions = {}
        self.known_positions = {}
        self.known_maps = {}
        self.mission_keys: dict = conversions['missions']
        self.mission_actions: dict = {}
        self.mission_action_types: dict = {}
        self.marker_type_keys: dict = conversions['marker_types']
        self.attempt_connection()

    def attempt_connection(self):
        try:
            requests.get(self.prefix + 'wifi/connections', headers=self.headers, timeout=self.timeout)
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
            return
        except Exception as err:
            print(f"Other error: {err}")
            return

        self.connected = True

        self.load_missions()
        self.load_maps()
        self.create_missions(self.rmf_missions)

        move_key = 'move'
        assert move_key in self.mission_keys, (
            f'{move_key} mission must be specified in fleet config file under '
            f'conversions -> missions'
        )
        self.move_mission: str = self.mission_keys[move_key]
        assert self.move_mission in self.known_missions, (
            f'RMF move mission [{self.move_mission}] has not yet been '
            f'defined as a mission in the MiR robot [{self.prefix}]'
        )

        dock_and_charge_key = 'dock_and_charge'
        assert dock_and_charge_key in self.mission_keys, (
            f'{dock_and_charge_key} mission must be specified in fleet config '
            f'file under conversions -> missions'
        )
        self.dock_and_charge_mission: str = self.mission_keys[dock_and_charge_key]
        assert self.dock_and_charge_mission in self.known_missions, (
            f'RMF dock and charge mission [{self.dock_and_charge_mission}] '
            f'has not yet been defined as a mission in the MiR robot '
            f'[{self.prefix}]'
        )

        def make_optional_mission(key):
            target_mission = None
            if key in self.mission_keys:
                target_mission = self.mission_keys[key]
                assert target_mission in self.known_missions, (
                    f'RMF {key} mission [{target_mission}] has not yet been '
                    f'defined as a mission in the MiR robot [{self.prefix}]'
                )
            return target_mission

        self.localize_mission: str | None = make_optional_mission('localize')
        self.go_to: str | None = make_optional_mission('go_to')

        if self.localize_mission is not None:
            localize_actions = self.missions_mission_id_actions_get(
                self.known_missions[self.localize_mission]['guid']
            )
            assert localize_actions is not None, (
                f'{self.localize_mission} mission does not have any actions'
            )
            self.localize_params = None
            for action in localize_actions:
                if action.get('action_type') == 'switch_map':
                    self.localize_params = action['parameters']
                    break
            assert self.localize_params is not None, (
                f'No switch_map action in the mission {self.localize_mission}:\n'
                f'{localize_actions}'
            )
            found_position_estimate_param = False
            for param in self.localize_params:
                if param.get('input_name') == LocalizationParamPosition:
                    found_position_estimate_param = True
            assert found_position_estimate_param, (
                f'No {LocalizationParamPosition} parameter found in the '
                f'mission {self.localize_mission}:\n{localize_actions}'
            )

        # Retrieve mission parameters
        for mission, mission_data in self.known_missions.items():
            self.mission_actions[mission] = self.missions_mission_id_actions_get(
                mission_data['guid'])

        self.created_by_id = self.me_get()['guid']

    def update_known_positions(self):
        self.known_positions = {}
        for pos in self.positions_get():
            if pos['name'] in self.known_positions and \
                    pos['type_id'] == self.known_positions[pos['name']]['type_id'] and \
                    ('rmf_localize' in pos['name']):
                # Delete any duplicate positions
                self.positions_guid_delete(pos['guid'])
            elif pos['type_id'] == MiRPositionTypes.ROBOT or \
                    pos['type_id'] == MiRPositionTypes.SHELF or \
                    pos['type_id'] == MiRPositionTypes.CHARGING_STATION or \
                    pos['type_id'] == MiRPositionTypes.LIFT or \
                    pos['type_id'] == MiRPositionTypes.LIFT_ENTRY:
                self.known_positions[pos['name']] = (
                    self.positions_guid_get(pos['guid'])
                )

    def create_missions(self, rmf_missions):
        if rmf_missions is None:
            return

        # Retrieve the RMF group id if it already exists
        mission_groups = self.mission_groups_get()
        rmf_mission_group_id = None
        mission_group_name = 'RMF'
        for group in mission_groups:
            if group['name'] == mission_group_name:
                rmf_mission_group_id = group['guid']
                break
        # If the RMF mission group doesn't exist, create one
        if rmf_mission_group_id is None:
            mission_group = self.mission_groups_post(mission_group_name)
            rmf_mission_group_id = mission_group['guid']

        # Create RMF missions if they don't exist on the robot
        for mission_name, mission_json in rmf_missions.items():
            if mission_name not in self.known_missions:
                # Create the relevant mission on MiR
                mission = self.missions_post(mission_name, rmf_mission_group_id)
                self.known_missions[mission['name']] = mission
                # Fill in mission actions
                self.create_rmf_mission_actions(mission_name, mission['guid'], mission_json)

    def create_rmf_mission_actions(self, mission_name: str, mission_id: str, mission_json: list[dict]):
        for action in mission_json:
            # Find schema for action type
            action_type = action.get('action_type')
            action_type_schema = self.mission_action_types.get(action_type)
            if action_type_schema is None:
                action_type_schema = self.action_type_get(action_type)
                self.mission_action_types[action_type] = action_type_schema

            action_body_param = []
            id_tracker = []
            # First, add the parameters that we have customized for RMF
            for param in action['parameters']:
                id_tracker.append(param['id'])
                param_body = {}
                param_body['id'] = param['id']
                param_body['input_name'] = param['input_name']
                param_body['value'] = param['value']

                # If the input type is a position, we would need a valid GUID for the value field
                # Let's find a placeholder position GUID from the list of known positions
                if param_body['id'] == 'position' or param_body['id'] == 'entry_position':
                    default_pos = self.positions_get()[0]['guid']
                    param_body['value'] = default_pos
                # Similarly, if the input type is a marker type, we'll need to find a placeholder
                # marker type GUID from the list of known marker types
                if param_body['id'] == 'marker_type':
                    default_marker = self.docking_offsets_get()[0]['guid']
                    param_body['value'] = default_marker
                # If the input type is a footprint, we'll also use a placeholder footprint GUID
                if param_body['id'] == 'footprint':
                    default_footprint = self.footprints_get()[0]['guid']
                    param_body['value'] = default_footprint

                action_body_param.append(param_body)

            # Next, add the default parameters that come with this action
            for param in action_type_schema['parameters']:
                if param['id'] in id_tracker:
                    continue
                id_tracker.append(param['id'])

                param_body = {}
                param_body['id'] = param['id']
                param_body['input_name'] = param.get('input_name')

                default_value = None
                if 'constraints' in param and 'default' in param['constraints']:
                    default_value = param['constraints']['default']
                param_body['value'] = default_value
                action_body_param.append(param_body)

            action_body = {}
            action_body['action_type'] = action_type
            action_body['priority'] = action.get('priority')
            action_body['parameters'] = action_body_param
            action_body['mission_id'] = mission_id
            self.missions_mission_id_actions_post(mission_id, action_body)

    def navigate(self, position):
        pos_x = round(position[0], 3)
        pos_y = round(position[1], 3)
        ori_deg = position[2]*180/math.pi
        mission_params = [
            {'id': 'X', 'value': pos_x, 'label': f'{pos_x}'},
            {'id': 'Y', 'value': pos_y, 'label': f'{pos_y}'},
            {'id': 'Orientation', 'value': ori_deg, 'label': f'{ori_deg:.3f}'}
        ]
        return self.queue_mission_by_name(self.move_mission, mission_params)

    def go_to_known_position(self, position_name):
        if not position_name in self.known_positions:
            return None
        return self.dock(self.go_to, None, position_name)

    def dock(self, mission_name, start_waypoint, end_waypoint):
        mission_params = None

        # Get parameters for start and end waypoints
        end_wp = self.known_positions.get(end_waypoint)
        if not end_wp:
            self.update_known_positions()
            end_wp = self.known_positions.get(end_waypoint)
        assert end_wp is not None
        end_wp_guid = end_wp.get('guid')
        assert end_wp_guid is not None
        end_param = self.get_mission_params_with_value(mission_name, 'move', 'end_waypoint', end_wp_guid)
        dock_param = self.get_mission_params_with_value(mission_name, 'docking', 'end_waypoint', end_wp_guid)

        # Start waypoint is optional
        if start_waypoint is not None:
            start_wp = self.known_positions.get(start_waypoint)
            if not start_wp:
                self.update_known_positions()
                start_wp = self.known_positions.get(start_waypoint)
            assert start_wp is not None
            start_wp_guid = start_wp.get('guid')
            assert start_wp_guid is not None
            start_param = self.get_mission_params_with_value(mission_name, 'move', 'start_waypoint', start_wp_guid)
            mission_params = start_param + end_param

        # Check whether we should dock into this end waypoint or not (for charging)
        elif dock_param:
            charger_marker_type = self.marker_type_keys['charger']
            charger_marker_type_guid = self.docking_offsets_guid_get(charger_marker_type)
            marker_param = self.get_mission_params_with_value(mission_name, 'docking', 'charger_marker_type', charger_marker_type_guid)
            mission_params = end_param + dock_param + marker_param
        else:
            mission_params = end_param
        return self.queue_mission_by_name(mission_name, mission_params)

    def localize(self, map, estimate, index):
        if self.localize_mission is None:
            raise Exception(
                'The fleet was not configured with a localize mission'
            )

        if index is not None:
            position_name = f'rmf_localize_{index}'
        else:
            p = estimate
            position_name = f'rmf_localize_{map}_{p[0]:.2f}_{p[1]:.2f}'
        mir_map = self.map_conversions.rmf_to_mir[map]
        map_id = self.known_maps[mir_map]
        position_guid = self.get_position_guid(position_name, map_id, estimate)
        if position_guid is None:
            raise Exception('Unable to set a localization position on the MiR')
        mission_params = copy.copy(self.localize_params)
        for param in mission_params:
            if param.get('input_name') == LocalizationParamPosition:
                param['value'] = position_guid

        return self.queue_mission_by_name(
            self.localize_mission,
            mission_params=mission_params
        )


    def get_position_guid(self, name, map_id, location):
        attempts = 0
        max_attempts = 10
        while True:
            attempts +=1
            if attempts >= max_attempts:
                print(
                    f'Too many attempts [{max_attempts}] to set a localization '
                    'position.'
                )
                return None

            # We keep cycling through these attempts until we can confirm that
            # the MiR server has an acceptable position for us to use for
            # localization
            position = self.known_positions.get(name)

            def position_matches(pos):
                if abs(pos['pos_x'] - location[0]) > 0.01:
                    return False
                if abs(pos['pos_y'] - location[1]) > 0.01:
                    return False
                if pos['map_id'] != map_id:
                    return False
                return True

            if position is None:
                # The position does not exist so we need to create a new one
                position_guid = self.positions_post(name, map_id, location)
                if position_guid is not None:
                    return position_guid
                # For some reason posting the new position failed. Maybe there
                # was a timeout or an argument error. We will update the known
                # positions and retry this loop.
                self.update_known_positions()
            elif not position_matches(position):
                if self.positions_put(position['guid'], name, map_id, location):
                    return position['guid']
            else:
                return position['guid']


    def queue_mission_by_name(self, mission_name, mission_params=None):
        mir_mission = self.known_missions.get(mission_name)
        if mir_mission is None:
            return None
        mission_guid = mir_mission['guid']
        mission_description = None
        if mission_params:
            mission_description = {
                'mission_id': mission_guid,
                'message': 'string',
                'parameters': mission_params,
                'priority': 0,
                'description': 'string'
            }

        response = self.mission_queue_post(mission_guid, mission_description)
        if response is None or 'id' not in response:
            return None
        mission_queue_id = response['id']
        return mission_queue_id

    def positions_post(self, name, map_id, location):
        data = {
            'created_by_id': self.created_by_id,
            'map_id': map_id,
            'name': name,
            'orientation': math.degrees(location[2]),
            'pos_x': location[0],
            'pos_y': location[1],
            'type_id': MiRPositionTypes.ROBOT
        }

        try:
            response = requests.post(
                self.prefix + "positions",
                headers=self.headers,
                data=json.dumps(data),
                timeout=self.timeout
            ).json()
            if response is None or 'guid' not in response:
                return None
            if self.debug:
                print(f'Response: {response}')
            self.known_positions[response['name']] = response
            return response['guid']
        except Exception as err:
            print(f'Position post failed: {err}')

    def positions_put(self, guid, name, map_id, location):
        data = {
            'map_id': map_id,
            'name': name,
            'orientation': math.degrees(location[2]),
            'pos_x': location[0],
            'pos_y': location[1],
            'type_id': MiRPositionTypes.ROBOT
        }

        try:
            response = requests.put(
                self.prefix + f'positions/{guid}',
                headers=self.headers,
                data=json.dumps(data),
                timeout=self.timeout
            ).json()
            if self.debug:
                print(f'Response: {response}')
            self.known_positions[response['name']] = response
            return response['guid']
        except:
            pass

    def positions_delete(self):
        new_known_positions = {}
        for position in self.known_positions:
            if 'rmf_localize' in position:
                pos_guid = self.known_positions[position]
                try:
                    response = requests.delete(
                        self.prefix + f'positions/{pos_guid}',
                        headers=self.headers,
                        timeout=self.timeout
                    ).json()
                    return response['guid']
                except:
                    pass
            else:
                new_known_positions[position] = copy.deepcopy(self.known_positions[position])
        self.known_positions = {}
        self.known_positions = new_known_positions

    def positions_guid_delete(self, guid):
        try:
            response = requests.delete(
                self.prefix  + f'positions/{guid}',
                headers=self.headers,
                timeout=self.timeout
            ).json()
            if self.debug:
                print(f'Response: {response}')
        except Exception as err:
            print(f'Failed to delete position guid [{guid}]: {err}')

    def status_get(self) -> MirStatus:
        if not self.connected:
            return
        try:
            response = requests.get(self.prefix + f'status', headers=self.headers, timeout=self.timeout)
            # To prevent adapter crashing in case of error
            if response.json() is None or 'position' not in response.json():
                return None
            map_id = response.json()['map_id']
            map_name = None
            for mname, mid in self.known_maps.items():
                if mid == map_id:
                    map_name = mname
                    break

            return MirStatus(response.json(), self.map_conversions, map_name)
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")

    def load_missions(self):
        robot_missions = self.missions_get()
        for mission in robot_missions:
            if 'move_coordinate' in mission['name']:
                self.missions_guid_delete(mission['guid'])
            else:
                self.known_missions[mission['name']] = mission

    def load_maps(self):
        robot_maps = self.maps_get()
        for map in robot_maps:
            self.known_maps[map['name']] = map['guid']

    def me_get(self):
        if not self.connected:
            return
        try:
            response = requests.get(self.prefix + 'users/me', headers=self.headers, timeout=self.timeout)
            if self.debug:
                print(f'Response: {response.json()}')
            return response.json()
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')
        except Exception as err:
            print(f'Other error: {err}')

    def mission_groups_get(self):
        if not self.connected:
            return
        try:
            response = requests.get(self.prefix + f'mission_groups', headers = self.headers, timeout=self.timeout)
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other here error: {err}")

    def actions_get(self):
        if not self.connected:
            return
        try:
            response = requests.get(self.prefix + 'actions', headers=self.headers, timeout=self.timeout)
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")

    def action_type_get(self, action_type: str):
        if not self.connected:
            return
        try:
            response = requests.get(self.prefix + f'actions/{action_type}', headers=self.headers, timeout=self.timeout)
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")

    def missions_get(self):
        if not self.connected:
            return
        try:
            response = requests.get(self.prefix + 'missions', headers=self.headers, timeout=self.timeout)
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")

    def positions_get(self):
        if not self.connected:
            return
        try:
            response = requests.get(self.prefix + 'positions' , headers=self.headers, timeout=self.timeout)
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")

    def mission_queue_get(self):
        if not self.connected:
            return
        try:
            response = requests.get(self.prefix + f'mission_queue', headers = self.headers, timeout=self.timeout)
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other here error: {err}")

    def mission_queue_id_get(self, mission_queue_id):
        if not self.connected:
            return
        try:
            response = requests.get(self.prefix + f'mission_queue/{mission_queue_id}', headers = self.headers, timeout=self.timeout)
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other here error: {err}")

    def mission_queue_post(self, mission_id, full_mission_description=None):
        if not self.connected:
            return
        data = {'mission_id': mission_id}
        if full_mission_description is not None:
            # print(f'---------->>> {full_mission_description}')
            data = full_mission_description
            if mission_id != full_mission_description['mission_id']:
                print(f'Inconsistent mission id, provided [{mission_id}], full_mission_description: [{full_mission_description}]')
                return

        try:
            response = requests.post(self.prefix + 'mission_queue' , headers = self.headers, data=json.dumps(data), timeout = self.timeout)
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other here error: {err}")

    def missions_mission_id_actions_post(self, mission_id, body):
        if not self.connected:
            return
        try:
            response = requests.post(self.prefix + 'missions/' + mission_id +'/actions' , headers = self.headers, data=json.dumps(body), timeout = self.timeout)
            if self.debug:
                print(f"Response: {response.json()}")
                print(response.status_code)
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")

    def missions_mission_id_actions_get(self, mission_id):
        if not self.connected:
            return
        try:
            response = requests.get(self.prefix + 'missions/' + str(mission_id) +'/actions' , headers = self.headers, timeout = self.timeout)
            if self.debug:
                print(f"Response: {response.json()}")
                print(response.status_code)
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")

    def mission_groups_post(self, group_name):
        if not self.connected:
            return
        data = {
            'name': group_name,
            'priority': 0,
            'feature': 'default',
            'icon': ''
        }
        try:
            response = requests.post(self.prefix + 'mission_groups' , headers = self.headers, data=json.dumps(data), timeout = self.timeout)
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")

    def missions_post(self, mission, mission_group_id):
        if not self.connected:
            return
        data = {
            'name': mission,
            'hidden': False,
            'group_id': mission_group_id
        }
        try:
            response = requests.post(self.prefix + 'missions' , headers = self.headers, data=json.dumps(data), timeout = self.timeout)
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")

    def positions_guid_get(self, guid):
        if not self.connected:
            return
        try:
            response = requests.get(self.prefix + 'positions/'+ guid, headers=self.headers, timeout=self.timeout)
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")

    def status_put(self, state_id):
        if not self.connected:
            return
        data = {"state_id": state_id}
        try:
            response = requests.put(self.prefix + 'status', headers = self.headers, data=json.dumps(data), timeout=self.timeout)
            if self.debug:
                  print(f"Response: {response.json()}")
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")

    def clear_error(self):
        if not self.connected:
            return
        data = {"clear_error": True}
        try:
            response = requests.put(self.prefix + 'status', headers = self.headers, data=json.dumps(data), timeout=self.timeout)
            if self.debug:
                  print(f"Response: {response.json()}")
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")

    def mission_queue_delete(self):
        if not self.connected:
            return
        try:
            response = requests.delete(self.prefix + 'mission_queue' , headers = self.headers, timeout = self.timeout)
            if self.debug:
                print(f"Response: {response.headers}")
            return True
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
            return False
        except Exception as err:
            print(f"Other error: {err}")
            return False

    def mission_queue_id_delete(self, mission_queue_id):
        if not self.connected:
            return
        try:
            response = requests.delete(
                self.prefix + 'mission_queue/' + str(mission_queue_id),
                headers = self.headers,
                timeout = self.timeout
            )
            if self.debug:
                print(f"Response: {response.headers}")
            return True
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
            return False
        except Exception as err:
            print(f"Other error: {err}")
            return False

    def missions_guid_delete(self, guid):
        if not self.connected:
            return
        try:
            response = requests.delete(self.prefix + 'missions/' +guid , headers = self.headers, timeout = self.timeout)
            if self.debug:
                print(f"Response: {response.headers}")
            return True
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
            return False
        except Exception as err:
            print(f"Other  error: {err}")
            return False

    def maps_get(self):
        if not self.connected:
            return []
        try:
            response = requests.get(self.prefix + 'maps', headers = self.headers, timeout = self.timeout)
            if self.debug:
                print(f"Response: {response.headers}")
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
            return []
        except Exception as err:
            print(f"Other  error: {err}")
            return []

    def docking_offsets_get(self):
        if not self.connected:
            return
        try:
            response = requests.get(self.prefix + 'docking_offsets', headers=self.headers, timeout=self.timeout)
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")

    def docking_offsets_guid_get(self, offset_name: str):
        offsets = self.docking_offsets_get()
        for offs in offsets:
            if offs['name'] == offset_name:
                return offs['guid']
        return None

    def footprints_get(self):
        if not self.connected:
            return
        try:
            response = requests.get(self.prefix + 'footprints', headers=self.headers, timeout=self.timeout)
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")

    def footprints_guid_get(self, footprint_name: str):
        footprints = self.footprints_get()
        for ft in footprints:
            if ft['name'] == footprint_name:
                return ft['guid']
        return None

    def mission_completed(self, mission_queue_id):
        mission_status = self.mission_queue_id_get(mission_queue_id)
        if not mission_status:
            return False
        if 'finished' in mission_status and mission_status['finished'] is not None:
        # if 'state' in mission_status and mission_status['state'] == 'Done':
            return True
        return False

    def get_mission_params_with_value(self, mission_name: str, action_type: str, param_name: str, value: str):
        mission_actions = self.mission_actions.get(mission_name)
        mission_params = None
        for d in mission_actions:
            if 'action_type' in d and d['action_type'] == action_type:
                mission_params = d['parameters']
                for p in mission_params:
                    if 'input_name' in p and p['input_name'] == param_name:
                        p['value'] = value
                        # Let's only include this mission param to avoid error
                        mission_params = [p]
                        return mission_params
        return mission_params
