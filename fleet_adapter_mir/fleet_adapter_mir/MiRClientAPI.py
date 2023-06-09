import requests
import json
from urllib.error import HTTPError


__all__ = [
    "MirAPI"
]


class MirAPI:
    def __init__(self, prefix, headers, timeout=10.0, debug=False):
        #HTTP connection
        self.prefix =  prefix
        self.debug = debug
        self.headers = headers
        self.timeout = timeout
        self.connected = False

        # Test connectivity
        try:
            response = requests.get(self.prefix + 'wifi/connections', headers=self.headers, timeout=self.timeout)
            self.connected = True
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")

    def status_get(self):
        if not self.connected:
            return
        try:
            response = requests.get(self.prefix + f'status', headers=self.headers, timeout=self.timeout)
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")

    def missions_get(self):
        if not self.connected:
            return
        try:
            response = requests.get(self.prefix + 'missions', headers = self.headers, timeout = 1.0)
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

    def missions_mission_id_actions_post(self,mission_id,body):
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

    def missions_post(self, mission):
        if not self.connected:
            return
        try:
            response = requests.post(self.prefix + 'missions' , headers = self.headers, data=mission, timeout = self.timeout)
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
            response = requests.put(self.prefix + 'status', headers = self.headers, data=data, timeout=self.timeout)
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
            response = requests.delete(self.prefix + 'missions' , headers = self.headers, timeout = self.timeout)
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
