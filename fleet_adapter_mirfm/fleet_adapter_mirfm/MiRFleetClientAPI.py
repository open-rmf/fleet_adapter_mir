import requests
import json
from urllib.error import HTTPError

from fleet_adapter_mir.MiRClientAPI import MirAPI


__all__ = [
    "MirAPI"
]


class MirFleetAPI(MirAPI):
    def __init__(self, prefix, headers, timeout=10.0, debug=False):
        MirAPI.__init__(prefix, headers, timeout, debug)

    def status_put2(self, status: dict):
        if not self.connected:
            return
        try:
            response = requests.put(self.prefix + 'status', headers = self.headers, json=status, timeout=self.timeout)
            if self.debug:
                  print(f"Response: {response.json()}")
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")

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
