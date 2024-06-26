import requests
from urllib.error import HTTPError

class CartDetection:
    def __init__(
            self,
            mir_api,
            action_config
    ):
        self.api = mir_api
        self.action_config = action_config

    def is_latch_open(self):
        '''
        Checks if the robot's latch is open and carrying a cart
        Return True if latch is open, else False
        '''
        # ------------------------
        # IMPLEMENT YOUR CODE HERE
        # ------------------------
        return False

    def is_under_cart(self):
        '''
        Checks if the robot is docked under a cart
        Return True if robot is under any carts, else False
        '''
        # ------------------------
        # IMPLEMENT YOUR CODE HERE
        # ------------------------
        return False

    def is_correct_cart(self, cart_id: str):
        '''
        Checks if the detected cart identifier matches the target cart_id
        Return True if cart is correct, False if cart is wrong, None if no
        cart detected
        '''
        # ------------------------
        # IMPLEMENT YOUR CODE HERE
        # ------------------------
        return None

    # --------------------------------------------------------------------------
    # HELPFUL FUNCTIONS FOR INTERACTING WITH MIR REST API
    # --------------------------------------------------------------------------

    def register_get(self, register: int):
        if not self.api.connected:
            return None
        try:
            response = requests.get(self.api.prefix + f'registers/{register}',
                                    headers=self.api.headers,
                                    timeout=self.api.timeout)
            if self.api.debug:
                print(f"Response: {response.headers}")
            # Response value is string, return integer of value
            return int(response.json().get('value', 0))
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
            return None
        except Exception as err:
            print(f"Other  error: {err}")
            return None

    def io_module_guid_status_get(self, io_guid: str):
        if not self.api.connected:
            return None
        if io_guid is None:
            return None
        try:
            response = requests.get(self.api.prefix +
                                    f'io_modules/{io_guid}/status',
                                    headers=self.api.headers,
                                    timeout=self.api.timeout)
            if self.api.debug:
                print(f"Response: {response.headers}")
            if 'input_state' not in response.json():
                return None
            return response.json()['input_state']
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
            return None
        except Exception as err:
            print(f"Other  error: {err}")
            return None

    def io_modules_get(self):
        if not self.api.connected:
            return None
        try:
            response = requests.get(self.api.prefix + f'io_modules',
                                    headers=self.api.headers,
                                    timeout=self.api.timeout)
            if self.api.debug:
                print(f"Response: {response.headers}")
            # Response value is string, return integer of value
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
            return None
        except Exception as err:
            print(f"Other  error: {err}")
            return None