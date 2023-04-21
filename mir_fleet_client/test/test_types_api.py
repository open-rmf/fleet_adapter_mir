"""
    2.13.5.3 FLEET REST API

    The REST API for the 2.13.5.3 interface of FLEET  # noqa: E501

    The version of the OpenAPI document: 2.13.5.3
    Contact: support@mir-robots.com
    Generated by: https://openapi-generator.tech
"""


import unittest

import mir_fleet_client
from mir_fleet_client.api.types_api import TypesApi  # noqa: E501


class TestTypesApi(unittest.TestCase):
    """TypesApi unit test stubs"""

    def setUp(self):
        self.api = TypesApi()  # noqa: E501

    def tearDown(self):
        pass

    def test_docking_offsets_types_get(self):
        """Test case for docking_offsets_types_get

        GET /docking_offsets/types  # noqa: E501
        """
        pass

    def test_docking_offsets_types_id_get(self):
        """Test case for docking_offsets_types_id_get

        GET /docking_offsets/types/{id}  # noqa: E501
        """
        pass


if __name__ == '__main__':
    unittest.main()