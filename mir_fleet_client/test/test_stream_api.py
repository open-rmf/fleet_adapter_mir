"""
    2.13.5.3 FLEET REST API

    The REST API for the 2.13.5.3 interface of FLEET  # noqa: E501

    The version of the OpenAPI document: 2.13.5.3
    Contact: support@mir-robots.com
    Generated by: https://openapi-generator.tech
"""


import unittest

import mir_fleet_client
from mir_fleet_client.api.stream_api import StreamApi  # noqa: E501


class TestStreamApi(unittest.TestCase):
    """StreamApi unit test stubs"""

    def setUp(self):
        self.api = StreamApi()  # noqa: E501

    def tearDown(self):
        pass

    def test_sounds_guid_stream_get(self):
        """Test case for sounds_guid_stream_get

        GET /sounds/{guid}/stream  # noqa: E501
        """
        pass


if __name__ == '__main__':
    unittest.main()