"""
    2.13.5.3 FLEET REST API

    The REST API for the 2.13.5.3 interface of FLEET  # noqa: E501

    The version of the OpenAPI document: 2.13.5.3
    Contact: support@mir-robots.com
    Generated by: https://openapi-generator.tech
"""


import unittest

import mir_fleet_client
from mir_fleet_client.api.widgets_api import WidgetsApi  # noqa: E501


class TestWidgetsApi(unittest.TestCase):
    """WidgetsApi unit test stubs"""

    def setUp(self):
        self.api = WidgetsApi()  # noqa: E501

    def tearDown(self):
        pass

    def test_dashboards_dashboard_id_widgets_get(self):
        """Test case for dashboards_dashboard_id_widgets_get

        GET /dashboards/{dashboard_id}/widgets  # noqa: E501
        """
        pass

    def test_dashboards_dashboard_id_widgets_guid_delete(self):
        """Test case for dashboards_dashboard_id_widgets_guid_delete

        DELETE /dashboards/{dashboard_id}/widgets/{guid}  # noqa: E501
        """
        pass

    def test_dashboards_dashboard_id_widgets_guid_get(self):
        """Test case for dashboards_dashboard_id_widgets_guid_get

        GET /dashboards/{dashboard_id}/widgets/{guid}  # noqa: E501
        """
        pass

    def test_dashboards_dashboard_id_widgets_guid_put(self):
        """Test case for dashboards_dashboard_id_widgets_guid_put

        PUT /dashboards/{dashboard_id}/widgets/{guid}  # noqa: E501
        """
        pass

    def test_dashboards_dashboard_id_widgets_post(self):
        """Test case for dashboards_dashboard_id_widgets_post

        POST /dashboards/{dashboard_id}/widgets  # noqa: E501
        """
        pass


if __name__ == '__main__':
    unittest.main()