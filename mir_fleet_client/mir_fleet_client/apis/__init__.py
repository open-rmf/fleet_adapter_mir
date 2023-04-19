
# flake8: noqa

# Import all APIs into this package.
# If you have many APIs here with many many models used in each API this may
# raise a `RecursionError`.
# In order to avoid this, import only the API that you directly need like:
#
#   from mir_fleet_client.api.action_definitions_api import ActionDefinitionsApi
#
# or import this package, but before doing it, use:
#
#   import sys
#   sys.setrecursionlimit(n)

# Import APIs into API package:
from mir_fleet_client.api.action_definitions_api import ActionDefinitionsApi
from mir_fleet_client.api.actions_api import ActionsApi
from mir_fleet_client.api.advanced_api import AdvancedApi
from mir_fleet_client.api.area_events_api import AreaEventsApi
from mir_fleet_client.api.areas_api import AreasApi
from mir_fleet_client.api.auth_api import AuthApi
from mir_fleet_client.api.backups_api import BackupsApi
from mir_fleet_client.api.blocked_api import BlockedApi
from mir_fleet_client.api.cart_calibrations_api import CartCalibrationsApi
from mir_fleet_client.api.cart_types_api import CartTypesApi
from mir_fleet_client.api.carts_api import CartsApi
from mir_fleet_client.api.cert_api import CertApi
from mir_fleet_client.api.chargers_api import ChargersApi
from mir_fleet_client.api.charging_groups_api import ChargingGroupsApi
from mir_fleet_client.api.cmd_check_server_api import CmdCheckServerApi
from mir_fleet_client.api.cmd_control_api import CmdControlApi
from mir_fleet_client.api.cmd_door_api import CmdDoorApi
from mir_fleet_client.api.cmd_floor_api import CmdFloorApi
from mir_fleet_client.api.dashboards_api import DashboardsApi
from mir_fleet_client.api.definition_api import DefinitionApi
from mir_fleet_client.api.definitions_api import DefinitionsApi
from mir_fleet_client.api.docking_offsets_api import DockingOffsetsApi
from mir_fleet_client.api.download_api import DownloadApi
from mir_fleet_client.api.elevator_floors_api import ElevatorFloorsApi
from mir_fleet_client.api.elevator_groups_api import ElevatorGroupsApi
from mir_fleet_client.api.elevator_status_api import ElevatorStatusApi
from mir_fleet_client.api.elevators_api import ElevatorsApi
from mir_fleet_client.api.error_reports_api import ErrorReportsApi
from mir_fleet_client.api.evacuations_api import EvacuationsApi
from mir_fleet_client.api.export_api import ExportApi
from mir_fleet_client.api.factory_reset_api import FactoryResetApi
from mir_fleet_client.api.fire_alarms_api import FireAlarmsApi
from mir_fleet_client.api.footprints_api import FootprintsApi
from mir_fleet_client.api.helper_positions_api import HelperPositionsApi
from mir_fleet_client.api.import_api import ImportApi
from mir_fleet_client.api.info_api import InfoApi
from mir_fleet_client.api.log_api import LogApi
from mir_fleet_client.api.logs_api import LogsApi
from mir_fleet_client.api.maps_api import MapsApi
from mir_fleet_client.api.me_api import MeApi
from mir_fleet_client.api.metrics_api import MetricsApi
from mir_fleet_client.api.mission_groups_api import MissionGroupsApi
from mir_fleet_client.api.mission_scheduler_api import MissionSchedulerApi
from mir_fleet_client.api.missions_api import MissionsApi
from mir_fleet_client.api.opcua_scanner_api import OpcuaScannerApi
from mir_fleet_client.api.options_api import OptionsApi
from mir_fleet_client.api.path_guides_api import PathGuidesApi
from mir_fleet_client.api.path_guides_positions_api import PathGuidesPositionsApi
from mir_fleet_client.api.path_guides_precalc_api import PathGuidesPrecalcApi
from mir_fleet_client.api.paths_api import PathsApi
from mir_fleet_client.api.permissions_api import PermissionsApi
from mir_fleet_client.api.position_transition_lists_api import PositionTransitionListsApi
from mir_fleet_client.api.position_types_api import PositionTypesApi
from mir_fleet_client.api.positions_api import PositionsApi
from mir_fleet_client.api.prompt_api import PromptApi
from mir_fleet_client.api.qualified_robots_api import QualifiedRobotsApi
from mir_fleet_client.api.remote_support_api import RemoteSupportApi
from mir_fleet_client.api.reset_api import ResetApi
from mir_fleet_client.api.resources_api import ResourcesApi
from mir_fleet_client.api.restart_api import RestartApi
from mir_fleet_client.api.robot_groups_api import RobotGroupsApi
from mir_fleet_client.api.robots_api import RobotsApi
from mir_fleet_client.api.scan_api import ScanApi
from mir_fleet_client.api.service_book_api import ServiceBookApi
from mir_fleet_client.api.sessions_api import SessionsApi
from mir_fleet_client.api.setting_groups_api import SettingGroupsApi
from mir_fleet_client.api.settings_api import SettingsApi
from mir_fleet_client.api.shelfs_api import ShelfsApi
from mir_fleet_client.api.software_api import SoftwareApi
from mir_fleet_client.api.sounds_api import SoundsApi
from mir_fleet_client.api.ssl_api import SslApi
from mir_fleet_client.api.status_api import StatusApi
from mir_fleet_client.api.stream_api import StreamApi
from mir_fleet_client.api.sw_version_status_api import SwVersionStatusApi
from mir_fleet_client.api.system_api import SystemApi
from mir_fleet_client.api.types_api import TypesApi
from mir_fleet_client.api.upgrades_api import UpgradesApi
from mir_fleet_client.api.user_groups_api import UserGroupsApi
from mir_fleet_client.api.users_api import UsersApi
from mir_fleet_client.api.widgets_api import WidgetsApi
