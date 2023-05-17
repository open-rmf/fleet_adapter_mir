from typing import TypedDict


class ReferenceCoordinates(TypedDict):
    rmf: list[tuple[float, float]]
    mir: list[tuple[float, float]]


class Limits(TypedDict):
    linear: tuple[float, float]  # velocity, acceleration
    angular: tuple[float, float]


class Profile(TypedDict):
    footprint: float
    vicinity: float


class BatterySystem(TypedDict):
    voltage: float
    capacity: float
    charging_current: float


class MechanicalSystem(TypedDict):
    mass: float  # kg
    moment_of_inertia: float
    friction_coefficient: float


class AmbientSystem(TypedDict):
    power: float


class CleaningSystem(TypedDict):
    power: float


class TaskCapabilities(TypedDict):
    loop: bool
    delivery: bool
    clean: bool
    finishing_request: str
    action_categories: list[str]


class FleetConfig(TypedDict):
    name: str
    limits: Limits
    profile: Profile
    battery_system: BatterySystem
    mechanical_system: MechanicalSystem
    ambient_system: AmbientSystem
    cleaning_system: CleaningSystem
    recharge_threshold: float
    recharge_soc: float
    account_for_battery_drain: bool
    publish_fleet_state: bool
    fleet_state_topic: str
    fleet_state_publish_frequency: int
    task_capabilities: TaskCapabilities


class NodeNames(TypedDict):
    robot_command_handle: str
    fleet_state_publisher: str
    rmf_fleet_adapter: str


class MirFmConfig(TypedDict):
    base_url: str
    username: str
    password: str
    robot_state_update_frequency: int  # Optional, defaults to 1
    max_merge_waypoint_distance: float
    max_merge_lane_distance: float
    # mapping between mir map names and rmf map names
    maps: dict[str, str]
    force_create_missions: bool


class Config(TypedDict):
    mirfm: MirFmConfig
    node_names: NodeNames
    reference_coordinates: dict[str, ReferenceCoordinates]
    rmf_fleet: FleetConfig
