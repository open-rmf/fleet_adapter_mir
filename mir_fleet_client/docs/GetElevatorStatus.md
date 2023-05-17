# GetElevatorStatus


## Properties
Name | Type | Description | Notes
------------ | ------------- | ------------- | -------------
**active** | **bool** | Boolean indicating if the elevator is active in the fleet | [optional] 
**connected** | **bool** | Boolean indicating if the elevator is connected | [optional] 
**control_requested** | **bool** | Boolean indicating if the fleet has requested control of the elevator | [optional] 
**current_floor** | **int** | The floor at which the elevator is currently | [optional] 
**door_1_open** | **bool** | Boolean indicating if door 1 of the elevator is open | [optional] 
**door_1_open_requested** | **bool** | Boolean indicating if door 1 is requested to be opened | [optional] 
**door_2_open** | **bool** | Boolean indicating if door 2 of the elevator is open | [optional] 
**door_2_open_requested** | **bool** | Boolean indicating if door 2 is requested to be opened | [optional] 
**floors** | [**[GetElevatorStatusFloorsInner]**](GetElevatorStatusFloorsInner.md) |  | [optional] 
**guid** | **str** | The guid of the elevator | [optional] 
**has_control** | **bool** | Boolean indicating, if the fleet has control of the elevator | [optional] 
**in_use** | **bool** | Boolean indicating, if the elevator is in use by a robot | [optional] 
**in_use_by_robot_ip** | **str** | The ip identifying the robot currently using the elevator | [optional] 
**ip** | **str** | The ip of the elevator | [optional] 
**name** | **str** | The name of the elevator | [optional] 
**queue** | **[int]** | List of all robot ids in queue for the elevator | [optional] 
**requested_floor** | **int** | The floor to which the fleet has requested the elevator to go | [optional] 
**any string name** | **bool, date, datetime, dict, float, int, list, str, none_type** | any string name can be used but the value must be the correct type | [optional]

[[Back to Model list]](../README.md#documentation-for-models) [[Back to API list]](../README.md#documentation-for-api-endpoints) [[Back to README]](../README.md)


