# mir_fleet_client.CmdFloorApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**elevators_guid_cmd_floor_post**](CmdFloorApi.md#elevators_guid_cmd_floor_post) | **POST** /elevators/{guid}/cmd_floor | POST /elevators/{guid}/cmd_floor


# **elevators_guid_cmd_floor_post**
> bool, date, datetime, dict, float, int, list, str, none_type elevators_guid_cmd_floor_post(guid, elevator_cmd_floor)

POST /elevators/{guid}/cmd_floor

Request the elevator to go the a floor and open the door.

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import cmd_floor_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.post_elevator_cmd_floor import PostElevatorCmdFloor
from pprint import pprint
# Defining the host is optional and defaults to http://localhost
# See configuration.py for a list of all supported configuration parameters.
configuration = mir_fleet_client.Configuration(
    host = "http://localhost"
)

# The client must configure the authentication and authorization parameters
# in accordance with the API server security policy.
# Examples for each auth method are provided below, use the example that
# satisfies your auth use case.

# Configure HTTP basic authorization: Basic
configuration = mir_fleet_client.Configuration(
    username = 'YOUR_USERNAME',
    password = 'YOUR_PASSWORD'
)

# Enter a context with an instance of the API client
with mir_fleet_client.ApiClient(configuration) as api_client:
    # Create an instance of the API class
    api_instance = cmd_floor_api.CmdFloorApi(api_client)
    guid = "guid_example" # str | The guid to add the new resource to
    elevator_cmd_floor = PostElevatorCmdFloor(
        door_front=True,
        door_rear=True,
        floor=1,
    ) # PostElevatorCmdFloor | The details of the elevator_cmd_floor

    # example passing only required values which don't have defaults set
    try:
        # POST /elevators/{guid}/cmd_floor
        api_response = api_instance.elevators_guid_cmd_floor_post(guid, elevator_cmd_floor)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling CmdFloorApi->elevators_guid_cmd_floor_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to add the new resource to |
 **elevator_cmd_floor** | [**PostElevatorCmdFloor**](PostElevatorCmdFloor.md)| The details of the elevator_cmd_floor |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

**bool, date, datetime, dict, float, int, list, str, none_type**

### Authorization

[Basic](../README.md#Basic)

### HTTP request headers

 - **Content-Type**: application/json
 - **Accept**: application/json


### HTTP response details

| Status code | Description | Response headers |
|-------------|-------------|------------------|
**201** | The element has been created successfully |  -  |
**400** | Argument error or Missing content type application/json on the header or Bad request or Invalid JSON |  -  |
**409** | Duplicate entry |  -  |

[[Back to top]](#) [[Back to API list]](../README.md#documentation-for-api-endpoints) [[Back to Model list]](../README.md#documentation-for-models) [[Back to README]](../README.md)

