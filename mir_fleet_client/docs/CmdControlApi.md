# mir_fleet_client.CmdControlApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**elevators_guid_cmd_control_post**](CmdControlApi.md#elevators_guid_cmd_control_post) | **POST** /elevators/{guid}/cmd_control | POST /elevators/{guid}/cmd_control


# **elevators_guid_cmd_control_post**
> bool, date, datetime, dict, float, int, list, str, none_type elevators_guid_cmd_control_post(guid, elevator_cmd_control)

POST /elevators/{guid}/cmd_control

Request a control of the elevator.

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import cmd_control_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.post_elevator_cmd_control import PostElevatorCmdControl
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
    api_instance = cmd_control_api.CmdControlApi(api_client)
    guid = "guid_example" # str | The guid to add the new resource to
    elevator_cmd_control = PostElevatorCmdControl(
        request=True,
    ) # PostElevatorCmdControl | The details of the elevator_cmd_control

    # example passing only required values which don't have defaults set
    try:
        # POST /elevators/{guid}/cmd_control
        api_response = api_instance.elevators_guid_cmd_control_post(guid, elevator_cmd_control)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling CmdControlApi->elevators_guid_cmd_control_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to add the new resource to |
 **elevator_cmd_control** | [**PostElevatorCmdControl**](PostElevatorCmdControl.md)| The details of the elevator_cmd_control |
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

