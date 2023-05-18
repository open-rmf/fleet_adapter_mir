# mir_fleet_client.ElevatorsApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**elevator_groups_group_id_elevators_elevator_guid_delete**](ElevatorsApi.md#elevator_groups_group_id_elevators_elevator_guid_delete) | **DELETE** /elevator_groups/{group_id}/elevators/{elevator_guid} | DELETE /elevator_groups/{group_id}/elevators/{elevator_guid}
[**elevator_groups_group_id_elevators_get**](ElevatorsApi.md#elevator_groups_group_id_elevators_get) | **GET** /elevator_groups/{group_id}/elevators | GET /elevator_groups/{group_id}/elevators
[**elevator_groups_group_id_elevators_post**](ElevatorsApi.md#elevator_groups_group_id_elevators_post) | **POST** /elevator_groups/{group_id}/elevators | POST /elevator_groups/{group_id}/elevators
[**elevators_get**](ElevatorsApi.md#elevators_get) | **GET** /elevators | GET /elevators
[**elevators_guid_cmd_check_server_get**](ElevatorsApi.md#elevators_guid_cmd_check_server_get) | **GET** /elevators/{guid}/cmd_check_server | GET /elevators/{guid}/cmd_check_server
[**elevators_guid_cmd_control_post**](ElevatorsApi.md#elevators_guid_cmd_control_post) | **POST** /elevators/{guid}/cmd_control | POST /elevators/{guid}/cmd_control
[**elevators_guid_cmd_door_post**](ElevatorsApi.md#elevators_guid_cmd_door_post) | **POST** /elevators/{guid}/cmd_door | POST /elevators/{guid}/cmd_door
[**elevators_guid_cmd_floor_post**](ElevatorsApi.md#elevators_guid_cmd_floor_post) | **POST** /elevators/{guid}/cmd_floor | POST /elevators/{guid}/cmd_floor
[**elevators_guid_delete**](ElevatorsApi.md#elevators_guid_delete) | **DELETE** /elevators/{guid} | DELETE /elevators/{guid}
[**elevators_guid_get**](ElevatorsApi.md#elevators_guid_get) | **GET** /elevators/{guid} | GET /elevators/{guid}
[**elevators_guid_opcua_scanner_get**](ElevatorsApi.md#elevators_guid_opcua_scanner_get) | **GET** /elevators/{guid}/opcua_scanner | GET /elevators/{guid}/opcua_scanner
[**elevators_guid_put**](ElevatorsApi.md#elevators_guid_put) | **PUT** /elevators/{guid} | PUT /elevators/{guid}
[**elevators_post**](ElevatorsApi.md#elevators_post) | **POST** /elevators | POST /elevators
[**sessions_session_id_elevators_get**](ElevatorsApi.md#sessions_session_id_elevators_get) | **GET** /sessions/{session_id}/elevators | GET /sessions/{session_id}/elevators


# **elevator_groups_group_id_elevators_elevator_guid_delete**
> elevator_groups_group_id_elevators_elevator_guid_delete(group_id, elevator_guid)

DELETE /elevator_groups/{group_id}/elevators/{elevator_guid}

Delete the elevator with the specified guid from the elevator group with the specified elevator group ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import elevators_api
from mir_fleet_client.model.error import Error
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
    api_instance = elevators_api.ElevatorsApi(api_client)
    group_id = 1 # int | The group_id to delete
    elevator_guid = "elevator_guid_example" # str | The elevator_guid to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /elevator_groups/{group_id}/elevators/{elevator_guid}
        api_instance.elevator_groups_group_id_elevators_elevator_guid_delete(group_id, elevator_guid)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ElevatorsApi->elevator_groups_group_id_elevators_elevator_guid_delete: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **group_id** | **int**| The group_id to delete |
 **elevator_guid** | **str**| The elevator_guid to delete |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

void (empty response body)

### Authorization

[Basic](../README.md#Basic)

### HTTP request headers

 - **Content-Type**: Not defined
 - **Accept**: application/json


### HTTP response details

| Status code | Description | Response headers |
|-------------|-------------|------------------|
**204** | The element has been deleted successfully |  -  |
**400** | Invalid filters or Invalid JSON or Argument error or Missing content type application/json on the header or Bad request or No fields |  -  |
**404** | Not found |  -  |

[[Back to top]](#) [[Back to API list]](../README.md#documentation-for-api-endpoints) [[Back to Model list]](../README.md#documentation-for-models) [[Back to README]](../README.md)

# **elevator_groups_group_id_elevators_get**
> [GetElevatorGroupRelElevator] elevator_groups_group_id_elevators_get(group_id)

GET /elevator_groups/{group_id}/elevators

Retrieve the list of elevators associated with the elevator group with the specified elevator group ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import elevators_api
from mir_fleet_client.model.get_elevator_group_rel_elevator import GetElevatorGroupRelElevator
from mir_fleet_client.model.error import Error
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
    api_instance = elevators_api.ElevatorsApi(api_client)
    group_id = 1 # int | The group_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /elevator_groups/{group_id}/elevators
        api_response = api_instance.elevator_groups_group_id_elevators_get(group_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ElevatorsApi->elevator_groups_group_id_elevators_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **group_id** | **int**| The group_id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetElevatorGroupRelElevator]**](GetElevatorGroupRelElevator.md)

### Authorization

[Basic](../README.md#Basic)

### HTTP request headers

 - **Content-Type**: Not defined
 - **Accept**: application/json


### HTTP response details

| Status code | Description | Response headers |
|-------------|-------------|------------------|
**200** | Successfully retrieved the list of elements |  -  |
**400** | Invalid ordering or Invalid filters or Wrong output fields or Invalid limits |  -  |
**404** | Not found |  -  |

[[Back to top]](#) [[Back to API list]](../README.md#documentation-for-api-endpoints) [[Back to Model list]](../README.md#documentation-for-models) [[Back to README]](../README.md)

# **elevator_groups_group_id_elevators_post**
> GetElevatorGroupRelElevator elevator_groups_group_id_elevators_post(group_id, elevator_group_rel_elevator)

POST /elevator_groups/{group_id}/elevators

Add new elevator to the elevator group with the specified ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import elevators_api
from mir_fleet_client.model.get_elevator_group_rel_elevator import GetElevatorGroupRelElevator
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.post_elevator_group_rel_elevator import PostElevatorGroupRelElevator
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
    api_instance = elevators_api.ElevatorsApi(api_client)
    group_id = 1 # int | The group_id to add the new resource to
    elevator_group_rel_elevator = PostElevatorGroupRelElevator(
        elevator_guid="elevator_guid_example",
        group_id=1,
    ) # PostElevatorGroupRelElevator | The details of the elevator_group_rel_elevator

    # example passing only required values which don't have defaults set
    try:
        # POST /elevator_groups/{group_id}/elevators
        api_response = api_instance.elevator_groups_group_id_elevators_post(group_id, elevator_group_rel_elevator)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ElevatorsApi->elevator_groups_group_id_elevators_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **group_id** | **int**| The group_id to add the new resource to |
 **elevator_group_rel_elevator** | [**PostElevatorGroupRelElevator**](PostElevatorGroupRelElevator.md)| The details of the elevator_group_rel_elevator |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetElevatorGroupRelElevator**](GetElevatorGroupRelElevator.md)

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

# **elevators_get**
> [GetElevators] elevators_get()

GET /elevators

Retrieve a list of elevators in the fleet

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import elevators_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_elevators import GetElevators
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
    api_instance = elevators_api.ElevatorsApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /elevators
        api_response = api_instance.elevators_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ElevatorsApi->elevators_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetElevators]**](GetElevators.md)

### Authorization

[Basic](../README.md#Basic)

### HTTP request headers

 - **Content-Type**: Not defined
 - **Accept**: application/json


### HTTP response details

| Status code | Description | Response headers |
|-------------|-------------|------------------|
**200** | Successfully retrieved the list of elements |  -  |
**400** | Invalid ordering or Invalid filters or Wrong output fields or Invalid limits |  -  |
**404** | Not found |  -  |

[[Back to top]](#) [[Back to API list]](../README.md#documentation-for-api-endpoints) [[Back to Model list]](../README.md#documentation-for-models) [[Back to README]](../README.md)

# **elevators_guid_cmd_check_server_get**
> bool, date, datetime, dict, float, int, list, str, none_type elevators_guid_cmd_check_server_get(guid)

GET /elevators/{guid}/cmd_check_server

Check if the server provides expected methods and variables.

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import elevators_api
from mir_fleet_client.model.error import Error
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
    api_instance = elevators_api.ElevatorsApi(api_client)
    guid = "guid_example" # str | The guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /elevators/{guid}/cmd_check_server
        api_response = api_instance.elevators_guid_cmd_check_server_get(guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ElevatorsApi->elevators_guid_cmd_check_server_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

**bool, date, datetime, dict, float, int, list, str, none_type**

### Authorization

[Basic](../README.md#Basic)

### HTTP request headers

 - **Content-Type**: Not defined
 - **Accept**: application/json


### HTTP response details

| Status code | Description | Response headers |
|-------------|-------------|------------------|
**200** | Successfully retrieve the specified element |  -  |
**400** | Invalid ordering or Invalid filters or Wrong output fields or Invalid limits |  -  |
**404** | Not found |  -  |

[[Back to top]](#) [[Back to API list]](../README.md#documentation-for-api-endpoints) [[Back to Model list]](../README.md#documentation-for-models) [[Back to README]](../README.md)

# **elevators_guid_cmd_control_post**
> bool, date, datetime, dict, float, int, list, str, none_type elevators_guid_cmd_control_post(guid, elevator_cmd_control)

POST /elevators/{guid}/cmd_control

Request a control of the elevator.

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import elevators_api
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
    api_instance = elevators_api.ElevatorsApi(api_client)
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
        print("Exception when calling ElevatorsApi->elevators_guid_cmd_control_post: %s\n" % e)
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

# **elevators_guid_cmd_door_post**
> bool, date, datetime, dict, float, int, list, str, none_type elevators_guid_cmd_door_post(guid, elevator_cmd_door)

POST /elevators/{guid}/cmd_door

Request the elevator to open a door.

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import elevators_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.post_elevator_cmd_door import PostElevatorCmdDoor
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
    api_instance = elevators_api.ElevatorsApi(api_client)
    guid = "guid_example" # str | The guid to add the new resource to
    elevator_cmd_door = PostElevatorCmdDoor(
        door_front=True,
        door_rear=True,
        open=True,
    ) # PostElevatorCmdDoor | The details of the elevator_cmd_door

    # example passing only required values which don't have defaults set
    try:
        # POST /elevators/{guid}/cmd_door
        api_response = api_instance.elevators_guid_cmd_door_post(guid, elevator_cmd_door)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ElevatorsApi->elevators_guid_cmd_door_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to add the new resource to |
 **elevator_cmd_door** | [**PostElevatorCmdDoor**](PostElevatorCmdDoor.md)| The details of the elevator_cmd_door |
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

# **elevators_guid_cmd_floor_post**
> bool, date, datetime, dict, float, int, list, str, none_type elevators_guid_cmd_floor_post(guid, elevator_cmd_floor)

POST /elevators/{guid}/cmd_floor

Request the elevator to go the a floor and open the door.

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import elevators_api
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
    api_instance = elevators_api.ElevatorsApi(api_client)
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
        print("Exception when calling ElevatorsApi->elevators_guid_cmd_floor_post: %s\n" % e)
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

# **elevators_guid_delete**
> elevators_guid_delete(guid)

DELETE /elevators/{guid}

Delete the elevator with the specified guid

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import elevators_api
from mir_fleet_client.model.error import Error
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
    api_instance = elevators_api.ElevatorsApi(api_client)
    guid = "guid_example" # str | The guid to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /elevators/{guid}
        api_instance.elevators_guid_delete(guid)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ElevatorsApi->elevators_guid_delete: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to delete |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

void (empty response body)

### Authorization

[Basic](../README.md#Basic)

### HTTP request headers

 - **Content-Type**: Not defined
 - **Accept**: application/json


### HTTP response details

| Status code | Description | Response headers |
|-------------|-------------|------------------|
**204** | The element has been deleted successfully |  -  |
**400** | Invalid filters or Invalid JSON or Argument error or Missing content type application/json on the header or Bad request or No fields |  -  |
**404** | Not found |  -  |

[[Back to top]](#) [[Back to API list]](../README.md#documentation-for-api-endpoints) [[Back to Model list]](../README.md#documentation-for-models) [[Back to README]](../README.md)

# **elevators_guid_get**
> GetElevator elevators_guid_get(guid)

GET /elevators/{guid}

Retrieve the details about the elevator with the specified guid

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import elevators_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_elevator import GetElevator
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
    api_instance = elevators_api.ElevatorsApi(api_client)
    guid = "guid_example" # str | The guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /elevators/{guid}
        api_response = api_instance.elevators_guid_get(guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ElevatorsApi->elevators_guid_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetElevator**](GetElevator.md)

### Authorization

[Basic](../README.md#Basic)

### HTTP request headers

 - **Content-Type**: Not defined
 - **Accept**: application/json


### HTTP response details

| Status code | Description | Response headers |
|-------------|-------------|------------------|
**200** | Successfully retrieve the specified element |  -  |
**400** | Invalid ordering or Invalid filters or Wrong output fields or Invalid limits |  -  |
**404** | Not found |  -  |

[[Back to top]](#) [[Back to API list]](../README.md#documentation-for-api-endpoints) [[Back to Model list]](../README.md#documentation-for-models) [[Back to README]](../README.md)

# **elevators_guid_opcua_scanner_get**
> bool, date, datetime, dict, float, int, list, str, none_type elevators_guid_opcua_scanner_get(guid)

GET /elevators/{guid}/opcua_scanner

Retrieve the list of available namespaces on specified Opc UA server

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import elevators_api
from mir_fleet_client.model.error import Error
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
    api_instance = elevators_api.ElevatorsApi(api_client)
    guid = "guid_example" # str | The guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /elevators/{guid}/opcua_scanner
        api_response = api_instance.elevators_guid_opcua_scanner_get(guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ElevatorsApi->elevators_guid_opcua_scanner_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

**bool, date, datetime, dict, float, int, list, str, none_type**

### Authorization

[Basic](../README.md#Basic)

### HTTP request headers

 - **Content-Type**: Not defined
 - **Accept**: application/json


### HTTP response details

| Status code | Description | Response headers |
|-------------|-------------|------------------|
**200** | Successfully retrieve the specified element |  -  |
**400** | Invalid ordering or Invalid filters or Wrong output fields or Invalid limits |  -  |
**404** | Not found |  -  |

[[Back to top]](#) [[Back to API list]](../README.md#documentation-for-api-endpoints) [[Back to Model list]](../README.md#documentation-for-models) [[Back to README]](../README.md)

# **elevators_guid_put**
> GetElevator elevators_guid_put(guid, elevator)

PUT /elevators/{guid}

Modify the values of the elevator with the specified guid

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import elevators_api
from mir_fleet_client.model.put_elevator import PutElevator
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_elevator import GetElevator
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
    api_instance = elevators_api.ElevatorsApi(api_client)
    guid = "guid_example" # str | The guid to modify
    elevator = PutElevator(
        active=True,
        authentication="authentication_example",
        driver="driver_example",
        ip="ip_example",
        name="name_example",
        namespace="namespace_example",
        one_way=1,
        password="password_example",
        port=1,
        session_guid="session_guid_example",
        turn_in_place=True,
        username="username_example",
    ) # PutElevator | The new values of the elevator

    # example passing only required values which don't have defaults set
    try:
        # PUT /elevators/{guid}
        api_response = api_instance.elevators_guid_put(guid, elevator)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ElevatorsApi->elevators_guid_put: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to modify |
 **elevator** | [**PutElevator**](PutElevator.md)| The new values of the elevator |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetElevator**](GetElevator.md)

### Authorization

[Basic](../README.md#Basic)

### HTTP request headers

 - **Content-Type**: application/json
 - **Accept**: application/json


### HTTP response details

| Status code | Description | Response headers |
|-------------|-------------|------------------|
**200** | The element has been modified successfully |  -  |
**400** | Invalid filters or Invalid JSON or Argument error or Missing content type application/json on the header or Bad request or No fields |  -  |
**404** | Not found |  -  |

[[Back to top]](#) [[Back to API list]](../README.md#documentation-for-api-endpoints) [[Back to Model list]](../README.md#documentation-for-models) [[Back to README]](../README.md)

# **elevators_post**
> GetElevators elevators_post(elevators)

POST /elevators

Add a new elevator to the fleet

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import elevators_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_elevators import GetElevators
from mir_fleet_client.model.post_elevators import PostElevators
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
    api_instance = elevators_api.ElevatorsApi(api_client)
    elevators = PostElevators(
        active=True,
        authentication="authentication_example",
        created_by_id="created_by_id_example",
        driver="driver_example",
        guid="guid_example",
        ip="ip_example",
        name="name_example",
        namespace="namespace_example",
        one_way=1,
        password="password_example",
        port=1,
        security_policy="security_policy_example",
        session_guid="session_guid_example",
        turn_in_place=True,
        username="username_example",
    ) # PostElevators | The details of the elevators

    # example passing only required values which don't have defaults set
    try:
        # POST /elevators
        api_response = api_instance.elevators_post(elevators)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ElevatorsApi->elevators_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **elevators** | [**PostElevators**](PostElevators.md)| The details of the elevators |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetElevators**](GetElevators.md)

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

# **sessions_session_id_elevators_get**
> [GetSessionElevators] sessions_session_id_elevators_get(session_id)

GET /sessions/{session_id}/elevators

Retrieve the list of elevators that belong to the session with the specified session ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import elevators_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_session_elevators import GetSessionElevators
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
    api_instance = elevators_api.ElevatorsApi(api_client)
    session_id = "session_id_example" # str | The session_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /sessions/{session_id}/elevators
        api_response = api_instance.sessions_session_id_elevators_get(session_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ElevatorsApi->sessions_session_id_elevators_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **session_id** | **str**| The session_id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetSessionElevators]**](GetSessionElevators.md)

### Authorization

[Basic](../README.md#Basic)

### HTTP request headers

 - **Content-Type**: Not defined
 - **Accept**: application/json


### HTTP response details

| Status code | Description | Response headers |
|-------------|-------------|------------------|
**200** | Successfully retrieved the list of elements |  -  |
**400** | Invalid ordering or Invalid filters or Wrong output fields or Invalid limits |  -  |
**404** | Not found |  -  |

[[Back to top]](#) [[Back to API list]](../README.md#documentation-for-api-endpoints) [[Back to Model list]](../README.md#documentation-for-models) [[Back to README]](../README.md)

