# mir_fleet_client.ElevatorFloorsApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**elevator_floors_get**](ElevatorFloorsApi.md#elevator_floors_get) | **GET** /elevator_floors | GET /elevator_floors
[**elevator_floors_guid_delete**](ElevatorFloorsApi.md#elevator_floors_guid_delete) | **DELETE** /elevator_floors/{guid} | DELETE /elevator_floors/{guid}
[**elevator_floors_guid_get**](ElevatorFloorsApi.md#elevator_floors_guid_get) | **GET** /elevator_floors/{guid} | GET /elevator_floors/{guid}
[**elevator_floors_guid_put**](ElevatorFloorsApi.md#elevator_floors_guid_put) | **PUT** /elevator_floors/{guid} | PUT /elevator_floors/{guid}
[**elevator_floors_post**](ElevatorFloorsApi.md#elevator_floors_post) | **POST** /elevator_floors | POST /elevator_floors
[**sessions_session_id_elevator_floors_get**](ElevatorFloorsApi.md#sessions_session_id_elevator_floors_get) | **GET** /sessions/{session_id}/elevator_floors | GET /sessions/{session_id}/elevator_floors


# **elevator_floors_get**
> [GetElevatorFloors] elevator_floors_get()

GET /elevator_floors

Retrieve the list of elevator floors in the fleet

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import elevator_floors_api
from mir_fleet_client.model.get_elevator_floors import GetElevatorFloors
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
    api_instance = elevator_floors_api.ElevatorFloorsApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /elevator_floors
        api_response = api_instance.elevator_floors_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ElevatorFloorsApi->elevator_floors_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetElevatorFloors]**](GetElevatorFloors.md)

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

# **elevator_floors_guid_delete**
> elevator_floors_guid_delete(guid)

DELETE /elevator_floors/{guid}

Delete the specified elevator floor

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import elevator_floors_api
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
    api_instance = elevator_floors_api.ElevatorFloorsApi(api_client)
    guid = "guid_example" # str | The guid to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /elevator_floors/{guid}
        api_instance.elevator_floors_guid_delete(guid)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ElevatorFloorsApi->elevator_floors_guid_delete: %s\n" % e)
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

# **elevator_floors_guid_get**
> GetElevatorFloor elevator_floors_guid_get(guid)

GET /elevator_floors/{guid}

Retrieve the details about the specified elevator floor

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import elevator_floors_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_elevator_floor import GetElevatorFloor
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
    api_instance = elevator_floors_api.ElevatorFloorsApi(api_client)
    guid = "guid_example" # str | The guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /elevator_floors/{guid}
        api_response = api_instance.elevator_floors_guid_get(guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ElevatorFloorsApi->elevator_floors_guid_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetElevatorFloor**](GetElevatorFloor.md)

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

# **elevator_floors_guid_put**
> GetElevatorFloor elevator_floors_guid_put(guid, elevator_floor)

PUT /elevator_floors/{guid}

Modify the values of the specified elevator floor

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import elevator_floors_api
from mir_fleet_client.model.put_elevator_floor import PutElevatorFloor
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_elevator_floor import GetElevatorFloor
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
    api_instance = elevator_floors_api.ElevatorFloorsApi(api_client)
    guid = "guid_example" # str | The guid to modify
    elevator_floor = PutElevatorFloor(
        door=1,
        elevator_entry_pos_guid="elevator_entry_pos_guid_example",
        elevator_guid="elevator_guid_example",
        elevator_pos_guid="elevator_pos_guid_example",
        entry_mission_guid="entry_mission_guid_example",
        exit_mission_guid="exit_mission_guid_example",
        floor=1,
        map_guid="map_guid_example",
    ) # PutElevatorFloor | The new values of the elevator_floor

    # example passing only required values which don't have defaults set
    try:
        # PUT /elevator_floors/{guid}
        api_response = api_instance.elevator_floors_guid_put(guid, elevator_floor)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ElevatorFloorsApi->elevator_floors_guid_put: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to modify |
 **elevator_floor** | [**PutElevatorFloor**](PutElevatorFloor.md)| The new values of the elevator_floor |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetElevatorFloor**](GetElevatorFloor.md)

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

# **elevator_floors_post**
> GetElevatorFloors elevator_floors_post(elevator_floors)

POST /elevator_floors

Add a new elevator floor to the fleet

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import elevator_floors_api
from mir_fleet_client.model.get_elevator_floors import GetElevatorFloors
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.post_elevator_floors import PostElevatorFloors
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
    api_instance = elevator_floors_api.ElevatorFloorsApi(api_client)
    elevator_floors = PostElevatorFloors(
        created_by_id="created_by_id_example",
        door=1,
        elevator_entry_pos_guid="elevator_entry_pos_guid_example",
        elevator_guid="elevator_guid_example",
        elevator_pos_guid="elevator_pos_guid_example",
        entry_mission_guid="entry_mission_guid_example",
        exit_mission_guid="exit_mission_guid_example",
        floor=1,
        guid="guid_example",
        map_guid="map_guid_example",
    ) # PostElevatorFloors | The details of the elevator_floors

    # example passing only required values which don't have defaults set
    try:
        # POST /elevator_floors
        api_response = api_instance.elevator_floors_post(elevator_floors)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ElevatorFloorsApi->elevator_floors_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **elevator_floors** | [**PostElevatorFloors**](PostElevatorFloors.md)| The details of the elevator_floors |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetElevatorFloors**](GetElevatorFloors.md)

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

# **sessions_session_id_elevator_floors_get**
> [GetSessionElevatorFloors] sessions_session_id_elevator_floors_get(session_id)

GET /sessions/{session_id}/elevator_floors

Retrieve the list of elevator floors that belong to the session with the specified session ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import elevator_floors_api
from mir_fleet_client.model.get_session_elevator_floors import GetSessionElevatorFloors
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
    api_instance = elevator_floors_api.ElevatorFloorsApi(api_client)
    session_id = "session_id_example" # str | The session_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /sessions/{session_id}/elevator_floors
        api_response = api_instance.sessions_session_id_elevator_floors_get(session_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ElevatorFloorsApi->sessions_session_id_elevator_floors_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **session_id** | **str**| The session_id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetSessionElevatorFloors]**](GetSessionElevatorFloors.md)

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

