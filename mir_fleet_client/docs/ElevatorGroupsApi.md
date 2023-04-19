# mir_fleet_client.ElevatorGroupsApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**elevator_groups_get**](ElevatorGroupsApi.md#elevator_groups_get) | **GET** /elevator_groups | GET /elevator_groups
[**elevator_groups_group_id_elevators_elevator_guid_delete**](ElevatorGroupsApi.md#elevator_groups_group_id_elevators_elevator_guid_delete) | **DELETE** /elevator_groups/{group_id}/elevators/{elevator_guid} | DELETE /elevator_groups/{group_id}/elevators/{elevator_guid}
[**elevator_groups_group_id_elevators_get**](ElevatorGroupsApi.md#elevator_groups_group_id_elevators_get) | **GET** /elevator_groups/{group_id}/elevators | GET /elevator_groups/{group_id}/elevators
[**elevator_groups_group_id_elevators_post**](ElevatorGroupsApi.md#elevator_groups_group_id_elevators_post) | **POST** /elevator_groups/{group_id}/elevators | POST /elevator_groups/{group_id}/elevators
[**elevator_groups_group_id_robots_get**](ElevatorGroupsApi.md#elevator_groups_group_id_robots_get) | **GET** /elevator_groups/{group_id}/robots | GET /elevator_groups/{group_id}/robots
[**elevator_groups_group_id_robots_post**](ElevatorGroupsApi.md#elevator_groups_group_id_robots_post) | **POST** /elevator_groups/{group_id}/robots | POST /elevator_groups/{group_id}/robots
[**elevator_groups_group_id_robots_robot_id_delete**](ElevatorGroupsApi.md#elevator_groups_group_id_robots_robot_id_delete) | **DELETE** /elevator_groups/{group_id}/robots/{robot_id} | DELETE /elevator_groups/{group_id}/robots/{robot_id}
[**elevator_groups_id_delete**](ElevatorGroupsApi.md#elevator_groups_id_delete) | **DELETE** /elevator_groups/{id} | DELETE /elevator_groups/{id}
[**elevator_groups_id_get**](ElevatorGroupsApi.md#elevator_groups_id_get) | **GET** /elevator_groups/{id} | GET /elevator_groups/{id}
[**elevator_groups_id_put**](ElevatorGroupsApi.md#elevator_groups_id_put) | **PUT** /elevator_groups/{id} | PUT /elevator_groups/{id}
[**elevator_groups_post**](ElevatorGroupsApi.md#elevator_groups_post) | **POST** /elevator_groups | POST /elevator_groups


# **elevator_groups_get**
> [GetElevatorGroups] elevator_groups_get()

GET /elevator_groups

Retrieve the list of elevator groups in the fleet

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import elevator_groups_api
from mir_fleet_client.model.get_elevator_groups import GetElevatorGroups
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
    api_instance = elevator_groups_api.ElevatorGroupsApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /elevator_groups
        api_response = api_instance.elevator_groups_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ElevatorGroupsApi->elevator_groups_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetElevatorGroups]**](GetElevatorGroups.md)

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

# **elevator_groups_group_id_elevators_elevator_guid_delete**
> elevator_groups_group_id_elevators_elevator_guid_delete(group_id, elevator_guid)

DELETE /elevator_groups/{group_id}/elevators/{elevator_guid}

Delete the elevator with the specified guid from the elevator group with the specified elevator group ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import elevator_groups_api
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
    api_instance = elevator_groups_api.ElevatorGroupsApi(api_client)
    group_id = 1 # int | The group_id to delete
    elevator_guid = "elevator_guid_example" # str | The elevator_guid to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /elevator_groups/{group_id}/elevators/{elevator_guid}
        api_instance.elevator_groups_group_id_elevators_elevator_guid_delete(group_id, elevator_guid)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ElevatorGroupsApi->elevator_groups_group_id_elevators_elevator_guid_delete: %s\n" % e)
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
from mir_fleet_client.api import elevator_groups_api
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
    api_instance = elevator_groups_api.ElevatorGroupsApi(api_client)
    group_id = 1 # int | The group_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /elevator_groups/{group_id}/elevators
        api_response = api_instance.elevator_groups_group_id_elevators_get(group_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ElevatorGroupsApi->elevator_groups_group_id_elevators_get: %s\n" % e)
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
from mir_fleet_client.api import elevator_groups_api
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
    api_instance = elevator_groups_api.ElevatorGroupsApi(api_client)
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
        print("Exception when calling ElevatorGroupsApi->elevator_groups_group_id_elevators_post: %s\n" % e)
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

# **elevator_groups_group_id_robots_get**
> [GetElevatorGroupRelRobot] elevator_groups_group_id_robots_get(group_id)

GET /elevator_groups/{group_id}/robots

Retrieve the list of robots associated with the elevator group with the specified elevator group ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import elevator_groups_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_elevator_group_rel_robot import GetElevatorGroupRelRobot
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
    api_instance = elevator_groups_api.ElevatorGroupsApi(api_client)
    group_id = 1 # int | The group_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /elevator_groups/{group_id}/robots
        api_response = api_instance.elevator_groups_group_id_robots_get(group_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ElevatorGroupsApi->elevator_groups_group_id_robots_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **group_id** | **int**| The group_id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetElevatorGroupRelRobot]**](GetElevatorGroupRelRobot.md)

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

# **elevator_groups_group_id_robots_post**
> GetElevatorGroupRelRobot elevator_groups_group_id_robots_post(group_id, elevator_group_rel_robot)

POST /elevator_groups/{group_id}/robots

Add new robot to the elevator group with the specified elevator group ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import elevator_groups_api
from mir_fleet_client.model.post_elevator_group_rel_robot import PostElevatorGroupRelRobot
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_elevator_group_rel_robot import GetElevatorGroupRelRobot
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
    api_instance = elevator_groups_api.ElevatorGroupsApi(api_client)
    group_id = 1 # int | The group_id to add the new resource to
    elevator_group_rel_robot = PostElevatorGroupRelRobot(
        robot_id=1,
    ) # PostElevatorGroupRelRobot | The details of the elevator_group_rel_robot

    # example passing only required values which don't have defaults set
    try:
        # POST /elevator_groups/{group_id}/robots
        api_response = api_instance.elevator_groups_group_id_robots_post(group_id, elevator_group_rel_robot)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ElevatorGroupsApi->elevator_groups_group_id_robots_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **group_id** | **int**| The group_id to add the new resource to |
 **elevator_group_rel_robot** | [**PostElevatorGroupRelRobot**](PostElevatorGroupRelRobot.md)| The details of the elevator_group_rel_robot |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetElevatorGroupRelRobot**](GetElevatorGroupRelRobot.md)

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

# **elevator_groups_group_id_robots_robot_id_delete**
> elevator_groups_group_id_robots_robot_id_delete(group_id, robot_id)

DELETE /elevator_groups/{group_id}/robots/{robot_id}

Delete the robot with the specified ID from the elevator group with the specified elevator group ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import elevator_groups_api
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
    api_instance = elevator_groups_api.ElevatorGroupsApi(api_client)
    group_id = 1 # int | The group_id to delete
    robot_id = 1 # int | The robot_id to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /elevator_groups/{group_id}/robots/{robot_id}
        api_instance.elevator_groups_group_id_robots_robot_id_delete(group_id, robot_id)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ElevatorGroupsApi->elevator_groups_group_id_robots_robot_id_delete: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **group_id** | **int**| The group_id to delete |
 **robot_id** | **int**| The robot_id to delete |
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

# **elevator_groups_id_delete**
> elevator_groups_id_delete(id)

DELETE /elevator_groups/{id}

Delete the elevator group with the specified ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import elevator_groups_api
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
    api_instance = elevator_groups_api.ElevatorGroupsApi(api_client)
    id = 1 # int | The id to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /elevator_groups/{id}
        api_instance.elevator_groups_id_delete(id)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ElevatorGroupsApi->elevator_groups_id_delete: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **id** | **int**| The id to delete |
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

# **elevator_groups_id_get**
> GetElevatorGroup elevator_groups_id_get(id)

GET /elevator_groups/{id}

Retrieve the details about the elevator group with the specified ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import elevator_groups_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_elevator_group import GetElevatorGroup
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
    api_instance = elevator_groups_api.ElevatorGroupsApi(api_client)
    id = 1 # int | The id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /elevator_groups/{id}
        api_response = api_instance.elevator_groups_id_get(id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ElevatorGroupsApi->elevator_groups_id_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **id** | **int**| The id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetElevatorGroup**](GetElevatorGroup.md)

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

# **elevator_groups_id_put**
> GetElevatorGroup elevator_groups_id_put(id, elevator_group)

PUT /elevator_groups/{id}

Modify the values of the elevator group with the specified ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import elevator_groups_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_elevator_group import GetElevatorGroup
from mir_fleet_client.model.put_elevator_group import PutElevatorGroup
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
    api_instance = elevator_groups_api.ElevatorGroupsApi(api_client)
    id = 1 # int | The id to modify
    elevator_group = PutElevatorGroup(
        name="name_example",
    ) # PutElevatorGroup | The new values of the elevator_group

    # example passing only required values which don't have defaults set
    try:
        # PUT /elevator_groups/{id}
        api_response = api_instance.elevator_groups_id_put(id, elevator_group)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ElevatorGroupsApi->elevator_groups_id_put: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **id** | **int**| The id to modify |
 **elevator_group** | [**PutElevatorGroup**](PutElevatorGroup.md)| The new values of the elevator_group |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetElevatorGroup**](GetElevatorGroup.md)

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

# **elevator_groups_post**
> GetElevatorGroups elevator_groups_post(elevator_groups)

POST /elevator_groups

Add new elevator group to the fleet

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import elevator_groups_api
from mir_fleet_client.model.get_elevator_groups import GetElevatorGroups
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.post_elevator_groups import PostElevatorGroups
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
    api_instance = elevator_groups_api.ElevatorGroupsApi(api_client)
    elevator_groups = PostElevatorGroups(
        name="name_example",
    ) # PostElevatorGroups | The details of the elevator_groups

    # example passing only required values which don't have defaults set
    try:
        # POST /elevator_groups
        api_response = api_instance.elevator_groups_post(elevator_groups)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ElevatorGroupsApi->elevator_groups_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **elevator_groups** | [**PostElevatorGroups**](PostElevatorGroups.md)| The details of the elevator_groups |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetElevatorGroups**](GetElevatorGroups.md)

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

