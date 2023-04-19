# mir_fleet_client.RobotsApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**charging_groups_group_id_robots_get**](RobotsApi.md#charging_groups_group_id_robots_get) | **GET** /charging_groups/{group_id}/robots | GET /charging_groups/{group_id}/robots
[**charging_groups_group_id_robots_post**](RobotsApi.md#charging_groups_group_id_robots_post) | **POST** /charging_groups/{group_id}/robots | POST /charging_groups/{group_id}/robots
[**charging_groups_group_id_robots_robot_id_delete**](RobotsApi.md#charging_groups_group_id_robots_robot_id_delete) | **DELETE** /charging_groups/{group_id}/robots/{robot_id} | DELETE /charging_groups/{group_id}/robots/{robot_id}
[**charging_groups_group_id_robots_robot_id_get**](RobotsApi.md#charging_groups_group_id_robots_robot_id_get) | **GET** /charging_groups/{group_id}/robots/{robot_id} | GET /charging_groups/{group_id}/robots/{robot_id}
[**elevator_groups_group_id_robots_get**](RobotsApi.md#elevator_groups_group_id_robots_get) | **GET** /elevator_groups/{group_id}/robots | GET /elevator_groups/{group_id}/robots
[**elevator_groups_group_id_robots_post**](RobotsApi.md#elevator_groups_group_id_robots_post) | **POST** /elevator_groups/{group_id}/robots | POST /elevator_groups/{group_id}/robots
[**elevator_groups_group_id_robots_robot_id_delete**](RobotsApi.md#elevator_groups_group_id_robots_robot_id_delete) | **DELETE** /elevator_groups/{group_id}/robots/{robot_id} | DELETE /elevator_groups/{group_id}/robots/{robot_id}
[**robots_get**](RobotsApi.md#robots_get) | **GET** /robots | GET /robots
[**robots_id_delete**](RobotsApi.md#robots_id_delete) | **DELETE** /robots/{id} | DELETE /robots/{id}
[**robots_id_get**](RobotsApi.md#robots_id_get) | **GET** /robots/{id} | GET /robots/{id}
[**robots_id_put**](RobotsApi.md#robots_id_put) | **PUT** /robots/{id} | PUT /robots/{id}
[**robots_post**](RobotsApi.md#robots_post) | **POST** /robots | POST /robots
[**robots_robot_id_charging_groups_get**](RobotsApi.md#robots_robot_id_charging_groups_get) | **GET** /robots/{robot_id}/charging_groups | GET /robots/{robot_id}/charging_groups
[**robots_scan_get**](RobotsApi.md#robots_scan_get) | **GET** /robots/scan | GET /robots/scan
[**robots_scan_ip_get**](RobotsApi.md#robots_scan_ip_get) | **GET** /robots/scan/{ip} | GET /robots/scan/{ip}


# **charging_groups_group_id_robots_get**
> [GetRobotChargingGroups] charging_groups_group_id_robots_get(group_id)

GET /charging_groups/{group_id}/robots

Retrieve the list of robots associated with the charging group with the specified charging group ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import robots_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_robot_charging_groups import GetRobotChargingGroups
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
    api_instance = robots_api.RobotsApi(api_client)
    group_id = 1 # int | The group_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /charging_groups/{group_id}/robots
        api_response = api_instance.charging_groups_group_id_robots_get(group_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling RobotsApi->charging_groups_group_id_robots_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **group_id** | **int**| The group_id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetRobotChargingGroups]**](GetRobotChargingGroups.md)

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

# **charging_groups_group_id_robots_post**
> GetRobotChargingGroups charging_groups_group_id_robots_post(group_id, robot_charging_groups)

POST /charging_groups/{group_id}/robots

Add new robot to the charging group with the specified charging group ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import robots_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.post_robot_charging_groups import PostRobotChargingGroups
from mir_fleet_client.model.get_robot_charging_groups import GetRobotChargingGroups
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
    api_instance = robots_api.RobotsApi(api_client)
    group_id = 1 # int | The group_id to add the new resource to
    robot_charging_groups = PostRobotChargingGroups(
        group_id=1,
        robot_id=1,
    ) # PostRobotChargingGroups | The details of the robot_charging_groups

    # example passing only required values which don't have defaults set
    try:
        # POST /charging_groups/{group_id}/robots
        api_response = api_instance.charging_groups_group_id_robots_post(group_id, robot_charging_groups)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling RobotsApi->charging_groups_group_id_robots_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **group_id** | **int**| The group_id to add the new resource to |
 **robot_charging_groups** | [**PostRobotChargingGroups**](PostRobotChargingGroups.md)| The details of the robot_charging_groups |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetRobotChargingGroups**](GetRobotChargingGroups.md)

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

# **charging_groups_group_id_robots_robot_id_delete**
> charging_groups_group_id_robots_robot_id_delete(group_id, robot_id)

DELETE /charging_groups/{group_id}/robots/{robot_id}

Delete the robot with the specified ID from the charging group with the specified charging group ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import robots_api
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
    api_instance = robots_api.RobotsApi(api_client)
    group_id = 1 # int | The group_id to delete
    robot_id = 1 # int | The robot_id to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /charging_groups/{group_id}/robots/{robot_id}
        api_instance.charging_groups_group_id_robots_robot_id_delete(group_id, robot_id)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling RobotsApi->charging_groups_group_id_robots_robot_id_delete: %s\n" % e)
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

# **charging_groups_group_id_robots_robot_id_get**
> GetRobotChargingGroup charging_groups_group_id_robots_robot_id_get(group_id, robot_id)

GET /charging_groups/{group_id}/robots/{robot_id}

Retrieve the details about the robot with the specified ID associated with the charging group with the specified charging group ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import robots_api
from mir_fleet_client.model.get_robot_charging_group import GetRobotChargingGroup
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
    api_instance = robots_api.RobotsApi(api_client)
    group_id = 1 # int | The group_id to search for
    robot_id = 1 # int | The robot_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /charging_groups/{group_id}/robots/{robot_id}
        api_response = api_instance.charging_groups_group_id_robots_robot_id_get(group_id, robot_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling RobotsApi->charging_groups_group_id_robots_robot_id_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **group_id** | **int**| The group_id to search for |
 **robot_id** | **int**| The robot_id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetRobotChargingGroup**](GetRobotChargingGroup.md)

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

# **elevator_groups_group_id_robots_get**
> [GetElevatorGroupRelRobot] elevator_groups_group_id_robots_get(group_id)

GET /elevator_groups/{group_id}/robots

Retrieve the list of robots associated with the elevator group with the specified elevator group ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import robots_api
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
    api_instance = robots_api.RobotsApi(api_client)
    group_id = 1 # int | The group_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /elevator_groups/{group_id}/robots
        api_response = api_instance.elevator_groups_group_id_robots_get(group_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling RobotsApi->elevator_groups_group_id_robots_get: %s\n" % e)
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
from mir_fleet_client.api import robots_api
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
    api_instance = robots_api.RobotsApi(api_client)
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
        print("Exception when calling RobotsApi->elevator_groups_group_id_robots_post: %s\n" % e)
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
from mir_fleet_client.api import robots_api
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
    api_instance = robots_api.RobotsApi(api_client)
    group_id = 1 # int | The group_id to delete
    robot_id = 1 # int | The robot_id to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /elevator_groups/{group_id}/robots/{robot_id}
        api_instance.elevator_groups_group_id_robots_robot_id_delete(group_id, robot_id)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling RobotsApi->elevator_groups_group_id_robots_robot_id_delete: %s\n" % e)
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

# **robots_get**
> [GetRobots] robots_get()

GET /robots

Retrieve the list of robots

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import robots_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_robots import GetRobots
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
    api_instance = robots_api.RobotsApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /robots
        api_response = api_instance.robots_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling RobotsApi->robots_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetRobots]**](GetRobots.md)

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

# **robots_id_delete**
> robots_id_delete(id)

DELETE /robots/{id}

Delete the robot with the specified id

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import robots_api
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
    api_instance = robots_api.RobotsApi(api_client)
    id = 1 # int | The id to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /robots/{id}
        api_instance.robots_id_delete(id)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling RobotsApi->robots_id_delete: %s\n" % e)
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

# **robots_id_get**
> GetRobot robots_id_get(id)

GET /robots/{id}

Retrieve the details about the robot with the specified id

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import robots_api
from mir_fleet_client.model.get_robot import GetRobot
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
    api_instance = robots_api.RobotsApi(api_client)
    id = 1 # int | The id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /robots/{id}
        api_response = api_instance.robots_id_get(id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling RobotsApi->robots_id_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **id** | **int**| The id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetRobot**](GetRobot.md)

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

# **robots_id_put**
> GetRobot robots_id_put(id, robot)

PUT /robots/{id}

Modify the values of the robot with the specified id

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import robots_api
from mir_fleet_client.model.get_robot import GetRobot
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.put_robot import PutRobot
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
    api_instance = robots_api.RobotsApi(api_client)
    id = 1 # int | The id to modify
    robot = PutRobot(
        active=True,
        description="description_example",
        robot_group_id=1,
    ) # PutRobot | The new values of the robot

    # example passing only required values which don't have defaults set
    try:
        # PUT /robots/{id}
        api_response = api_instance.robots_id_put(id, robot)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling RobotsApi->robots_id_put: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **id** | **int**| The id to modify |
 **robot** | [**PutRobot**](PutRobot.md)| The new values of the robot |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetRobot**](GetRobot.md)

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

# **robots_post**
> GetRobots robots_post(robots)

POST /robots

Add a new robot

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import robots_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.post_robots import PostRobots
from mir_fleet_client.model.get_robots import GetRobots
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
    api_instance = robots_api.RobotsApi(api_client)
    robots = PostRobots(
        active=True,
        created_by_id="created_by_id_example",
        description="description_example",
        factory_reset_needed=True,
        ip="ip_example",
        robot_group_id=1,
    ) # PostRobots | The details of the robots

    # example passing only required values which don't have defaults set
    try:
        # POST /robots
        api_response = api_instance.robots_post(robots)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling RobotsApi->robots_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **robots** | [**PostRobots**](PostRobots.md)| The details of the robots |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetRobots**](GetRobots.md)

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

# **robots_robot_id_charging_groups_get**
> [GetRobotsChargingGroups] robots_robot_id_charging_groups_get(robot_id)

GET /robots/{robot_id}/charging_groups

Retrieve list of charging groups associated with the robot with the specified ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import robots_api
from mir_fleet_client.model.get_robots_charging_groups import GetRobotsChargingGroups
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
    api_instance = robots_api.RobotsApi(api_client)
    robot_id = 1 # int | The robot_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /robots/{robot_id}/charging_groups
        api_response = api_instance.robots_robot_id_charging_groups_get(robot_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling RobotsApi->robots_robot_id_charging_groups_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **robot_id** | **int**| The robot_id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetRobotsChargingGroups]**](GetRobotsChargingGroups.md)

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

# **robots_scan_get**
> [GetRobotsScan] robots_scan_get()

GET /robots/scan

Scan for robots

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import robots_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_robots_scan import GetRobotsScan
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
    api_instance = robots_api.RobotsApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /robots/scan
        api_response = api_instance.robots_scan_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling RobotsApi->robots_scan_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetRobotsScan]**](GetRobotsScan.md)

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

# **robots_scan_ip_get**
> GetRobotScan robots_scan_ip_get(ip)

GET /robots/scan/{ip}

Scan for robots on IP

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import robots_api
from mir_fleet_client.model.get_robot_scan import GetRobotScan
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
    api_instance = robots_api.RobotsApi(api_client)
    ip = "ip_example" # str | The ip to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /robots/scan/{ip}
        api_response = api_instance.robots_scan_ip_get(ip)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling RobotsApi->robots_scan_ip_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **ip** | **str**| The ip to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetRobotScan**](GetRobotScan.md)

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

