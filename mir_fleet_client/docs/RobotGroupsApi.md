# mir_fleet_client.RobotGroupsApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**robot_groups_get**](RobotGroupsApi.md#robot_groups_get) | **GET** /robot_groups | GET /robot_groups
[**robot_groups_id_delete**](RobotGroupsApi.md#robot_groups_id_delete) | **DELETE** /robot_groups/{id} | DELETE /robot_groups/{id}
[**robot_groups_id_get**](RobotGroupsApi.md#robot_groups_id_get) | **GET** /robot_groups/{id} | GET /robot_groups/{id}
[**robot_groups_id_put**](RobotGroupsApi.md#robot_groups_id_put) | **PUT** /robot_groups/{id} | PUT /robot_groups/{id}
[**robot_groups_post**](RobotGroupsApi.md#robot_groups_post) | **POST** /robot_groups | POST /robot_groups
[**robot_groups_robot_group_id_mission_groups_get**](RobotGroupsApi.md#robot_groups_robot_group_id_mission_groups_get) | **GET** /robot_groups/{robot_group_id}/mission_groups | GET /robot_groups/{robot_group_id}/mission_groups
[**robot_groups_robot_group_id_mission_groups_mission_group_id_delete**](RobotGroupsApi.md#robot_groups_robot_group_id_mission_groups_mission_group_id_delete) | **DELETE** /robot_groups/{robot_group_id}/mission_groups/{mission_group_id} | DELETE /robot_groups/{robot_group_id}/mission_groups/{mission_group_id}
[**robot_groups_robot_group_id_mission_groups_mission_group_id_get**](RobotGroupsApi.md#robot_groups_robot_group_id_mission_groups_mission_group_id_get) | **GET** /robot_groups/{robot_group_id}/mission_groups/{mission_group_id} | GET /robot_groups/{robot_group_id}/mission_groups/{mission_group_id}
[**robot_groups_robot_group_id_mission_groups_post**](RobotGroupsApi.md#robot_groups_robot_group_id_mission_groups_post) | **POST** /robot_groups/{robot_group_id}/mission_groups | POST /robot_groups/{robot_group_id}/mission_groups


# **robot_groups_get**
> [GetRobotGroups] robot_groups_get()

GET /robot_groups

Retrieve the list of robot groups in the fleet

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import robot_groups_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_robot_groups import GetRobotGroups
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
    api_instance = robot_groups_api.RobotGroupsApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /robot_groups
        api_response = api_instance.robot_groups_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling RobotGroupsApi->robot_groups_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetRobotGroups]**](GetRobotGroups.md)

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

# **robot_groups_id_delete**
> robot_groups_id_delete(id)

DELETE /robot_groups/{id}

Delete the robot group with the specified id

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import robot_groups_api
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
    api_instance = robot_groups_api.RobotGroupsApi(api_client)
    id = 1 # int | The id to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /robot_groups/{id}
        api_instance.robot_groups_id_delete(id)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling RobotGroupsApi->robot_groups_id_delete: %s\n" % e)
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

# **robot_groups_id_get**
> GetRobotGroup robot_groups_id_get(id)

GET /robot_groups/{id}

Retrieve the details about the robot group with the specified id

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import robot_groups_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_robot_group import GetRobotGroup
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
    api_instance = robot_groups_api.RobotGroupsApi(api_client)
    id = 1 # int | The id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /robot_groups/{id}
        api_response = api_instance.robot_groups_id_get(id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling RobotGroupsApi->robot_groups_id_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **id** | **int**| The id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetRobotGroup**](GetRobotGroup.md)

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

# **robot_groups_id_put**
> GetRobotGroup robot_groups_id_put(id, robot_group)

PUT /robot_groups/{id}

Modify the values of the robot group with the specified id

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import robot_groups_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.put_robot_group import PutRobotGroup
from mir_fleet_client.model.get_robot_group import GetRobotGroup
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
    api_instance = robot_groups_api.RobotGroupsApi(api_client)
    id = 1 # int | The id to modify
    robot_group = PutRobotGroup(
        allow_all_mission_groups=True,
        description="description_example",
        name="name_example",
    ) # PutRobotGroup | The new values of the robot_group

    # example passing only required values which don't have defaults set
    try:
        # PUT /robot_groups/{id}
        api_response = api_instance.robot_groups_id_put(id, robot_group)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling RobotGroupsApi->robot_groups_id_put: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **id** | **int**| The id to modify |
 **robot_group** | [**PutRobotGroup**](PutRobotGroup.md)| The new values of the robot_group |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetRobotGroup**](GetRobotGroup.md)

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

# **robot_groups_post**
> GetRobotGroups robot_groups_post(robot_groups)

POST /robot_groups

Add a new robot group to the fleet

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import robot_groups_api
from mir_fleet_client.model.post_robot_groups import PostRobotGroups
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_robot_groups import GetRobotGroups
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
    api_instance = robot_groups_api.RobotGroupsApi(api_client)
    robot_groups = PostRobotGroups(
        allow_all_mission_groups=True,
        created_by_id="created_by_id_example",
        description="description_example",
        name="name_example",
    ) # PostRobotGroups | The details of the robot_groups

    # example passing only required values which don't have defaults set
    try:
        # POST /robot_groups
        api_response = api_instance.robot_groups_post(robot_groups)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling RobotGroupsApi->robot_groups_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **robot_groups** | [**PostRobotGroups**](PostRobotGroups.md)| The details of the robot_groups |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetRobotGroups**](GetRobotGroups.md)

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

# **robot_groups_robot_group_id_mission_groups_get**
> GetRobotGroupMissionGroups robot_groups_robot_group_id_mission_groups_get(robot_group_id)

GET /robot_groups/{robot_group_id}/mission_groups

Get mission groups in robot group

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import robot_groups_api
from mir_fleet_client.model.get_robot_group_mission_groups import GetRobotGroupMissionGroups
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
    api_instance = robot_groups_api.RobotGroupsApi(api_client)
    robot_group_id = 1 # int | The robot_group_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /robot_groups/{robot_group_id}/mission_groups
        api_response = api_instance.robot_groups_robot_group_id_mission_groups_get(robot_group_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling RobotGroupsApi->robot_groups_robot_group_id_mission_groups_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **robot_group_id** | **int**| The robot_group_id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetRobotGroupMissionGroups**](GetRobotGroupMissionGroups.md)

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

# **robot_groups_robot_group_id_mission_groups_mission_group_id_delete**
> robot_groups_robot_group_id_mission_groups_mission_group_id_delete(robot_group_id, mission_group_id)

DELETE /robot_groups/{robot_group_id}/mission_groups/{mission_group_id}

Delete mission group in robot group

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import robot_groups_api
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
    api_instance = robot_groups_api.RobotGroupsApi(api_client)
    robot_group_id = 1 # int | The robot_group_id to delete
    mission_group_id = "mission_group_id_example" # str | The mission_group_id to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /robot_groups/{robot_group_id}/mission_groups/{mission_group_id}
        api_instance.robot_groups_robot_group_id_mission_groups_mission_group_id_delete(robot_group_id, mission_group_id)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling RobotGroupsApi->robot_groups_robot_group_id_mission_groups_mission_group_id_delete: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **robot_group_id** | **int**| The robot_group_id to delete |
 **mission_group_id** | **str**| The mission_group_id to delete |
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

# **robot_groups_robot_group_id_mission_groups_mission_group_id_get**
> GetRobotGroupMissionGroup robot_groups_robot_group_id_mission_groups_mission_group_id_get(robot_group_id, mission_group_id)

GET /robot_groups/{robot_group_id}/mission_groups/{mission_group_id}

Get mission group in robot group

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import robot_groups_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_robot_group_mission_group import GetRobotGroupMissionGroup
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
    api_instance = robot_groups_api.RobotGroupsApi(api_client)
    robot_group_id = 1 # int | The robot_group_id to search for
    mission_group_id = "mission_group_id_example" # str | The mission_group_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /robot_groups/{robot_group_id}/mission_groups/{mission_group_id}
        api_response = api_instance.robot_groups_robot_group_id_mission_groups_mission_group_id_get(robot_group_id, mission_group_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling RobotGroupsApi->robot_groups_robot_group_id_mission_groups_mission_group_id_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **robot_group_id** | **int**| The robot_group_id to search for |
 **mission_group_id** | **str**| The mission_group_id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetRobotGroupMissionGroup**](GetRobotGroupMissionGroup.md)

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

# **robot_groups_robot_group_id_mission_groups_post**
> GetRobotGroupMissionGroups robot_groups_robot_group_id_mission_groups_post(robot_group_id, robot_group_mission_groups)

POST /robot_groups/{robot_group_id}/mission_groups

Add mission groups in robot group

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import robot_groups_api
from mir_fleet_client.model.get_robot_group_mission_groups import GetRobotGroupMissionGroups
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.post_robot_group_mission_groups import PostRobotGroupMissionGroups
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
    api_instance = robot_groups_api.RobotGroupsApi(api_client)
    robot_group_id = 1 # int | The robot_group_id to add the new resource to
    robot_group_mission_groups = PostRobotGroupMissionGroups(
        mission_group_id="mission_group_id_example",
    ) # PostRobotGroupMissionGroups | The details of the robot_group_mission_groups

    # example passing only required values which don't have defaults set
    try:
        # POST /robot_groups/{robot_group_id}/mission_groups
        api_response = api_instance.robot_groups_robot_group_id_mission_groups_post(robot_group_id, robot_group_mission_groups)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling RobotGroupsApi->robot_groups_robot_group_id_mission_groups_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **robot_group_id** | **int**| The robot_group_id to add the new resource to |
 **robot_group_mission_groups** | [**PostRobotGroupMissionGroups**](PostRobotGroupMissionGroups.md)| The details of the robot_group_mission_groups |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetRobotGroupMissionGroups**](GetRobotGroupMissionGroups.md)

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

