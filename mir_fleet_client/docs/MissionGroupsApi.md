# mir_fleet_client.MissionGroupsApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**mission_groups_get**](MissionGroupsApi.md#mission_groups_get) | **GET** /mission_groups | GET /mission_groups
[**mission_groups_group_id_missions_get**](MissionGroupsApi.md#mission_groups_group_id_missions_get) | **GET** /mission_groups/{group_id}/missions | GET /mission_groups/{group_id}/missions
[**mission_groups_guid_delete**](MissionGroupsApi.md#mission_groups_guid_delete) | **DELETE** /mission_groups/{guid} | DELETE /mission_groups/{guid}
[**mission_groups_guid_get**](MissionGroupsApi.md#mission_groups_guid_get) | **GET** /mission_groups/{guid} | GET /mission_groups/{guid}
[**mission_groups_guid_put**](MissionGroupsApi.md#mission_groups_guid_put) | **PUT** /mission_groups/{guid} | PUT /mission_groups/{guid}
[**mission_groups_mission_group_id_actions_get**](MissionGroupsApi.md#mission_groups_mission_group_id_actions_get) | **GET** /mission_groups/{mission_group_id}/actions | GET /mission_groups/{mission_group_id}/actions
[**mission_groups_post**](MissionGroupsApi.md#mission_groups_post) | **POST** /mission_groups | POST /mission_groups
[**robot_groups_robot_group_id_mission_groups_get**](MissionGroupsApi.md#robot_groups_robot_group_id_mission_groups_get) | **GET** /robot_groups/{robot_group_id}/mission_groups | GET /robot_groups/{robot_group_id}/mission_groups
[**robot_groups_robot_group_id_mission_groups_mission_group_id_delete**](MissionGroupsApi.md#robot_groups_robot_group_id_mission_groups_mission_group_id_delete) | **DELETE** /robot_groups/{robot_group_id}/mission_groups/{mission_group_id} | DELETE /robot_groups/{robot_group_id}/mission_groups/{mission_group_id}
[**robot_groups_robot_group_id_mission_groups_mission_group_id_get**](MissionGroupsApi.md#robot_groups_robot_group_id_mission_groups_mission_group_id_get) | **GET** /robot_groups/{robot_group_id}/mission_groups/{mission_group_id} | GET /robot_groups/{robot_group_id}/mission_groups/{mission_group_id}
[**robot_groups_robot_group_id_mission_groups_post**](MissionGroupsApi.md#robot_groups_robot_group_id_mission_groups_post) | **POST** /robot_groups/{robot_group_id}/mission_groups | POST /robot_groups/{robot_group_id}/mission_groups


# **mission_groups_get**
> [GetMissionGroups] mission_groups_get()

GET /mission_groups

Retrieve the list of mission groups

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import mission_groups_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_mission_groups import GetMissionGroups
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
    api_instance = mission_groups_api.MissionGroupsApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /mission_groups
        api_response = api_instance.mission_groups_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MissionGroupsApi->mission_groups_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetMissionGroups]**](GetMissionGroups.md)

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

# **mission_groups_group_id_missions_get**
> [GetGroupMissions] mission_groups_group_id_missions_get(group_id)

GET /mission_groups/{group_id}/missions

Retrieve the list of missions that belong to the group with the specified group ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import mission_groups_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_group_missions import GetGroupMissions
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
    api_instance = mission_groups_api.MissionGroupsApi(api_client)
    group_id = "group_id_example" # str | The group_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /mission_groups/{group_id}/missions
        api_response = api_instance.mission_groups_group_id_missions_get(group_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MissionGroupsApi->mission_groups_group_id_missions_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **group_id** | **str**| The group_id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetGroupMissions]**](GetGroupMissions.md)

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

# **mission_groups_guid_delete**
> mission_groups_guid_delete(guid)

DELETE /mission_groups/{guid}

Erase the mission group with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import mission_groups_api
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
    api_instance = mission_groups_api.MissionGroupsApi(api_client)
    guid = "guid_example" # str | The guid to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /mission_groups/{guid}
        api_instance.mission_groups_guid_delete(guid)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MissionGroupsApi->mission_groups_guid_delete: %s\n" % e)
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

# **mission_groups_guid_get**
> GetMissionGroup mission_groups_guid_get(guid)

GET /mission_groups/{guid}

Retrieve the details about the mission group with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import mission_groups_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_mission_group import GetMissionGroup
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
    api_instance = mission_groups_api.MissionGroupsApi(api_client)
    guid = "guid_example" # str | The guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /mission_groups/{guid}
        api_response = api_instance.mission_groups_guid_get(guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MissionGroupsApi->mission_groups_guid_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetMissionGroup**](GetMissionGroup.md)

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

# **mission_groups_guid_put**
> GetMissionGroup mission_groups_guid_put(guid, mission_group)

PUT /mission_groups/{guid}

Modify the values of the mission group with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import mission_groups_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.put_mission_group import PutMissionGroup
from mir_fleet_client.model.get_mission_group import GetMissionGroup
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
    api_instance = mission_groups_api.MissionGroupsApi(api_client)
    guid = "guid_example" # str | The guid to modify
    mission_group = PutMissionGroup(
        feature="feature_example",
        icon='YQ==',
        name="name_example",
        priority=1,
    ) # PutMissionGroup | The new values of the mission_group

    # example passing only required values which don't have defaults set
    try:
        # PUT /mission_groups/{guid}
        api_response = api_instance.mission_groups_guid_put(guid, mission_group)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MissionGroupsApi->mission_groups_guid_put: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to modify |
 **mission_group** | [**PutMissionGroup**](PutMissionGroup.md)| The new values of the mission_group |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetMissionGroup**](GetMissionGroup.md)

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

# **mission_groups_mission_group_id_actions_get**
> GetGroupActionDefinition mission_groups_mission_group_id_actions_get(mission_group_id)

GET /mission_groups/{mission_group_id}/actions

Retrieve the list of action definitions from the mission group with the specified mission group ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import mission_groups_api
from mir_fleet_client.model.get_group_action_definition import GetGroupActionDefinition
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
    api_instance = mission_groups_api.MissionGroupsApi(api_client)
    mission_group_id = "mission_group_id_example" # str | The mission_group_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /mission_groups/{mission_group_id}/actions
        api_response = api_instance.mission_groups_mission_group_id_actions_get(mission_group_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MissionGroupsApi->mission_groups_mission_group_id_actions_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **mission_group_id** | **str**| The mission_group_id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetGroupActionDefinition**](GetGroupActionDefinition.md)

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

# **mission_groups_post**
> GetMissionGroups mission_groups_post(mission_groups)

POST /mission_groups

Add a new mission group

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import mission_groups_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.post_mission_groups import PostMissionGroups
from mir_fleet_client.model.get_mission_groups import GetMissionGroups
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
    api_instance = mission_groups_api.MissionGroupsApi(api_client)
    mission_groups = PostMissionGroups(
        created_by_id="created_by_id_example",
        feature="feature_example",
        guid="guid_example",
        icon='YQ==',
        name="name_example",
        priority=1,
    ) # PostMissionGroups | The details of the mission_groups

    # example passing only required values which don't have defaults set
    try:
        # POST /mission_groups
        api_response = api_instance.mission_groups_post(mission_groups)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MissionGroupsApi->mission_groups_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **mission_groups** | [**PostMissionGroups**](PostMissionGroups.md)| The details of the mission_groups |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetMissionGroups**](GetMissionGroups.md)

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
from mir_fleet_client.api import mission_groups_api
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
    api_instance = mission_groups_api.MissionGroupsApi(api_client)
    robot_group_id = 1 # int | The robot_group_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /robot_groups/{robot_group_id}/mission_groups
        api_response = api_instance.robot_groups_robot_group_id_mission_groups_get(robot_group_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MissionGroupsApi->robot_groups_robot_group_id_mission_groups_get: %s\n" % e)
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
from mir_fleet_client.api import mission_groups_api
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
    api_instance = mission_groups_api.MissionGroupsApi(api_client)
    robot_group_id = 1 # int | The robot_group_id to delete
    mission_group_id = "mission_group_id_example" # str | The mission_group_id to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /robot_groups/{robot_group_id}/mission_groups/{mission_group_id}
        api_instance.robot_groups_robot_group_id_mission_groups_mission_group_id_delete(robot_group_id, mission_group_id)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MissionGroupsApi->robot_groups_robot_group_id_mission_groups_mission_group_id_delete: %s\n" % e)
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
from mir_fleet_client.api import mission_groups_api
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
    api_instance = mission_groups_api.MissionGroupsApi(api_client)
    robot_group_id = 1 # int | The robot_group_id to search for
    mission_group_id = "mission_group_id_example" # str | The mission_group_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /robot_groups/{robot_group_id}/mission_groups/{mission_group_id}
        api_response = api_instance.robot_groups_robot_group_id_mission_groups_mission_group_id_get(robot_group_id, mission_group_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MissionGroupsApi->robot_groups_robot_group_id_mission_groups_mission_group_id_get: %s\n" % e)
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
from mir_fleet_client.api import mission_groups_api
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
    api_instance = mission_groups_api.MissionGroupsApi(api_client)
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
        print("Exception when calling MissionGroupsApi->robot_groups_robot_group_id_mission_groups_post: %s\n" % e)
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

