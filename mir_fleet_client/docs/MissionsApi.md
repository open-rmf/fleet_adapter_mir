# mir_fleet_client.MissionsApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**mission_groups_group_id_missions_get**](MissionsApi.md#mission_groups_group_id_missions_get) | **GET** /mission_groups/{group_id}/missions | GET /mission_groups/{group_id}/missions
[**missions_get**](MissionsApi.md#missions_get) | **GET** /missions | GET /missions
[**missions_guid_definition_get**](MissionsApi.md#missions_guid_definition_get) | **GET** /missions/{guid}/definition | GET /missions/{guid}/definition
[**missions_guid_delete**](MissionsApi.md#missions_guid_delete) | **DELETE** /missions/{guid} | DELETE /missions/{guid}
[**missions_guid_get**](MissionsApi.md#missions_guid_get) | **GET** /missions/{guid} | GET /missions/{guid}
[**missions_guid_put**](MissionsApi.md#missions_guid_put) | **PUT** /missions/{guid} | PUT /missions/{guid}
[**missions_guid_qualified_robots_get**](MissionsApi.md#missions_guid_qualified_robots_get) | **GET** /missions/{guid}/qualified_robots | GET /missions/{guid}/qualified_robots
[**missions_mission_id_actions_get**](MissionsApi.md#missions_mission_id_actions_get) | **GET** /missions/{mission_id}/actions | GET /missions/{mission_id}/actions
[**missions_mission_id_actions_guid_delete**](MissionsApi.md#missions_mission_id_actions_guid_delete) | **DELETE** /missions/{mission_id}/actions/{guid} | DELETE /missions/{mission_id}/actions/{guid}
[**missions_mission_id_actions_guid_get**](MissionsApi.md#missions_mission_id_actions_guid_get) | **GET** /missions/{mission_id}/actions/{guid} | GET /missions/{mission_id}/actions/{guid}
[**missions_mission_id_actions_guid_put**](MissionsApi.md#missions_mission_id_actions_guid_put) | **PUT** /missions/{mission_id}/actions/{guid} | PUT /missions/{mission_id}/actions/{guid}
[**missions_mission_id_actions_post**](MissionsApi.md#missions_mission_id_actions_post) | **POST** /missions/{mission_id}/actions | POST /missions/{mission_id}/actions
[**missions_post**](MissionsApi.md#missions_post) | **POST** /missions | POST /missions
[**sessions_session_id_missions_get**](MissionsApi.md#sessions_session_id_missions_get) | **GET** /sessions/{session_id}/missions | GET /sessions/{session_id}/missions


# **mission_groups_group_id_missions_get**
> [GetGroupMissions] mission_groups_group_id_missions_get(group_id)

GET /mission_groups/{group_id}/missions

Retrieve the list of missions that belong to the group with the specified group ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import missions_api
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
    api_instance = missions_api.MissionsApi(api_client)
    group_id = "group_id_example" # str | The group_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /mission_groups/{group_id}/missions
        api_response = api_instance.mission_groups_group_id_missions_get(group_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MissionsApi->mission_groups_group_id_missions_get: %s\n" % e)
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

# **missions_get**
> [GetMissions] missions_get()

GET /missions

Retrieve the list of missions

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import missions_api
from mir_fleet_client.model.get_missions import GetMissions
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
    api_instance = missions_api.MissionsApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /missions
        api_response = api_instance.missions_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MissionsApi->missions_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetMissions]**](GetMissions.md)

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

# **missions_guid_definition_get**
> [GetMissionDefinition] missions_guid_definition_get(guid)

GET /missions/{guid}/definition

Retrieve the mission with the specified GUID as an action definition that can be inserted in another mission

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import missions_api
from mir_fleet_client.model.get_mission_definition import GetMissionDefinition
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
    api_instance = missions_api.MissionsApi(api_client)
    guid = "guid_example" # str | The guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /missions/{guid}/definition
        api_response = api_instance.missions_guid_definition_get(guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MissionsApi->missions_guid_definition_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetMissionDefinition]**](GetMissionDefinition.md)

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

# **missions_guid_delete**
> missions_guid_delete(guid)

DELETE /missions/{guid}

Erase the mission with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import missions_api
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
    api_instance = missions_api.MissionsApi(api_client)
    guid = "guid_example" # str | The guid to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /missions/{guid}
        api_instance.missions_guid_delete(guid)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MissionsApi->missions_guid_delete: %s\n" % e)
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

# **missions_guid_get**
> GetMission missions_guid_get(guid)

GET /missions/{guid}

Retrieve the details about the mission with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import missions_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_mission import GetMission
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
    api_instance = missions_api.MissionsApi(api_client)
    guid = "guid_example" # str | The guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /missions/{guid}
        api_response = api_instance.missions_guid_get(guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MissionsApi->missions_guid_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetMission**](GetMission.md)

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

# **missions_guid_put**
> GetMission missions_guid_put(guid, mission)

PUT /missions/{guid}

Modify the values of the mission with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import missions_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_mission import GetMission
from mir_fleet_client.model.put_mission import PutMission
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
    api_instance = missions_api.MissionsApi(api_client)
    guid = "guid_example" # str | The guid to modify
    mission = PutMission(
        description="description_example",
        group_id="group_id_example",
        hidden=True,
        name="name_example",
        session_id="session_id_example",
    ) # PutMission | The new values of the mission

    # example passing only required values which don't have defaults set
    try:
        # PUT /missions/{guid}
        api_response = api_instance.missions_guid_put(guid, mission)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MissionsApi->missions_guid_put: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to modify |
 **mission** | [**PutMission**](PutMission.md)| The new values of the mission |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetMission**](GetMission.md)

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

# **missions_guid_qualified_robots_get**
> [GetQualifiedRobots] missions_guid_qualified_robots_get(guid)

GET /missions/{guid}/qualified_robots

Lists the qualified robots for this mission

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import missions_api
from mir_fleet_client.model.get_qualified_robots import GetQualifiedRobots
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
    api_instance = missions_api.MissionsApi(api_client)
    guid = "guid_example" # str | The guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /missions/{guid}/qualified_robots
        api_response = api_instance.missions_guid_qualified_robots_get(guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MissionsApi->missions_guid_qualified_robots_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetQualifiedRobots]**](GetQualifiedRobots.md)

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

# **missions_mission_id_actions_get**
> [GetMissionActions] missions_mission_id_actions_get(mission_id)

GET /missions/{mission_id}/actions

Retrieve the list of actions that belong to the mission with the specified mission ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import missions_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_mission_actions import GetMissionActions
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
    api_instance = missions_api.MissionsApi(api_client)
    mission_id = "mission_id_example" # str | The mission_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /missions/{mission_id}/actions
        api_response = api_instance.missions_mission_id_actions_get(mission_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MissionsApi->missions_mission_id_actions_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **mission_id** | **str**| The mission_id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetMissionActions]**](GetMissionActions.md)

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

# **missions_mission_id_actions_guid_delete**
> missions_mission_id_actions_guid_delete(mission_id, guid)

DELETE /missions/{mission_id}/actions/{guid}

Erase the action with the specified GUID from the mission with the specified mission ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import missions_api
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
    api_instance = missions_api.MissionsApi(api_client)
    mission_id = "mission_id_example" # str | The mission_id to delete
    guid = "guid_example" # str | The guid to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /missions/{mission_id}/actions/{guid}
        api_instance.missions_mission_id_actions_guid_delete(mission_id, guid)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MissionsApi->missions_mission_id_actions_guid_delete: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **mission_id** | **str**| The mission_id to delete |
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

# **missions_mission_id_actions_guid_get**
> GetMissionAction missions_mission_id_actions_guid_get(mission_id, guid)

GET /missions/{mission_id}/actions/{guid}

Retrieve the details about the action with the specified GUID that belongs to the mission with the specified mission ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import missions_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_mission_action import GetMissionAction
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
    api_instance = missions_api.MissionsApi(api_client)
    mission_id = "mission_id_example" # str | The mission_id to search for
    guid = "guid_example" # str | The guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /missions/{mission_id}/actions/{guid}
        api_response = api_instance.missions_mission_id_actions_guid_get(mission_id, guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MissionsApi->missions_mission_id_actions_guid_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **mission_id** | **str**| The mission_id to search for |
 **guid** | **str**| The guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetMissionAction**](GetMissionAction.md)

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

# **missions_mission_id_actions_guid_put**
> GetMissionAction missions_mission_id_actions_guid_put(mission_id, guid, mission_action)

PUT /missions/{mission_id}/actions/{guid}

Modify the values of the action with the specified GUID that belongs to the mission with the specified mission ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import missions_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_mission_action import GetMissionAction
from mir_fleet_client.model.put_mission_action import PutMissionAction
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
    api_instance = missions_api.MissionsApi(api_client)
    mission_id = "mission_id_example" # str | The mission_id to modify
    guid = "guid_example" # str | The guid to modify
    mission_action = PutMissionAction(
        parameters=[
            {},
        ],
        priority=1,
        scope_reference="scope_reference_example",
    ) # PutMissionAction | The new values of the mission_action

    # example passing only required values which don't have defaults set
    try:
        # PUT /missions/{mission_id}/actions/{guid}
        api_response = api_instance.missions_mission_id_actions_guid_put(mission_id, guid, mission_action)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MissionsApi->missions_mission_id_actions_guid_put: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **mission_id** | **str**| The mission_id to modify |
 **guid** | **str**| The guid to modify |
 **mission_action** | [**PutMissionAction**](PutMissionAction.md)| The new values of the mission_action |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetMissionAction**](GetMissionAction.md)

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

# **missions_mission_id_actions_post**
> GetMissionActions missions_mission_id_actions_post(mission_id, mission_actions)

POST /missions/{mission_id}/actions

Add a new action to the mission with the specified mission ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import missions_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_mission_actions import GetMissionActions
from mir_fleet_client.model.post_mission_actions import PostMissionActions
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
    api_instance = missions_api.MissionsApi(api_client)
    mission_id = "mission_id_example" # str | The mission_id to add the new resource to
    mission_actions = PostMissionActions(
        action_type="action_type_example",
        guid="guid_example",
        mission_id="mission_id_example",
        parameters=[
            {},
        ],
        priority=1,
        scope_reference="scope_reference_example",
    ) # PostMissionActions | The details of the mission_actions

    # example passing only required values which don't have defaults set
    try:
        # POST /missions/{mission_id}/actions
        api_response = api_instance.missions_mission_id_actions_post(mission_id, mission_actions)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MissionsApi->missions_mission_id_actions_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **mission_id** | **str**| The mission_id to add the new resource to |
 **mission_actions** | [**PostMissionActions**](PostMissionActions.md)| The details of the mission_actions |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetMissionActions**](GetMissionActions.md)

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

# **missions_post**
> GetMissions missions_post(missions)

POST /missions

Add a new mission

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import missions_api
from mir_fleet_client.model.get_missions import GetMissions
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.post_missions import PostMissions
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
    api_instance = missions_api.MissionsApi(api_client)
    missions = PostMissions(
        created_by_id="created_by_id_example",
        description="description_example",
        group_id="group_id_example",
        guid="guid_example",
        hidden=True,
        name="name_example",
        session_id="session_id_example",
    ) # PostMissions | The details of the missions

    # example passing only required values which don't have defaults set
    try:
        # POST /missions
        api_response = api_instance.missions_post(missions)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MissionsApi->missions_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **missions** | [**PostMissions**](PostMissions.md)| The details of the missions |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetMissions**](GetMissions.md)

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

# **sessions_session_id_missions_get**
> [GetSessionMissions] sessions_session_id_missions_get(session_id)

GET /sessions/{session_id}/missions

Retrieve the list of missions that belong to the session with the specified session ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import missions_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_session_missions import GetSessionMissions
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
    api_instance = missions_api.MissionsApi(api_client)
    session_id = "session_id_example" # str | The session_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /sessions/{session_id}/missions
        api_response = api_instance.sessions_session_id_missions_get(session_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MissionsApi->sessions_session_id_missions_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **session_id** | **str**| The session_id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetSessionMissions]**](GetSessionMissions.md)

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

