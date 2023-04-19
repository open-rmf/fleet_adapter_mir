# mir_fleet_client.ActionsApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**actions_action_type_get**](ActionsApi.md#actions_action_type_get) | **GET** /actions/{action_type} | GET /actions/{action_type}
[**actions_action_type_post**](ActionsApi.md#actions_action_type_post) | **POST** /actions/{action_type} | POST /actions/{action_type}
[**actions_get**](ActionsApi.md#actions_get) | **GET** /actions | GET /actions
[**mission_groups_mission_group_id_actions_get**](ActionsApi.md#mission_groups_mission_group_id_actions_get) | **GET** /mission_groups/{mission_group_id}/actions | GET /mission_groups/{mission_group_id}/actions
[**missions_mission_id_actions_get**](ActionsApi.md#missions_mission_id_actions_get) | **GET** /missions/{mission_id}/actions | GET /missions/{mission_id}/actions
[**missions_mission_id_actions_guid_delete**](ActionsApi.md#missions_mission_id_actions_guid_delete) | **DELETE** /missions/{mission_id}/actions/{guid} | DELETE /missions/{mission_id}/actions/{guid}
[**missions_mission_id_actions_guid_get**](ActionsApi.md#missions_mission_id_actions_guid_get) | **GET** /missions/{mission_id}/actions/{guid} | GET /missions/{mission_id}/actions/{guid}
[**missions_mission_id_actions_guid_put**](ActionsApi.md#missions_mission_id_actions_guid_put) | **PUT** /missions/{mission_id}/actions/{guid} | PUT /missions/{mission_id}/actions/{guid}
[**missions_mission_id_actions_post**](ActionsApi.md#missions_mission_id_actions_post) | **POST** /missions/{mission_id}/actions | POST /missions/{mission_id}/actions


# **actions_action_type_get**
> GetActionDefinition actions_action_type_get(action_type)

GET /actions/{action_type}

Retrieve the details about the action. It displays the parameters of the action and the limits for the values among others

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import actions_api
from mir_fleet_client.model.get_action_definition import GetActionDefinition
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
    api_instance = actions_api.ActionsApi(api_client)
    action_type = "action_type_example" # str | The action_type to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /actions/{action_type}
        api_response = api_instance.actions_action_type_get(action_type)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ActionsApi->actions_action_type_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **action_type** | **str**| The action_type to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetActionDefinition**](GetActionDefinition.md)

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

# **actions_action_type_post**
> GetActionDefinition actions_action_type_post(action_type, action_definition)

POST /actions/{action_type}

Add a new action definition with the specified action_type

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import actions_api
from mir_fleet_client.model.get_action_definition import GetActionDefinition
from mir_fleet_client.model.post_action_definition import PostActionDefinition
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
    api_instance = actions_api.ActionsApi(api_client)
    action_type = "action_type_example" # str | The action_type to add the new resource to
    action_definition = PostActionDefinition(
        parameters=[
            {},
        ],
    ) # PostActionDefinition | The details of the action_definition

    # example passing only required values which don't have defaults set
    try:
        # POST /actions/{action_type}
        api_response = api_instance.actions_action_type_post(action_type, action_definition)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ActionsApi->actions_action_type_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **action_type** | **str**| The action_type to add the new resource to |
 **action_definition** | [**PostActionDefinition**](PostActionDefinition.md)| The details of the action_definition |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetActionDefinition**](GetActionDefinition.md)

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

# **actions_get**
> GetActionDefinitions actions_get()

GET /actions

Retrieve the list of action definitions

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import actions_api
from mir_fleet_client.model.get_action_definitions import GetActionDefinitions
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
    api_instance = actions_api.ActionsApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /actions
        api_response = api_instance.actions_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ActionsApi->actions_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetActionDefinitions**](GetActionDefinitions.md)

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

# **mission_groups_mission_group_id_actions_get**
> GetGroupActionDefinition mission_groups_mission_group_id_actions_get(mission_group_id)

GET /mission_groups/{mission_group_id}/actions

Retrieve the list of action definitions from the mission group with the specified mission group ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import actions_api
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
    api_instance = actions_api.ActionsApi(api_client)
    mission_group_id = "mission_group_id_example" # str | The mission_group_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /mission_groups/{mission_group_id}/actions
        api_response = api_instance.mission_groups_mission_group_id_actions_get(mission_group_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ActionsApi->mission_groups_mission_group_id_actions_get: %s\n" % e)
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

# **missions_mission_id_actions_get**
> [GetMissionActions] missions_mission_id_actions_get(mission_id)

GET /missions/{mission_id}/actions

Retrieve the list of actions that belong to the mission with the specified mission ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import actions_api
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
    api_instance = actions_api.ActionsApi(api_client)
    mission_id = "mission_id_example" # str | The mission_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /missions/{mission_id}/actions
        api_response = api_instance.missions_mission_id_actions_get(mission_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ActionsApi->missions_mission_id_actions_get: %s\n" % e)
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
from mir_fleet_client.api import actions_api
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
    api_instance = actions_api.ActionsApi(api_client)
    mission_id = "mission_id_example" # str | The mission_id to delete
    guid = "guid_example" # str | The guid to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /missions/{mission_id}/actions/{guid}
        api_instance.missions_mission_id_actions_guid_delete(mission_id, guid)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ActionsApi->missions_mission_id_actions_guid_delete: %s\n" % e)
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
from mir_fleet_client.api import actions_api
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
    api_instance = actions_api.ActionsApi(api_client)
    mission_id = "mission_id_example" # str | The mission_id to search for
    guid = "guid_example" # str | The guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /missions/{mission_id}/actions/{guid}
        api_response = api_instance.missions_mission_id_actions_guid_get(mission_id, guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ActionsApi->missions_mission_id_actions_guid_get: %s\n" % e)
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
from mir_fleet_client.api import actions_api
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
    api_instance = actions_api.ActionsApi(api_client)
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
        print("Exception when calling ActionsApi->missions_mission_id_actions_guid_put: %s\n" % e)
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
from mir_fleet_client.api import actions_api
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
    api_instance = actions_api.ActionsApi(api_client)
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
        print("Exception when calling ActionsApi->missions_mission_id_actions_post: %s\n" % e)
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

