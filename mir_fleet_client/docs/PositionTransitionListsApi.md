# mir_fleet_client.PositionTransitionListsApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**position_transition_lists_get**](PositionTransitionListsApi.md#position_transition_lists_get) | **GET** /position_transition_lists | GET /position_transition_lists
[**position_transition_lists_guid_delete**](PositionTransitionListsApi.md#position_transition_lists_guid_delete) | **DELETE** /position_transition_lists/{guid} | DELETE /position_transition_lists/{guid}
[**position_transition_lists_guid_get**](PositionTransitionListsApi.md#position_transition_lists_guid_get) | **GET** /position_transition_lists/{guid} | GET /position_transition_lists/{guid}
[**position_transition_lists_guid_put**](PositionTransitionListsApi.md#position_transition_lists_guid_put) | **PUT** /position_transition_lists/{guid} | PUT /position_transition_lists/{guid}
[**position_transition_lists_post**](PositionTransitionListsApi.md#position_transition_lists_post) | **POST** /position_transition_lists | POST /position_transition_lists
[**sessions_session_id_position_transition_lists_get**](PositionTransitionListsApi.md#sessions_session_id_position_transition_lists_get) | **GET** /sessions/{session_id}/position_transition_lists | GET /sessions/{session_id}/position_transition_lists


# **position_transition_lists_get**
> GetPositionTransitionLists position_transition_lists_get()

GET /position_transition_lists

Retrieve the list of position transition lists

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import position_transition_lists_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_position_transition_lists import GetPositionTransitionLists
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
    api_instance = position_transition_lists_api.PositionTransitionListsApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /position_transition_lists
        api_response = api_instance.position_transition_lists_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PositionTransitionListsApi->position_transition_lists_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetPositionTransitionLists**](GetPositionTransitionLists.md)

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

# **position_transition_lists_guid_delete**
> position_transition_lists_guid_delete(guid)

DELETE /position_transition_lists/{guid}

Erase the position transition list with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import position_transition_lists_api
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
    api_instance = position_transition_lists_api.PositionTransitionListsApi(api_client)
    guid = "guid_example" # str | The guid to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /position_transition_lists/{guid}
        api_instance.position_transition_lists_guid_delete(guid)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PositionTransitionListsApi->position_transition_lists_guid_delete: %s\n" % e)
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

# **position_transition_lists_guid_get**
> GetPositionTransitionList position_transition_lists_guid_get(guid)

GET /position_transition_lists/{guid}

Retrieve the details about the position transition list with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import position_transition_lists_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_position_transition_list import GetPositionTransitionList
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
    api_instance = position_transition_lists_api.PositionTransitionListsApi(api_client)
    guid = "guid_example" # str | The guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /position_transition_lists/{guid}
        api_response = api_instance.position_transition_lists_guid_get(guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PositionTransitionListsApi->position_transition_lists_guid_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetPositionTransitionList**](GetPositionTransitionList.md)

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

# **position_transition_lists_guid_put**
> GetPositionTransitionList position_transition_lists_guid_put(guid, position_transition_list)

PUT /position_transition_lists/{guid}

Modify the values of the position transition list with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import position_transition_lists_api
from mir_fleet_client.model.put_position_transition_list import PutPositionTransitionList
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_position_transition_list import GetPositionTransitionList
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
    api_instance = position_transition_lists_api.PositionTransitionListsApi(api_client)
    guid = "guid_example" # str | The guid to modify
    position_transition_list = PutPositionTransitionList(
        goal_pos_id="goal_pos_id_example",
        mission_id="mission_id_example",
        start_pos_id="start_pos_id_example",
    ) # PutPositionTransitionList | The new values of the position_transition_list

    # example passing only required values which don't have defaults set
    try:
        # PUT /position_transition_lists/{guid}
        api_response = api_instance.position_transition_lists_guid_put(guid, position_transition_list)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PositionTransitionListsApi->position_transition_lists_guid_put: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to modify |
 **position_transition_list** | [**PutPositionTransitionList**](PutPositionTransitionList.md)| The new values of the position_transition_list |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetPositionTransitionList**](GetPositionTransitionList.md)

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

# **position_transition_lists_post**
> GetPositionTransitionLists position_transition_lists_post(position_transition_lists)

POST /position_transition_lists

Add a new position transition list

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import position_transition_lists_api
from mir_fleet_client.model.post_position_transition_lists import PostPositionTransitionLists
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_position_transition_lists import GetPositionTransitionLists
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
    api_instance = position_transition_lists_api.PositionTransitionListsApi(api_client)
    position_transition_lists = PostPositionTransitionLists(
        created_by_id="created_by_id_example",
        goal_pos_id="goal_pos_id_example",
        guid="guid_example",
        mission_id="mission_id_example",
        start_pos_id="start_pos_id_example",
    ) # PostPositionTransitionLists | The details of the position_transition_lists

    # example passing only required values which don't have defaults set
    try:
        # POST /position_transition_lists
        api_response = api_instance.position_transition_lists_post(position_transition_lists)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PositionTransitionListsApi->position_transition_lists_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **position_transition_lists** | [**PostPositionTransitionLists**](PostPositionTransitionLists.md)| The details of the position_transition_lists |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetPositionTransitionLists**](GetPositionTransitionLists.md)

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

# **sessions_session_id_position_transition_lists_get**
> GetPositionTransitionListFromSession sessions_session_id_position_transition_lists_get(session_id)

GET /sessions/{session_id}/position_transition_lists

Retrieve the list of position transition lists that belong to the session with the specified session ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import position_transition_lists_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_position_transition_list_from_session import GetPositionTransitionListFromSession
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
    api_instance = position_transition_lists_api.PositionTransitionListsApi(api_client)
    session_id = "session_id_example" # str | The session_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /sessions/{session_id}/position_transition_lists
        api_response = api_instance.sessions_session_id_position_transition_lists_get(session_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PositionTransitionListsApi->sessions_session_id_position_transition_lists_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **session_id** | **str**| The session_id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetPositionTransitionListFromSession**](GetPositionTransitionListFromSession.md)

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

