# mir_fleet_client.AreaEventsApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**area_events_action_definitions_action_type_get**](AreaEventsApi.md#area_events_action_definitions_action_type_get) | **GET** /area_events/action_definitions/{action_type} | GET /area_events/action_definitions/{action_type}
[**area_events_action_definitions_get**](AreaEventsApi.md#area_events_action_definitions_get) | **GET** /area_events/action_definitions | GET /area_events/action_definitions
[**area_events_definitions_get**](AreaEventsApi.md#area_events_definitions_get) | **GET** /area_events/definitions | GET /area_events/definitions
[**area_events_get**](AreaEventsApi.md#area_events_get) | **GET** /area_events | GET /area_events
[**area_events_guid_blocked_put**](AreaEventsApi.md#area_events_guid_blocked_put) | **PUT** /area_events/{guid}/blocked | PUT /area_events/{guid}/blocked
[**area_events_guid_delete**](AreaEventsApi.md#area_events_guid_delete) | **DELETE** /area_events/{guid} | DELETE /area_events/{guid}
[**area_events_guid_get**](AreaEventsApi.md#area_events_guid_get) | **GET** /area_events/{guid} | GET /area_events/{guid}
[**area_events_guid_put**](AreaEventsApi.md#area_events_guid_put) | **PUT** /area_events/{guid} | PUT /area_events/{guid}
[**area_events_post**](AreaEventsApi.md#area_events_post) | **POST** /area_events | POST /area_events
[**maps_map_id_area_events_get**](AreaEventsApi.md#maps_map_id_area_events_get) | **GET** /maps/{map_id}/area_events | GET /maps/{map_id}/area_events


# **area_events_action_definitions_action_type_get**
> GetAreaActionDefinition area_events_action_definitions_action_type_get(action_type)

GET /area_events/action_definitions/{action_type}

Retrieve the details about the action. It displays the parameters of the action and the limits for the values among others

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import area_events_api
from mir_fleet_client.model.get_area_action_definition import GetAreaActionDefinition
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
    api_instance = area_events_api.AreaEventsApi(api_client)
    action_type = "action_type_example" # str | The action_type to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /area_events/action_definitions/{action_type}
        api_response = api_instance.area_events_action_definitions_action_type_get(action_type)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling AreaEventsApi->area_events_action_definitions_action_type_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **action_type** | **str**| The action_type to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetAreaActionDefinition**](GetAreaActionDefinition.md)

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

# **area_events_action_definitions_get**
> GetAreaActionDefinitions area_events_action_definitions_get()

GET /area_events/action_definitions

Retrieve definitions of area actions and their parameters

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import area_events_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_area_action_definitions import GetAreaActionDefinitions
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
    api_instance = area_events_api.AreaEventsApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /area_events/action_definitions
        api_response = api_instance.area_events_action_definitions_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling AreaEventsApi->area_events_action_definitions_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetAreaActionDefinitions**](GetAreaActionDefinitions.md)

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

# **area_events_definitions_get**
> GetAreaEventsDefinitions area_events_definitions_get()

GET /area_events/definitions

Retrieve definitions of areas and their actions

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import area_events_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_area_events_definitions import GetAreaEventsDefinitions
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
    api_instance = area_events_api.AreaEventsApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /area_events/definitions
        api_response = api_instance.area_events_definitions_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling AreaEventsApi->area_events_definitions_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetAreaEventsDefinitions**](GetAreaEventsDefinitions.md)

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

# **area_events_get**
> [GetAreaEvents] area_events_get()

GET /area_events

Retrieve the list of area events

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import area_events_api
from mir_fleet_client.model.get_area_events import GetAreaEvents
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
    api_instance = area_events_api.AreaEventsApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /area_events
        api_response = api_instance.area_events_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling AreaEventsApi->area_events_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetAreaEvents]**](GetAreaEvents.md)

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

# **area_events_guid_blocked_put**
> GetBlockArea area_events_guid_blocked_put(guid, block_area)

PUT /area_events/{guid}/blocked

Block or unblock entry for a Limit-robots zone. While blocked robots are not allowed to enter the zone no matter the specified limit.

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import area_events_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.put_block_area import PutBlockArea
from mir_fleet_client.model.get_block_area import GetBlockArea
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
    api_instance = area_events_api.AreaEventsApi(api_client)
    guid = "guid_example" # str | The guid to modify
    block_area = PutBlockArea(
        block=True,
    ) # PutBlockArea | The new values of the block_area

    # example passing only required values which don't have defaults set
    try:
        # PUT /area_events/{guid}/blocked
        api_response = api_instance.area_events_guid_blocked_put(guid, block_area)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling AreaEventsApi->area_events_guid_blocked_put: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to modify |
 **block_area** | [**PutBlockArea**](PutBlockArea.md)| The new values of the block_area |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetBlockArea**](GetBlockArea.md)

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

# **area_events_guid_delete**
> area_events_guid_delete(guid)

DELETE /area_events/{guid}

Erase the area event with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import area_events_api
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
    api_instance = area_events_api.AreaEventsApi(api_client)
    guid = "guid_example" # str | The guid to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /area_events/{guid}
        api_instance.area_events_guid_delete(guid)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling AreaEventsApi->area_events_guid_delete: %s\n" % e)
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

# **area_events_guid_get**
> GetAreaEvent area_events_guid_get(guid)

GET /area_events/{guid}

Retrieve the details about the area event with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import area_events_api
from mir_fleet_client.model.get_area_event import GetAreaEvent
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
    api_instance = area_events_api.AreaEventsApi(api_client)
    guid = "guid_example" # str | The guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /area_events/{guid}
        api_response = api_instance.area_events_guid_get(guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling AreaEventsApi->area_events_guid_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetAreaEvent**](GetAreaEvent.md)

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

# **area_events_guid_put**
> GetAreaEvent area_events_guid_put(guid, area_event)

PUT /area_events/{guid}

Modify the values of the area event with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import area_events_api
from mir_fleet_client.model.get_area_event import GetAreaEvent
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.put_area_event import PutAreaEvent
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
    api_instance = area_events_api.AreaEventsApi(api_client)
    guid = "guid_example" # str | The guid to modify
    area_event = PutAreaEvent(
        actions=[
            {},
        ],
        name="name_example",
        polygon=[
            {},
        ],
    ) # PutAreaEvent | The new values of the area_event

    # example passing only required values which don't have defaults set
    try:
        # PUT /area_events/{guid}
        api_response = api_instance.area_events_guid_put(guid, area_event)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling AreaEventsApi->area_events_guid_put: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to modify |
 **area_event** | [**PutAreaEvent**](PutAreaEvent.md)| The new values of the area_event |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetAreaEvent**](GetAreaEvent.md)

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

# **area_events_post**
> GetAreaEvents area_events_post(area_events)

POST /area_events

Add a new area event

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import area_events_api
from mir_fleet_client.model.post_area_events import PostAreaEvents
from mir_fleet_client.model.get_area_events import GetAreaEvents
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
    api_instance = area_events_api.AreaEventsApi(api_client)
    area_events = PostAreaEvents(
        actions=[
            {},
        ],
        created_by_id="created_by_id_example",
        guid="guid_example",
        map_id="map_id_example",
        name="name_example",
        polygon=[
            {},
        ],
        type_id=1,
    ) # PostAreaEvents | The details of the area_events

    # example passing only required values which don't have defaults set
    try:
        # POST /area_events
        api_response = api_instance.area_events_post(area_events)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling AreaEventsApi->area_events_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **area_events** | [**PostAreaEvents**](PostAreaEvents.md)| The details of the area_events |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetAreaEvents**](GetAreaEvents.md)

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

# **maps_map_id_area_events_get**
> [GetMapAreaEvent] maps_map_id_area_events_get(map_id)

GET /maps/{map_id}/area_events

Retrieve the list of area events that belong to the map with the specified map ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import area_events_api
from mir_fleet_client.model.get_map_area_event import GetMapAreaEvent
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
    api_instance = area_events_api.AreaEventsApi(api_client)
    map_id = "map_id_example" # str | The map_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /maps/{map_id}/area_events
        api_response = api_instance.maps_map_id_area_events_get(map_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling AreaEventsApi->maps_map_id_area_events_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **map_id** | **str**| The map_id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetMapAreaEvent]**](GetMapAreaEvent.md)

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

