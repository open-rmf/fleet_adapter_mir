# mir_fleet_client.MapsApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**maps_get**](MapsApi.md#maps_get) | **GET** /maps | GET /maps
[**maps_guid_delete**](MapsApi.md#maps_guid_delete) | **DELETE** /maps/{guid} | DELETE /maps/{guid}
[**maps_guid_get**](MapsApi.md#maps_guid_get) | **GET** /maps/{guid} | GET /maps/{guid}
[**maps_guid_put**](MapsApi.md#maps_guid_put) | **PUT** /maps/{guid} | PUT /maps/{guid}
[**maps_map_id_area_events_get**](MapsApi.md#maps_map_id_area_events_get) | **GET** /maps/{map_id}/area_events | GET /maps/{map_id}/area_events
[**maps_map_id_path_guides_get**](MapsApi.md#maps_map_id_path_guides_get) | **GET** /maps/{map_id}/path_guides | GET /maps/{map_id}/path_guides
[**maps_map_id_paths_get**](MapsApi.md#maps_map_id_paths_get) | **GET** /maps/{map_id}/paths | GET /maps/{map_id}/paths
[**maps_map_id_positions_get**](MapsApi.md#maps_map_id_positions_get) | **GET** /maps/{map_id}/positions | GET /maps/{map_id}/positions
[**maps_post**](MapsApi.md#maps_post) | **POST** /maps | POST /maps
[**sessions_session_id_maps_get**](MapsApi.md#sessions_session_id_maps_get) | **GET** /sessions/{session_id}/maps | GET /sessions/{session_id}/maps


# **maps_get**
> [GetMaps] maps_get()

GET /maps

Retrieve the list of maps

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import maps_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_maps import GetMaps
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
    api_instance = maps_api.MapsApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /maps
        api_response = api_instance.maps_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MapsApi->maps_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetMaps]**](GetMaps.md)

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

# **maps_guid_delete**
> maps_guid_delete(guid)

DELETE /maps/{guid}

Erase the map with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import maps_api
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
    api_instance = maps_api.MapsApi(api_client)
    guid = "guid_example" # str | The guid to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /maps/{guid}
        api_instance.maps_guid_delete(guid)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MapsApi->maps_guid_delete: %s\n" % e)
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

# **maps_guid_get**
> GetMap maps_guid_get(guid)

GET /maps/{guid}

Retrieve the details about the map with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import maps_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_map import GetMap
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
    api_instance = maps_api.MapsApi(api_client)
    guid = "guid_example" # str | The guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /maps/{guid}
        api_response = api_instance.maps_guid_get(guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MapsApi->maps_guid_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetMap**](GetMap.md)

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

# **maps_guid_put**
> GetMap maps_guid_put(guid, map)

PUT /maps/{guid}

Modify the values of the map with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import maps_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_map import GetMap
from mir_fleet_client.model.put_map import PutMap
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
    api_instance = maps_api.MapsApi(api_client)
    guid = "guid_example" # str | The guid to modify
    map = PutMap(
        map='YQ==',
        metadata='YQ==',
        name="name_example",
        one_way_map='YQ==',
        origin_theta=3.14,
        origin_x=3.14,
        origin_y=3.14,
        resolution=3.14,
    ) # PutMap | The new values of the map

    # example passing only required values which don't have defaults set
    try:
        # PUT /maps/{guid}
        api_response = api_instance.maps_guid_put(guid, map)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MapsApi->maps_guid_put: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to modify |
 **map** | [**PutMap**](PutMap.md)| The new values of the map |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetMap**](GetMap.md)

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

# **maps_map_id_area_events_get**
> [GetMapAreaEvent] maps_map_id_area_events_get(map_id)

GET /maps/{map_id}/area_events

Retrieve the list of area events that belong to the map with the specified map ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import maps_api
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
    api_instance = maps_api.MapsApi(api_client)
    map_id = "map_id_example" # str | The map_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /maps/{map_id}/area_events
        api_response = api_instance.maps_map_id_area_events_get(map_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MapsApi->maps_map_id_area_events_get: %s\n" % e)
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

# **maps_map_id_path_guides_get**
> [GetMapPathGuides] maps_map_id_path_guides_get(map_id)

GET /maps/{map_id}/path_guides

Retrieve the list of path guides that belong to the map with the specified map ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import maps_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_map_path_guides import GetMapPathGuides
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
    api_instance = maps_api.MapsApi(api_client)
    map_id = "map_id_example" # str | The map_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /maps/{map_id}/path_guides
        api_response = api_instance.maps_map_id_path_guides_get(map_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MapsApi->maps_map_id_path_guides_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **map_id** | **str**| The map_id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetMapPathGuides]**](GetMapPathGuides.md)

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

# **maps_map_id_paths_get**
> [GetMapPaths] maps_map_id_paths_get(map_id)

GET /maps/{map_id}/paths

Retrieve the list of paths that belong to the map with the specified map ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import maps_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_map_paths import GetMapPaths
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
    api_instance = maps_api.MapsApi(api_client)
    map_id = "map_id_example" # str | The map_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /maps/{map_id}/paths
        api_response = api_instance.maps_map_id_paths_get(map_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MapsApi->maps_map_id_paths_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **map_id** | **str**| The map_id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetMapPaths]**](GetMapPaths.md)

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

# **maps_map_id_positions_get**
> [GetMapPositions] maps_map_id_positions_get(map_id)

GET /maps/{map_id}/positions

Retrieve the list of positions that belong to the map with the specified map ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import maps_api
from mir_fleet_client.model.get_map_positions import GetMapPositions
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
    api_instance = maps_api.MapsApi(api_client)
    map_id = "map_id_example" # str | The map_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /maps/{map_id}/positions
        api_response = api_instance.maps_map_id_positions_get(map_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MapsApi->maps_map_id_positions_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **map_id** | **str**| The map_id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetMapPositions]**](GetMapPositions.md)

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

# **maps_post**
> GetMaps maps_post(maps)

POST /maps

Add a new map

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import maps_api
from mir_fleet_client.model.post_maps import PostMaps
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_maps import GetMaps
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
    api_instance = maps_api.MapsApi(api_client)
    maps = PostMaps(
        created_by_id="created_by_id_example",
        guid="guid_example",
        map='YQ==',
        metadata='YQ==',
        name="name_example",
        one_way_map='YQ==',
        origin_theta=3.14,
        origin_x=3.14,
        origin_y=3.14,
        resolution=3.14,
        session_id="session_id_example",
    ) # PostMaps | The details of the maps

    # example passing only required values which don't have defaults set
    try:
        # POST /maps
        api_response = api_instance.maps_post(maps)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MapsApi->maps_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **maps** | [**PostMaps**](PostMaps.md)| The details of the maps |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetMaps**](GetMaps.md)

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

# **sessions_session_id_maps_get**
> [GetSessionMaps] sessions_session_id_maps_get(session_id)

GET /sessions/{session_id}/maps

Retrieve the list of maps that belong to the session with the specified session ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import maps_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_session_maps import GetSessionMaps
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
    api_instance = maps_api.MapsApi(api_client)
    session_id = "session_id_example" # str | The session_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /sessions/{session_id}/maps
        api_response = api_instance.sessions_session_id_maps_get(session_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MapsApi->sessions_session_id_maps_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **session_id** | **str**| The session_id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetSessionMaps]**](GetSessionMaps.md)

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

