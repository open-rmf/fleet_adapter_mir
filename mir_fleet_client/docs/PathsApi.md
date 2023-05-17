# mir_fleet_client.PathsApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**maps_map_id_paths_get**](PathsApi.md#maps_map_id_paths_get) | **GET** /maps/{map_id}/paths | GET /maps/{map_id}/paths
[**paths_get**](PathsApi.md#paths_get) | **GET** /paths | GET /paths
[**paths_guid_delete**](PathsApi.md#paths_guid_delete) | **DELETE** /paths/{guid} | DELETE /paths/{guid}
[**paths_guid_get**](PathsApi.md#paths_guid_get) | **GET** /paths/{guid} | GET /paths/{guid}
[**paths_guid_put**](PathsApi.md#paths_guid_put) | **PUT** /paths/{guid} | PUT /paths/{guid}
[**paths_post**](PathsApi.md#paths_post) | **POST** /paths | POST /paths


# **maps_map_id_paths_get**
> [GetMapPaths] maps_map_id_paths_get(map_id)

GET /maps/{map_id}/paths

Retrieve the list of paths that belong to the map with the specified map ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import paths_api
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
    api_instance = paths_api.PathsApi(api_client)
    map_id = "map_id_example" # str | The map_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /maps/{map_id}/paths
        api_response = api_instance.maps_map_id_paths_get(map_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PathsApi->maps_map_id_paths_get: %s\n" % e)
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

# **paths_get**
> [GetPaths] paths_get()

GET /paths

Retrieve the list of paths

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import paths_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_paths import GetPaths
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
    api_instance = paths_api.PathsApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /paths
        api_response = api_instance.paths_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PathsApi->paths_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetPaths]**](GetPaths.md)

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

# **paths_guid_delete**
> paths_guid_delete(guid)

DELETE /paths/{guid}

Erase the path with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import paths_api
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
    api_instance = paths_api.PathsApi(api_client)
    guid = "guid_example" # str | The guid to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /paths/{guid}
        api_instance.paths_guid_delete(guid)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PathsApi->paths_guid_delete: %s\n" % e)
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

# **paths_guid_get**
> GetPath paths_guid_get(guid)

GET /paths/{guid}

Retrieve the path with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import paths_api
from mir_fleet_client.model.get_path import GetPath
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
    api_instance = paths_api.PathsApi(api_client)
    guid = "guid_example" # str | The guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /paths/{guid}
        api_response = api_instance.paths_guid_get(guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PathsApi->paths_guid_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetPath**](GetPath.md)

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

# **paths_guid_put**
> GetPath paths_guid_put(guid, path)

PUT /paths/{guid}

Modify the values of the path with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import paths_api
from mir_fleet_client.model.get_path import GetPath
from mir_fleet_client.model.put_path import PutPath
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
    api_instance = paths_api.PathsApi(api_client)
    guid = "guid_example" # str | The guid to modify
    path = PutPath(
        autogenerated=True,
        footprint="footprint_example",
        goal_pos_id="goal_pos_id_example",
        last_used=dateutil_parser('1970-01-01T00:00:00.00Z'),
        length=3.14,
        map_crc="map_crc_example",
        path='YQ==',
        start_pos_id="start_pos_id_example",
        time=3.14,
        valid=True,
    ) # PutPath | The new values of the path

    # example passing only required values which don't have defaults set
    try:
        # PUT /paths/{guid}
        api_response = api_instance.paths_guid_put(guid, path)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PathsApi->paths_guid_put: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to modify |
 **path** | [**PutPath**](PutPath.md)| The new values of the path |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetPath**](GetPath.md)

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

# **paths_post**
> GetPaths paths_post(paths)

POST /paths

Add a new path

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import paths_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.post_paths import PostPaths
from mir_fleet_client.model.get_paths import GetPaths
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
    api_instance = paths_api.PathsApi(api_client)
    paths = PostPaths(
        autogenerated=True,
        footprint="footprint_example",
        goal_pos_id="goal_pos_id_example",
        guid="guid_example",
        last_used=dateutil_parser('1970-01-01T00:00:00.00Z'),
        length=3.14,
        map_crc="map_crc_example",
        path='YQ==',
        start_pos_id="start_pos_id_example",
        time=3.14,
        valid=True,
    ) # PostPaths | The details of the paths

    # example passing only required values which don't have defaults set
    try:
        # POST /paths
        api_response = api_instance.paths_post(paths)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PathsApi->paths_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **paths** | [**PostPaths**](PostPaths.md)| The details of the paths |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetPaths**](GetPaths.md)

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

