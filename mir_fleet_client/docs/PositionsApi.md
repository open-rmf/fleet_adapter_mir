# mir_fleet_client.PositionsApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**maps_map_id_positions_get**](PositionsApi.md#maps_map_id_positions_get) | **GET** /maps/{map_id}/positions | GET /maps/{map_id}/positions
[**path_guides_path_guide_guid_positions_get**](PositionsApi.md#path_guides_path_guide_guid_positions_get) | **GET** /path_guides/{path_guide_guid}/positions | GET /path_guides/{path_guide_guid}/positions
[**path_guides_path_guide_guid_positions_guid_delete**](PositionsApi.md#path_guides_path_guide_guid_positions_guid_delete) | **DELETE** /path_guides/{path_guide_guid}/positions/{guid} | DELETE /path_guides/{path_guide_guid}/positions/{guid}
[**path_guides_path_guide_guid_positions_guid_get**](PositionsApi.md#path_guides_path_guide_guid_positions_guid_get) | **GET** /path_guides/{path_guide_guid}/positions/{guid} | GET /path_guides/{path_guide_guid}/positions/{guid}
[**path_guides_path_guide_guid_positions_guid_put**](PositionsApi.md#path_guides_path_guide_guid_positions_guid_put) | **PUT** /path_guides/{path_guide_guid}/positions/{guid} | PUT /path_guides/{path_guide_guid}/positions/{guid}
[**path_guides_path_guide_guid_positions_post**](PositionsApi.md#path_guides_path_guide_guid_positions_post) | **POST** /path_guides/{path_guide_guid}/positions | POST /path_guides/{path_guide_guid}/positions
[**positions_get**](PositionsApi.md#positions_get) | **GET** /positions | GET /positions
[**positions_guid_delete**](PositionsApi.md#positions_guid_delete) | **DELETE** /positions/{guid} | DELETE /positions/{guid}
[**positions_guid_get**](PositionsApi.md#positions_guid_get) | **GET** /positions/{guid} | GET /positions/{guid}
[**positions_guid_put**](PositionsApi.md#positions_guid_put) | **PUT** /positions/{guid} | PUT /positions/{guid}
[**positions_parent_guid_helper_positions_get**](PositionsApi.md#positions_parent_guid_helper_positions_get) | **GET** /positions/{parent_guid}/helper_positions | GET /positions/{parent_guid}/helper_positions
[**positions_pos_id_docking_offsets_get**](PositionsApi.md#positions_pos_id_docking_offsets_get) | **GET** /positions/{pos_id}/docking_offsets | GET /positions/{pos_id}/docking_offsets
[**positions_post**](PositionsApi.md#positions_post) | **POST** /positions | POST /positions
[**resources_positions_get**](PositionsApi.md#resources_positions_get) | **GET** /resources/positions | GET /resources/positions
[**resources_positions_guid_get**](PositionsApi.md#resources_positions_guid_get) | **GET** /resources/positions/{guid} | GET /resources/positions/{guid}


# **maps_map_id_positions_get**
> [GetMapPositions] maps_map_id_positions_get(map_id)

GET /maps/{map_id}/positions

Retrieve the list of positions that belong to the map with the specified map ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import positions_api
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
    api_instance = positions_api.PositionsApi(api_client)
    map_id = "map_id_example" # str | The map_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /maps/{map_id}/positions
        api_response = api_instance.maps_map_id_positions_get(map_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PositionsApi->maps_map_id_positions_get: %s\n" % e)
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

# **path_guides_path_guide_guid_positions_get**
> [GetPathGuidePositions] path_guides_path_guide_guid_positions_get(path_guide_guid)

GET /path_guides/{path_guide_guid}/positions

Retrieve the list of positions for the path guide with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import positions_api
from mir_fleet_client.model.get_path_guide_positions import GetPathGuidePositions
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
    api_instance = positions_api.PositionsApi(api_client)
    path_guide_guid = "path_guide_guid_example" # str | The path_guide_guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /path_guides/{path_guide_guid}/positions
        api_response = api_instance.path_guides_path_guide_guid_positions_get(path_guide_guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PositionsApi->path_guides_path_guide_guid_positions_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **path_guide_guid** | **str**| The path_guide_guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetPathGuidePositions]**](GetPathGuidePositions.md)

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

# **path_guides_path_guide_guid_positions_guid_delete**
> path_guides_path_guide_guid_positions_guid_delete(path_guide_guid, guid)

DELETE /path_guides/{path_guide_guid}/positions/{guid}

Erase the position with the specified GUID from the path guide with the specified path guide GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import positions_api
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
    api_instance = positions_api.PositionsApi(api_client)
    path_guide_guid = "path_guide_guid_example" # str | The path_guide_guid to delete
    guid = "guid_example" # str | The guid to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /path_guides/{path_guide_guid}/positions/{guid}
        api_instance.path_guides_path_guide_guid_positions_guid_delete(path_guide_guid, guid)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PositionsApi->path_guides_path_guide_guid_positions_guid_delete: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **path_guide_guid** | **str**| The path_guide_guid to delete |
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

# **path_guides_path_guide_guid_positions_guid_get**
> GetPathGuidePosition path_guides_path_guide_guid_positions_guid_get(path_guide_guid, guid)

GET /path_guides/{path_guide_guid}/positions/{guid}

Retrieve the position with the specified GUID from the path guide with the specified path guide GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import positions_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_path_guide_position import GetPathGuidePosition
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
    api_instance = positions_api.PositionsApi(api_client)
    path_guide_guid = "path_guide_guid_example" # str | The path_guide_guid to search for
    guid = "guid_example" # str | The guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /path_guides/{path_guide_guid}/positions/{guid}
        api_response = api_instance.path_guides_path_guide_guid_positions_guid_get(path_guide_guid, guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PositionsApi->path_guides_path_guide_guid_positions_guid_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **path_guide_guid** | **str**| The path_guide_guid to search for |
 **guid** | **str**| The guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetPathGuidePosition**](GetPathGuidePosition.md)

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

# **path_guides_path_guide_guid_positions_guid_put**
> GetPathGuidePosition path_guides_path_guide_guid_positions_guid_put(path_guide_guid, guid, path_guide_position)

PUT /path_guides/{path_guide_guid}/positions/{guid}

Modify the values of the position with the specified GUID from the path guide with the specified path guide GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import positions_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_path_guide_position import GetPathGuidePosition
from mir_fleet_client.model.put_path_guide_position import PutPathGuidePosition
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
    api_instance = positions_api.PositionsApi(api_client)
    path_guide_guid = "path_guide_guid_example" # str | The path_guide_guid to modify
    guid = "guid_example" # str | The guid to modify
    path_guide_position = PutPathGuidePosition(
        pos_guid="pos_guid_example",
        priority=1,
    ) # PutPathGuidePosition | The new values of the path_guide_position

    # example passing only required values which don't have defaults set
    try:
        # PUT /path_guides/{path_guide_guid}/positions/{guid}
        api_response = api_instance.path_guides_path_guide_guid_positions_guid_put(path_guide_guid, guid, path_guide_position)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PositionsApi->path_guides_path_guide_guid_positions_guid_put: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **path_guide_guid** | **str**| The path_guide_guid to modify |
 **guid** | **str**| The guid to modify |
 **path_guide_position** | [**PutPathGuidePosition**](PutPathGuidePosition.md)| The new values of the path_guide_position |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetPathGuidePosition**](GetPathGuidePosition.md)

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

# **path_guides_path_guide_guid_positions_post**
> GetPathGuidePositions path_guides_path_guide_guid_positions_post(path_guide_guid, path_guide_positions)

POST /path_guides/{path_guide_guid}/positions

Add a new position to the path guide with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import positions_api
from mir_fleet_client.model.get_path_guide_positions import GetPathGuidePositions
from mir_fleet_client.model.post_path_guide_positions import PostPathGuidePositions
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
    api_instance = positions_api.PositionsApi(api_client)
    path_guide_guid = "path_guide_guid_example" # str | The path_guide_guid to add the new resource to
    path_guide_positions = PostPathGuidePositions(
        guid="guid_example",
        path_guide_guid="path_guide_guid_example",
        pos_guid="pos_guid_example",
        pos_type="pos_type_example",
        priority=1,
    ) # PostPathGuidePositions | The details of the path_guide_positions

    # example passing only required values which don't have defaults set
    try:
        # POST /path_guides/{path_guide_guid}/positions
        api_response = api_instance.path_guides_path_guide_guid_positions_post(path_guide_guid, path_guide_positions)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PositionsApi->path_guides_path_guide_guid_positions_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **path_guide_guid** | **str**| The path_guide_guid to add the new resource to |
 **path_guide_positions** | [**PostPathGuidePositions**](PostPathGuidePositions.md)| The details of the path_guide_positions |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetPathGuidePositions**](GetPathGuidePositions.md)

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

# **positions_get**
> [GetPositions] positions_get()

GET /positions

Retrieve the list of positions

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import positions_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_positions import GetPositions
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
    api_instance = positions_api.PositionsApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /positions
        api_response = api_instance.positions_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PositionsApi->positions_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetPositions]**](GetPositions.md)

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

# **positions_guid_delete**
> positions_guid_delete(guid)

DELETE /positions/{guid}

Erase the position with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import positions_api
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
    api_instance = positions_api.PositionsApi(api_client)
    guid = "guid_example" # str | The guid to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /positions/{guid}
        api_instance.positions_guid_delete(guid)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PositionsApi->positions_guid_delete: %s\n" % e)
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

# **positions_guid_get**
> GetPosition positions_guid_get(guid)

GET /positions/{guid}

Retrieve the details about the position with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import positions_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_position import GetPosition
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
    api_instance = positions_api.PositionsApi(api_client)
    guid = "guid_example" # str | The guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /positions/{guid}
        api_response = api_instance.positions_guid_get(guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PositionsApi->positions_guid_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetPosition**](GetPosition.md)

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

# **positions_guid_put**
> GetPosition positions_guid_put(guid, position)

PUT /positions/{guid}

Modify the values of the position with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import positions_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_position import GetPosition
from mir_fleet_client.model.put_position import PutPosition
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
    api_instance = positions_api.PositionsApi(api_client)
    guid = "guid_example" # str | The guid to modify
    position = PutPosition(
        map_id="map_id_example",
        name="name_example",
        orientation=3.14,
        parent_id="parent_id_example",
        pos_x=3.14,
        pos_y=3.14,
        type_id=1,
    ) # PutPosition | The new values of the position

    # example passing only required values which don't have defaults set
    try:
        # PUT /positions/{guid}
        api_response = api_instance.positions_guid_put(guid, position)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PositionsApi->positions_guid_put: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to modify |
 **position** | [**PutPosition**](PutPosition.md)| The new values of the position |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetPosition**](GetPosition.md)

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

# **positions_parent_guid_helper_positions_get**
> [GetHelperPositions] positions_parent_guid_helper_positions_get(parent_guid)

GET /positions/{parent_guid}/helper_positions

Retrieve the list of helper positions for the position with the specified parent GUID. Only Charging Stations, V markers, VL markers, Shelf and Trolley positions have helper positions

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import positions_api
from mir_fleet_client.model.get_helper_positions import GetHelperPositions
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
    api_instance = positions_api.PositionsApi(api_client)
    parent_guid = "parent_guid_example" # str | The parent_guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /positions/{parent_guid}/helper_positions
        api_response = api_instance.positions_parent_guid_helper_positions_get(parent_guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PositionsApi->positions_parent_guid_helper_positions_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **parent_guid** | **str**| The parent_guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetHelperPositions]**](GetHelperPositions.md)

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

# **positions_pos_id_docking_offsets_get**
> [GetPosDockingOffsets] positions_pos_id_docking_offsets_get(pos_id)

GET /positions/{pos_id}/docking_offsets

Retrieve the details of the docking offset of the position with the specified position ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import positions_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_pos_docking_offsets import GetPosDockingOffsets
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
    api_instance = positions_api.PositionsApi(api_client)
    pos_id = "pos_id_example" # str | The pos_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /positions/{pos_id}/docking_offsets
        api_response = api_instance.positions_pos_id_docking_offsets_get(pos_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PositionsApi->positions_pos_id_docking_offsets_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **pos_id** | **str**| The pos_id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetPosDockingOffsets]**](GetPosDockingOffsets.md)

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

# **positions_post**
> GetPositions positions_post(positions)

POST /positions

Add a new position

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import positions_api
from mir_fleet_client.model.post_positions import PostPositions
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_positions import GetPositions
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
    api_instance = positions_api.PositionsApi(api_client)
    positions = PostPositions(
        created_by_id="created_by_id_example",
        guid="guid_example",
        map_id="map_id_example",
        name="name_example",
        orientation=3.14,
        parent_id="parent_id_example",
        pos_x=3.14,
        pos_y=3.14,
        type_id=1,
    ) # PostPositions | The details of the positions

    # example passing only required values which don't have defaults set
    try:
        # POST /positions
        api_response = api_instance.positions_post(positions)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PositionsApi->positions_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **positions** | [**PostPositions**](PostPositions.md)| The details of the positions |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetPositions**](GetPositions.md)

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

# **resources_positions_get**
> [GetResourcePositions] resources_positions_get()

GET /resources/positions

Retrieve the list of position resources in the fleet

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import positions_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_resource_positions import GetResourcePositions
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
    api_instance = positions_api.PositionsApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /resources/positions
        api_response = api_instance.resources_positions_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PositionsApi->resources_positions_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetResourcePositions]**](GetResourcePositions.md)

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

# **resources_positions_guid_get**
> GetResourcePosition resources_positions_guid_get(guid)

GET /resources/positions/{guid}

Retrieve the details about the queue to the resource with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import positions_api
from mir_fleet_client.model.get_resource_position import GetResourcePosition
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
    api_instance = positions_api.PositionsApi(api_client)
    guid = "guid_example" # str | The guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /resources/positions/{guid}
        api_response = api_instance.resources_positions_guid_get(guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PositionsApi->resources_positions_guid_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetResourcePosition**](GetResourcePosition.md)

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

