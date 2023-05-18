# mir_fleet_client.PathGuidesApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**maps_map_id_path_guides_get**](PathGuidesApi.md#maps_map_id_path_guides_get) | **GET** /maps/{map_id}/path_guides | GET /maps/{map_id}/path_guides
[**path_guides_get**](PathGuidesApi.md#path_guides_get) | **GET** /path_guides | GET /path_guides
[**path_guides_guid_delete**](PathGuidesApi.md#path_guides_guid_delete) | **DELETE** /path_guides/{guid} | DELETE /path_guides/{guid}
[**path_guides_guid_get**](PathGuidesApi.md#path_guides_guid_get) | **GET** /path_guides/{guid} | GET /path_guides/{guid}
[**path_guides_guid_put**](PathGuidesApi.md#path_guides_guid_put) | **PUT** /path_guides/{guid} | PUT /path_guides/{guid}
[**path_guides_path_guide_guid_options_get**](PathGuidesApi.md#path_guides_path_guide_guid_options_get) | **GET** /path_guides/{path_guide_guid}/options | GET /path_guides/{path_guide_guid}/options
[**path_guides_path_guide_guid_positions_get**](PathGuidesApi.md#path_guides_path_guide_guid_positions_get) | **GET** /path_guides/{path_guide_guid}/positions | GET /path_guides/{path_guide_guid}/positions
[**path_guides_path_guide_guid_positions_guid_delete**](PathGuidesApi.md#path_guides_path_guide_guid_positions_guid_delete) | **DELETE** /path_guides/{path_guide_guid}/positions/{guid} | DELETE /path_guides/{path_guide_guid}/positions/{guid}
[**path_guides_path_guide_guid_positions_guid_get**](PathGuidesApi.md#path_guides_path_guide_guid_positions_guid_get) | **GET** /path_guides/{path_guide_guid}/positions/{guid} | GET /path_guides/{path_guide_guid}/positions/{guid}
[**path_guides_path_guide_guid_positions_guid_put**](PathGuidesApi.md#path_guides_path_guide_guid_positions_guid_put) | **PUT** /path_guides/{path_guide_guid}/positions/{guid} | PUT /path_guides/{path_guide_guid}/positions/{guid}
[**path_guides_path_guide_guid_positions_post**](PathGuidesApi.md#path_guides_path_guide_guid_positions_post) | **POST** /path_guides/{path_guide_guid}/positions | POST /path_guides/{path_guide_guid}/positions
[**path_guides_post**](PathGuidesApi.md#path_guides_post) | **POST** /path_guides | POST /path_guides


# **maps_map_id_path_guides_get**
> [GetMapPathGuides] maps_map_id_path_guides_get(map_id)

GET /maps/{map_id}/path_guides

Retrieve the list of path guides that belong to the map with the specified map ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import path_guides_api
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
    api_instance = path_guides_api.PathGuidesApi(api_client)
    map_id = "map_id_example" # str | The map_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /maps/{map_id}/path_guides
        api_response = api_instance.maps_map_id_path_guides_get(map_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PathGuidesApi->maps_map_id_path_guides_get: %s\n" % e)
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

# **path_guides_get**
> [GetPathGuides] path_guides_get()

GET /path_guides

Retrieve the list of path guides

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import path_guides_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_path_guides import GetPathGuides
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
    api_instance = path_guides_api.PathGuidesApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /path_guides
        api_response = api_instance.path_guides_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PathGuidesApi->path_guides_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetPathGuides]**](GetPathGuides.md)

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

# **path_guides_guid_delete**
> path_guides_guid_delete(guid)

DELETE /path_guides/{guid}

Erase the path guide with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import path_guides_api
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
    api_instance = path_guides_api.PathGuidesApi(api_client)
    guid = "guid_example" # str | The guid to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /path_guides/{guid}
        api_instance.path_guides_guid_delete(guid)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PathGuidesApi->path_guides_guid_delete: %s\n" % e)
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

# **path_guides_guid_get**
> GetPathGuide path_guides_guid_get(guid)

GET /path_guides/{guid}

Retrieve the path guide with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import path_guides_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_path_guide import GetPathGuide
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
    api_instance = path_guides_api.PathGuidesApi(api_client)
    guid = "guid_example" # str | The guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /path_guides/{guid}
        api_response = api_instance.path_guides_guid_get(guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PathGuidesApi->path_guides_guid_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetPathGuide**](GetPathGuide.md)

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

# **path_guides_guid_put**
> GetPathGuide path_guides_guid_put(guid, path_guide)

PUT /path_guides/{guid}

Modify the values of the path guide with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import path_guides_api
from mir_fleet_client.model.put_path_guide import PutPathGuide
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_path_guide import GetPathGuide
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
    api_instance = path_guides_api.PathGuidesApi(api_client)
    guid = "guid_example" # str | The guid to modify
    path_guide = PutPathGuide(
        map_id="map_id_example",
        name="name_example",
    ) # PutPathGuide | The new values of the path_guide

    # example passing only required values which don't have defaults set
    try:
        # PUT /path_guides/{guid}
        api_response = api_instance.path_guides_guid_put(guid, path_guide)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PathGuidesApi->path_guides_guid_put: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to modify |
 **path_guide** | [**PutPathGuide**](PutPathGuide.md)| The new values of the path_guide |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetPathGuide**](GetPathGuide.md)

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

# **path_guides_path_guide_guid_options_get**
> GetPathGuideOptions path_guides_path_guide_guid_options_get(path_guide_guid)

GET /path_guides/{path_guide_guid}/options

Retrieve the list of allowed start/via/goal options for the selected path guide

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import path_guides_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_path_guide_options import GetPathGuideOptions
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
    api_instance = path_guides_api.PathGuidesApi(api_client)
    path_guide_guid = "path_guide_guid_example" # str | The path_guide_guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /path_guides/{path_guide_guid}/options
        api_response = api_instance.path_guides_path_guide_guid_options_get(path_guide_guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PathGuidesApi->path_guides_path_guide_guid_options_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **path_guide_guid** | **str**| The path_guide_guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetPathGuideOptions**](GetPathGuideOptions.md)

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

# **path_guides_path_guide_guid_positions_get**
> [GetPathGuidePositions] path_guides_path_guide_guid_positions_get(path_guide_guid)

GET /path_guides/{path_guide_guid}/positions

Retrieve the list of positions for the path guide with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import path_guides_api
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
    api_instance = path_guides_api.PathGuidesApi(api_client)
    path_guide_guid = "path_guide_guid_example" # str | The path_guide_guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /path_guides/{path_guide_guid}/positions
        api_response = api_instance.path_guides_path_guide_guid_positions_get(path_guide_guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PathGuidesApi->path_guides_path_guide_guid_positions_get: %s\n" % e)
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
from mir_fleet_client.api import path_guides_api
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
    api_instance = path_guides_api.PathGuidesApi(api_client)
    path_guide_guid = "path_guide_guid_example" # str | The path_guide_guid to delete
    guid = "guid_example" # str | The guid to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /path_guides/{path_guide_guid}/positions/{guid}
        api_instance.path_guides_path_guide_guid_positions_guid_delete(path_guide_guid, guid)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PathGuidesApi->path_guides_path_guide_guid_positions_guid_delete: %s\n" % e)
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
from mir_fleet_client.api import path_guides_api
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
    api_instance = path_guides_api.PathGuidesApi(api_client)
    path_guide_guid = "path_guide_guid_example" # str | The path_guide_guid to search for
    guid = "guid_example" # str | The guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /path_guides/{path_guide_guid}/positions/{guid}
        api_response = api_instance.path_guides_path_guide_guid_positions_guid_get(path_guide_guid, guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PathGuidesApi->path_guides_path_guide_guid_positions_guid_get: %s\n" % e)
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
from mir_fleet_client.api import path_guides_api
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
    api_instance = path_guides_api.PathGuidesApi(api_client)
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
        print("Exception when calling PathGuidesApi->path_guides_path_guide_guid_positions_guid_put: %s\n" % e)
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
from mir_fleet_client.api import path_guides_api
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
    api_instance = path_guides_api.PathGuidesApi(api_client)
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
        print("Exception when calling PathGuidesApi->path_guides_path_guide_guid_positions_post: %s\n" % e)
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

# **path_guides_post**
> GetPathGuides path_guides_post(path_guides)

POST /path_guides

Add a new path guide

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import path_guides_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.post_path_guides import PostPathGuides
from mir_fleet_client.model.get_path_guides import GetPathGuides
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
    api_instance = path_guides_api.PathGuidesApi(api_client)
    path_guides = PostPathGuides(
        created_by_id="created_by_id_example",
        guid="guid_example",
        map_id="map_id_example",
        name="name_example",
    ) # PostPathGuides | The details of the path_guides

    # example passing only required values which don't have defaults set
    try:
        # POST /path_guides
        api_response = api_instance.path_guides_post(path_guides)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PathGuidesApi->path_guides_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **path_guides** | [**PostPathGuides**](PostPathGuides.md)| The details of the path_guides |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetPathGuides**](GetPathGuides.md)

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

