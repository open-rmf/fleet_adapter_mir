# mir_fleet_client.PathGuidesPositionsApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**path_guides_positions_get**](PathGuidesPositionsApi.md#path_guides_positions_get) | **GET** /path_guides_positions | GET /path_guides_positions
[**path_guides_positions_guid_delete**](PathGuidesPositionsApi.md#path_guides_positions_guid_delete) | **DELETE** /path_guides_positions/{guid} | DELETE /path_guides_positions/{guid}
[**path_guides_positions_guid_get**](PathGuidesPositionsApi.md#path_guides_positions_guid_get) | **GET** /path_guides_positions/{guid} | GET /path_guides_positions/{guid}
[**path_guides_positions_guid_put**](PathGuidesPositionsApi.md#path_guides_positions_guid_put) | **PUT** /path_guides_positions/{guid} | PUT /path_guides_positions/{guid}
[**path_guides_positions_post**](PathGuidesPositionsApi.md#path_guides_positions_post) | **POST** /path_guides_positions | POST /path_guides_positions


# **path_guides_positions_get**
> [GetPathGuidesPositions] path_guides_positions_get()

GET /path_guides_positions

Retrieve the list of positions used for path guides

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import path_guides_positions_api
from mir_fleet_client.model.get_path_guides_positions import GetPathGuidesPositions
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
    api_instance = path_guides_positions_api.PathGuidesPositionsApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /path_guides_positions
        api_response = api_instance.path_guides_positions_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PathGuidesPositionsApi->path_guides_positions_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetPathGuidesPositions]**](GetPathGuidesPositions.md)

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

# **path_guides_positions_guid_delete**
> path_guides_positions_guid_delete(guid)

DELETE /path_guides_positions/{guid}

Erase the path guide position with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import path_guides_positions_api
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
    api_instance = path_guides_positions_api.PathGuidesPositionsApi(api_client)
    guid = "guid_example" # str | The guid to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /path_guides_positions/{guid}
        api_instance.path_guides_positions_guid_delete(guid)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PathGuidesPositionsApi->path_guides_positions_guid_delete: %s\n" % e)
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

# **path_guides_positions_guid_get**
> GetPathGuidesPosition path_guides_positions_guid_get(guid)

GET /path_guides_positions/{guid}

Retrieve the position for path guides with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import path_guides_positions_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_path_guides_position import GetPathGuidesPosition
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
    api_instance = path_guides_positions_api.PathGuidesPositionsApi(api_client)
    guid = "guid_example" # str | The guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /path_guides_positions/{guid}
        api_response = api_instance.path_guides_positions_guid_get(guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PathGuidesPositionsApi->path_guides_positions_guid_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetPathGuidesPosition**](GetPathGuidesPosition.md)

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

# **path_guides_positions_guid_put**
> GetPathGuidesPosition path_guides_positions_guid_put(guid, path_guides_position)

PUT /path_guides_positions/{guid}

Modify the values of the position for path guides with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import path_guides_positions_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.put_path_guides_position import PutPathGuidesPosition
from mir_fleet_client.model.get_path_guides_position import GetPathGuidesPosition
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
    api_instance = path_guides_positions_api.PathGuidesPositionsApi(api_client)
    guid = "guid_example" # str | The guid to modify
    path_guides_position = PutPathGuidesPosition(
        pos_guid="pos_guid_example",
        priority=1,
    ) # PutPathGuidesPosition | The new values of the path_guides_position

    # example passing only required values which don't have defaults set
    try:
        # PUT /path_guides_positions/{guid}
        api_response = api_instance.path_guides_positions_guid_put(guid, path_guides_position)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PathGuidesPositionsApi->path_guides_positions_guid_put: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to modify |
 **path_guides_position** | [**PutPathGuidesPosition**](PutPathGuidesPosition.md)| The new values of the path_guides_position |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetPathGuidesPosition**](GetPathGuidesPosition.md)

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

# **path_guides_positions_post**
> GetPathGuidesPositions path_guides_positions_post(path_guides_positions)

POST /path_guides_positions

Add a new position in a path guide

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import path_guides_positions_api
from mir_fleet_client.model.post_path_guides_positions import PostPathGuidesPositions
from mir_fleet_client.model.get_path_guides_positions import GetPathGuidesPositions
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
    api_instance = path_guides_positions_api.PathGuidesPositionsApi(api_client)
    path_guides_positions = PostPathGuidesPositions(
        guid="guid_example",
        path_guide_guid="path_guide_guid_example",
        pos_guid="pos_guid_example",
        pos_type="pos_type_example",
        priority=1,
    ) # PostPathGuidesPositions | The details of the path_guides_positions

    # example passing only required values which don't have defaults set
    try:
        # POST /path_guides_positions
        api_response = api_instance.path_guides_positions_post(path_guides_positions)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PathGuidesPositionsApi->path_guides_positions_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **path_guides_positions** | [**PostPathGuidesPositions**](PostPathGuidesPositions.md)| The details of the path_guides_positions |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetPathGuidesPositions**](GetPathGuidesPositions.md)

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

