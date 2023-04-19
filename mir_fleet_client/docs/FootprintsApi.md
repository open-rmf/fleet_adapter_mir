# mir_fleet_client.FootprintsApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**footprints_get**](FootprintsApi.md#footprints_get) | **GET** /footprints | GET /footprints
[**footprints_guid_delete**](FootprintsApi.md#footprints_guid_delete) | **DELETE** /footprints/{guid} | DELETE /footprints/{guid}
[**footprints_guid_get**](FootprintsApi.md#footprints_guid_get) | **GET** /footprints/{guid} | GET /footprints/{guid}
[**footprints_guid_put**](FootprintsApi.md#footprints_guid_put) | **PUT** /footprints/{guid} | PUT /footprints/{guid}
[**footprints_post**](FootprintsApi.md#footprints_post) | **POST** /footprints | POST /footprints


# **footprints_get**
> [GetFootprints] footprints_get()

GET /footprints

Retrieve a list of footprints currently stored

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import footprints_api
from mir_fleet_client.model.get_footprints import GetFootprints
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
    api_instance = footprints_api.FootprintsApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /footprints
        api_response = api_instance.footprints_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling FootprintsApi->footprints_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetFootprints]**](GetFootprints.md)

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

# **footprints_guid_delete**
> footprints_guid_delete(guid)

DELETE /footprints/{guid}

Delete with a specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import footprints_api
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
    api_instance = footprints_api.FootprintsApi(api_client)
    guid = "guid_example" # str | The guid to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /footprints/{guid}
        api_instance.footprints_guid_delete(guid)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling FootprintsApi->footprints_guid_delete: %s\n" % e)
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

# **footprints_guid_get**
> GetFootprint footprints_guid_get(guid)

GET /footprints/{guid}

Retrieve information about a footprint with specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import footprints_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_footprint import GetFootprint
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
    api_instance = footprints_api.FootprintsApi(api_client)
    guid = "guid_example" # str | The guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /footprints/{guid}
        api_response = api_instance.footprints_guid_get(guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling FootprintsApi->footprints_guid_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetFootprint**](GetFootprint.md)

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

# **footprints_guid_put**
> GetFootprint footprints_guid_put(guid, footprint)

PUT /footprints/{guid}

Modify a footprint with a specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import footprints_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_footprint import GetFootprint
from mir_fleet_client.model.put_footprint import PutFootprint
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
    api_instance = footprints_api.FootprintsApi(api_client)
    guid = "guid_example" # str | The guid to modify
    footprint = PutFootprint(
        config_id="config_id_example",
        footprint_points="footprint_points_example",
        height=3.14,
        hook=True,
        name="name_example",
    ) # PutFootprint | The new values of the footprint

    # example passing only required values which don't have defaults set
    try:
        # PUT /footprints/{guid}
        api_response = api_instance.footprints_guid_put(guid, footprint)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling FootprintsApi->footprints_guid_put: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to modify |
 **footprint** | [**PutFootprint**](PutFootprint.md)| The new values of the footprint |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetFootprint**](GetFootprint.md)

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

# **footprints_post**
> GetFootprints footprints_post(footprints)

POST /footprints

Add new footprint to the database

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import footprints_api
from mir_fleet_client.model.get_footprints import GetFootprints
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.post_footprints import PostFootprints
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
    api_instance = footprints_api.FootprintsApi(api_client)
    footprints = PostFootprints(
        config_id="config_id_example",
        created_by_id="created_by_id_example",
        footprint_points="footprint_points_example",
        guid="guid_example",
        height=3.14,
        hook=True,
        name="name_example",
    ) # PostFootprints | The details of the footprints

    # example passing only required values which don't have defaults set
    try:
        # POST /footprints
        api_response = api_instance.footprints_post(footprints)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling FootprintsApi->footprints_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **footprints** | [**PostFootprints**](PostFootprints.md)| The details of the footprints |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetFootprints**](GetFootprints.md)

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

