# mir_fleet_client.DockingOffsetsApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**docking_offsets_get**](DockingOffsetsApi.md#docking_offsets_get) | **GET** /docking_offsets | GET /docking_offsets
[**docking_offsets_guid_delete**](DockingOffsetsApi.md#docking_offsets_guid_delete) | **DELETE** /docking_offsets/{guid} | DELETE /docking_offsets/{guid}
[**docking_offsets_guid_get**](DockingOffsetsApi.md#docking_offsets_guid_get) | **GET** /docking_offsets/{guid} | GET /docking_offsets/{guid}
[**docking_offsets_guid_put**](DockingOffsetsApi.md#docking_offsets_guid_put) | **PUT** /docking_offsets/{guid} | PUT /docking_offsets/{guid}
[**docking_offsets_post**](DockingOffsetsApi.md#docking_offsets_post) | **POST** /docking_offsets | POST /docking_offsets
[**docking_offsets_shelfs_get**](DockingOffsetsApi.md#docking_offsets_shelfs_get) | **GET** /docking_offsets/shelfs | GET /docking_offsets/shelfs
[**docking_offsets_types_get**](DockingOffsetsApi.md#docking_offsets_types_get) | **GET** /docking_offsets/types | GET /docking_offsets/types
[**docking_offsets_types_id_get**](DockingOffsetsApi.md#docking_offsets_types_id_get) | **GET** /docking_offsets/types/{id} | GET /docking_offsets/types/{id}
[**positions_pos_id_docking_offsets_get**](DockingOffsetsApi.md#positions_pos_id_docking_offsets_get) | **GET** /positions/{pos_id}/docking_offsets | GET /positions/{pos_id}/docking_offsets


# **docking_offsets_get**
> [GetDockingOffsets] docking_offsets_get()

GET /docking_offsets

Retrieve the list of docking offsets

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import docking_offsets_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_docking_offsets import GetDockingOffsets
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
    api_instance = docking_offsets_api.DockingOffsetsApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /docking_offsets
        api_response = api_instance.docking_offsets_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling DockingOffsetsApi->docking_offsets_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetDockingOffsets]**](GetDockingOffsets.md)

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

# **docking_offsets_guid_delete**
> docking_offsets_guid_delete(guid)

DELETE /docking_offsets/{guid}

Erase the docking offset with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import docking_offsets_api
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
    api_instance = docking_offsets_api.DockingOffsetsApi(api_client)
    guid = "guid_example" # str | The guid to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /docking_offsets/{guid}
        api_instance.docking_offsets_guid_delete(guid)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling DockingOffsetsApi->docking_offsets_guid_delete: %s\n" % e)
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

# **docking_offsets_guid_get**
> GetDockingOffset docking_offsets_guid_get(guid)

GET /docking_offsets/{guid}

Retrieve the details of the docking offset with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import docking_offsets_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_docking_offset import GetDockingOffset
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
    api_instance = docking_offsets_api.DockingOffsetsApi(api_client)
    guid = "guid_example" # str | The guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /docking_offsets/{guid}
        api_response = api_instance.docking_offsets_guid_get(guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling DockingOffsetsApi->docking_offsets_guid_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetDockingOffset**](GetDockingOffset.md)

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

# **docking_offsets_guid_put**
> GetDockingOffset docking_offsets_guid_put(guid, docking_offset)

PUT /docking_offsets/{guid}

Modify the values of the docking offset with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import docking_offsets_api
from mir_fleet_client.model.put_docking_offset import PutDockingOffset
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_docking_offset import GetDockingOffset
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
    api_instance = docking_offsets_api.DockingOffsetsApi(api_client)
    guid = "guid_example" # str | The guid to modify
    docking_offset = PutDockingOffset(
        name="name_example",
        orientation_offset=3.14,
        x_offset=3.14,
        y_offset=3.14,
    ) # PutDockingOffset | The new values of the docking_offset

    # example passing only required values which don't have defaults set
    try:
        # PUT /docking_offsets/{guid}
        api_response = api_instance.docking_offsets_guid_put(guid, docking_offset)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling DockingOffsetsApi->docking_offsets_guid_put: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to modify |
 **docking_offset** | [**PutDockingOffset**](PutDockingOffset.md)| The new values of the docking_offset |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetDockingOffset**](GetDockingOffset.md)

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

# **docking_offsets_post**
> GetDockingOffsets docking_offsets_post(docking_offsets)

POST /docking_offsets

Add a new docking offset. The only positions that can have docking offsets are Charging stations, V markers and VL markers

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import docking_offsets_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.post_docking_offsets import PostDockingOffsets
from mir_fleet_client.model.get_docking_offsets import GetDockingOffsets
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
    api_instance = docking_offsets_api.DockingOffsetsApi(api_client)
    docking_offsets = PostDockingOffsets(
        created_by_id="created_by_id_example",
        docking_type=1,
        guid="guid_example",
        name="name_example",
        orientation_offset=3.14,
        pos_id="pos_id_example",
        x_offset=3.14,
        y_offset=3.14,
    ) # PostDockingOffsets | The details of the docking_offsets

    # example passing only required values which don't have defaults set
    try:
        # POST /docking_offsets
        api_response = api_instance.docking_offsets_post(docking_offsets)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling DockingOffsetsApi->docking_offsets_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **docking_offsets** | [**PostDockingOffsets**](PostDockingOffsets.md)| The details of the docking_offsets |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetDockingOffsets**](GetDockingOffsets.md)

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

# **docking_offsets_shelfs_get**
> [GetDockingOffsetsNoPos] docking_offsets_shelfs_get()

GET /docking_offsets/shelfs

Retrieve the list of docking offsets

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import docking_offsets_api
from mir_fleet_client.model.get_docking_offsets_no_pos import GetDockingOffsetsNoPos
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
    api_instance = docking_offsets_api.DockingOffsetsApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /docking_offsets/shelfs
        api_response = api_instance.docking_offsets_shelfs_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling DockingOffsetsApi->docking_offsets_shelfs_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetDockingOffsetsNoPos]**](GetDockingOffsetsNoPos.md)

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

# **docking_offsets_types_get**
> [GetDockingOffsetTypes] docking_offsets_types_get()

GET /docking_offsets/types

Retrieve a list of possible position types

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import docking_offsets_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_docking_offset_types import GetDockingOffsetTypes
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
    api_instance = docking_offsets_api.DockingOffsetsApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /docking_offsets/types
        api_response = api_instance.docking_offsets_types_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling DockingOffsetsApi->docking_offsets_types_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetDockingOffsetTypes]**](GetDockingOffsetTypes.md)

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

# **docking_offsets_types_id_get**
> GetDockingOffsetType docking_offsets_types_id_get(id)

GET /docking_offsets/types/{id}

Retrieve the details about the position type with the specified ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import docking_offsets_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_docking_offset_type import GetDockingOffsetType
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
    api_instance = docking_offsets_api.DockingOffsetsApi(api_client)
    id = 1 # int | The id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /docking_offsets/types/{id}
        api_response = api_instance.docking_offsets_types_id_get(id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling DockingOffsetsApi->docking_offsets_types_id_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **id** | **int**| The id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetDockingOffsetType**](GetDockingOffsetType.md)

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

# **positions_pos_id_docking_offsets_get**
> [GetPosDockingOffsets] positions_pos_id_docking_offsets_get(pos_id)

GET /positions/{pos_id}/docking_offsets

Retrieve the details of the docking offset of the position with the specified position ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import docking_offsets_api
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
    api_instance = docking_offsets_api.DockingOffsetsApi(api_client)
    pos_id = "pos_id_example" # str | The pos_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /positions/{pos_id}/docking_offsets
        api_response = api_instance.positions_pos_id_docking_offsets_get(pos_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling DockingOffsetsApi->positions_pos_id_docking_offsets_get: %s\n" % e)
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

