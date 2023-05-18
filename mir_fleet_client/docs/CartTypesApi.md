# mir_fleet_client.CartTypesApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**cart_types_get**](CartTypesApi.md#cart_types_get) | **GET** /cart_types | GET /cart_types
[**cart_types_guid_delete**](CartTypesApi.md#cart_types_guid_delete) | **DELETE** /cart_types/{guid} | DELETE /cart_types/{guid}
[**cart_types_guid_get**](CartTypesApi.md#cart_types_guid_get) | **GET** /cart_types/{guid} | GET /cart_types/{guid}
[**cart_types_guid_put**](CartTypesApi.md#cart_types_guid_put) | **PUT** /cart_types/{guid} | PUT /cart_types/{guid}
[**cart_types_post**](CartTypesApi.md#cart_types_post) | **POST** /cart_types | POST /cart_types


# **cart_types_get**
> [GetCartTypes] cart_types_get()

GET /cart_types

Retrieve the list of cart types

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import cart_types_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_cart_types import GetCartTypes
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
    api_instance = cart_types_api.CartTypesApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /cart_types
        api_response = api_instance.cart_types_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling CartTypesApi->cart_types_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetCartTypes]**](GetCartTypes.md)

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

# **cart_types_guid_delete**
> cart_types_guid_delete(guid)

DELETE /cart_types/{guid}

Erase the cart type with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import cart_types_api
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
    api_instance = cart_types_api.CartTypesApi(api_client)
    guid = "guid_example" # str | The guid to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /cart_types/{guid}
        api_instance.cart_types_guid_delete(guid)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling CartTypesApi->cart_types_guid_delete: %s\n" % e)
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

# **cart_types_guid_get**
> GetCartType cart_types_guid_get(guid)

GET /cart_types/{guid}

Retrieve the details about the cart type with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import cart_types_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_cart_type import GetCartType
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
    api_instance = cart_types_api.CartTypesApi(api_client)
    guid = "guid_example" # str | The guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /cart_types/{guid}
        api_response = api_instance.cart_types_guid_get(guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling CartTypesApi->cart_types_guid_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetCartType**](GetCartType.md)

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

# **cart_types_guid_put**
> GetCartType cart_types_guid_put(guid, cart_type)

PUT /cart_types/{guid}

Modify the values of the cart type with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import cart_types_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.put_cart_type import PutCartType
from mir_fleet_client.model.get_cart_type import GetCartType
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
    api_instance = cart_types_api.CartTypesApi(api_client)
    guid = "guid_example" # str | The guid to modify
    cart_type = PutCartType(
        height=3.14,
        length=3.14,
        name="name_example",
        offset_locked_wheels=3.14,
        width=3.14,
    ) # PutCartType | The new values of the cart_type

    # example passing only required values which don't have defaults set
    try:
        # PUT /cart_types/{guid}
        api_response = api_instance.cart_types_guid_put(guid, cart_type)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling CartTypesApi->cart_types_guid_put: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to modify |
 **cart_type** | [**PutCartType**](PutCartType.md)| The new values of the cart_type |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetCartType**](GetCartType.md)

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

# **cart_types_post**
> GetCartTypes cart_types_post(cart_types)

POST /cart_types

Add a new cart type

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import cart_types_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_cart_types import GetCartTypes
from mir_fleet_client.model.post_cart_types import PostCartTypes
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
    api_instance = cart_types_api.CartTypesApi(api_client)
    cart_types = PostCartTypes(
        created_by_id="created_by_id_example",
        guid="guid_example",
        height=3.14,
        length=3.14,
        name="name_example",
        offset_locked_wheels=3.14,
        width=3.14,
    ) # PostCartTypes | The details of the cart_types

    # example passing only required values which don't have defaults set
    try:
        # POST /cart_types
        api_response = api_instance.cart_types_post(cart_types)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling CartTypesApi->cart_types_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **cart_types** | [**PostCartTypes**](PostCartTypes.md)| The details of the cart_types |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetCartTypes**](GetCartTypes.md)

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

