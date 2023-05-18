# mir_fleet_client.CartsApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**carts_get**](CartsApi.md#carts_get) | **GET** /carts | GET /carts
[**carts_guid_delete**](CartsApi.md#carts_guid_delete) | **DELETE** /carts/{guid} | DELETE /carts/{guid}
[**carts_guid_get**](CartsApi.md#carts_guid_get) | **GET** /carts/{guid} | GET /carts/{guid}
[**carts_guid_put**](CartsApi.md#carts_guid_put) | **PUT** /carts/{guid} | PUT /carts/{guid}
[**carts_post**](CartsApi.md#carts_post) | **POST** /carts | POST /carts


# **carts_get**
> [GetCarts] carts_get()

GET /carts

Retrieve the list of carts

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import carts_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_carts import GetCarts
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
    api_instance = carts_api.CartsApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /carts
        api_response = api_instance.carts_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling CartsApi->carts_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetCarts]**](GetCarts.md)

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

# **carts_guid_delete**
> carts_guid_delete(guid)

DELETE /carts/{guid}

Erase the cart with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import carts_api
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
    api_instance = carts_api.CartsApi(api_client)
    guid = "guid_example" # str | The guid to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /carts/{guid}
        api_instance.carts_guid_delete(guid)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling CartsApi->carts_guid_delete: %s\n" % e)
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

# **carts_guid_get**
> GetCart carts_guid_get(guid)

GET /carts/{guid}

Retrieve the details about the cart with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import carts_api
from mir_fleet_client.model.get_cart import GetCart
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
    api_instance = carts_api.CartsApi(api_client)
    guid = "guid_example" # str | The guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /carts/{guid}
        api_response = api_instance.carts_guid_get(guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling CartsApi->carts_guid_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetCart**](GetCart.md)

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

# **carts_guid_put**
> GetCart carts_guid_put(guid, cart)

PUT /carts/{guid}

Modify the values of the cart with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import carts_api
from mir_fleet_client.model.put_cart import PutCart
from mir_fleet_client.model.get_cart import GetCart
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
    api_instance = carts_api.CartsApi(api_client)
    guid = "guid_example" # str | The guid to modify
    cart = PutCart(
        cart_calibration_id="cart_calibration_id_example",
        cart_type_id="cart_type_id_example",
        name="name_example",
    ) # PutCart | The new values of the cart

    # example passing only required values which don't have defaults set
    try:
        # PUT /carts/{guid}
        api_response = api_instance.carts_guid_put(guid, cart)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling CartsApi->carts_guid_put: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to modify |
 **cart** | [**PutCart**](PutCart.md)| The new values of the cart |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetCart**](GetCart.md)

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

# **carts_post**
> GetCarts carts_post(carts)

POST /carts

Add a new cart

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import carts_api
from mir_fleet_client.model.post_carts import PostCarts
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_carts import GetCarts
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
    api_instance = carts_api.CartsApi(api_client)
    carts = PostCarts(
        cart_calibration_id="cart_calibration_id_example",
        cart_type_id="cart_type_id_example",
        created_by_id="created_by_id_example",
        guid="guid_example",
        name="name_example",
    ) # PostCarts | The details of the carts

    # example passing only required values which don't have defaults set
    try:
        # POST /carts
        api_response = api_instance.carts_post(carts)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling CartsApi->carts_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **carts** | [**PostCarts**](PostCarts.md)| The details of the carts |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetCarts**](GetCarts.md)

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

