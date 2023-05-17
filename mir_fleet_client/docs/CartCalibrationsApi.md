# mir_fleet_client.CartCalibrationsApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**cart_calibrations_get**](CartCalibrationsApi.md#cart_calibrations_get) | **GET** /cart_calibrations | GET /cart_calibrations
[**cart_calibrations_guid_delete**](CartCalibrationsApi.md#cart_calibrations_guid_delete) | **DELETE** /cart_calibrations/{guid} | DELETE /cart_calibrations/{guid}
[**cart_calibrations_guid_get**](CartCalibrationsApi.md#cart_calibrations_guid_get) | **GET** /cart_calibrations/{guid} | GET /cart_calibrations/{guid}
[**cart_calibrations_guid_put**](CartCalibrationsApi.md#cart_calibrations_guid_put) | **PUT** /cart_calibrations/{guid} | PUT /cart_calibrations/{guid}
[**cart_calibrations_post**](CartCalibrationsApi.md#cart_calibrations_post) | **POST** /cart_calibrations | POST /cart_calibrations


# **cart_calibrations_get**
> [GetCartCalibrations] cart_calibrations_get()

GET /cart_calibrations

Retrieve the list of cart calibrations

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import cart_calibrations_api
from mir_fleet_client.model.get_cart_calibrations import GetCartCalibrations
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
    api_instance = cart_calibrations_api.CartCalibrationsApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /cart_calibrations
        api_response = api_instance.cart_calibrations_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling CartCalibrationsApi->cart_calibrations_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetCartCalibrations]**](GetCartCalibrations.md)

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

# **cart_calibrations_guid_delete**
> cart_calibrations_guid_delete(guid)

DELETE /cart_calibrations/{guid}

Erase the cart calibration with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import cart_calibrations_api
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
    api_instance = cart_calibrations_api.CartCalibrationsApi(api_client)
    guid = "guid_example" # str | The guid to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /cart_calibrations/{guid}
        api_instance.cart_calibrations_guid_delete(guid)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling CartCalibrationsApi->cart_calibrations_guid_delete: %s\n" % e)
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

# **cart_calibrations_guid_get**
> GetCartCalibration cart_calibrations_guid_get(guid)

GET /cart_calibrations/{guid}

Retrieve the details about the cart calibration with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import cart_calibrations_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_cart_calibration import GetCartCalibration
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
    api_instance = cart_calibrations_api.CartCalibrationsApi(api_client)
    guid = "guid_example" # str | The guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /cart_calibrations/{guid}
        api_response = api_instance.cart_calibrations_guid_get(guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling CartCalibrationsApi->cart_calibrations_guid_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetCartCalibration**](GetCartCalibration.md)

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

# **cart_calibrations_guid_put**
> GetCartCalibration cart_calibrations_guid_put(guid, cart_calibration)

PUT /cart_calibrations/{guid}

Modify the values of the cart calibration with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import cart_calibrations_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.put_cart_calibration import PutCartCalibration
from mir_fleet_client.model.get_cart_calibration import GetCartCalibration
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
    api_instance = cart_calibrations_api.CartCalibrationsApi(api_client)
    guid = "guid_example" # str | The guid to modify
    cart_calibration = PutCartCalibration(
        drive_height=1,
        entry_height=1,
        lock_height=1,
        name="name_example",
        qw=3.14,
        qx=3.14,
        qy=3.14,
        qz=3.14,
        standard=True,
        x=3.14,
        y=3.14,
        z=3.14,
    ) # PutCartCalibration | The new values of the cart_calibration

    # example passing only required values which don't have defaults set
    try:
        # PUT /cart_calibrations/{guid}
        api_response = api_instance.cart_calibrations_guid_put(guid, cart_calibration)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling CartCalibrationsApi->cart_calibrations_guid_put: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to modify |
 **cart_calibration** | [**PutCartCalibration**](PutCartCalibration.md)| The new values of the cart_calibration |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetCartCalibration**](GetCartCalibration.md)

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

# **cart_calibrations_post**
> GetCartCalibrations cart_calibrations_post(cart_calibrations)

POST /cart_calibrations

Add a new cart calibration

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import cart_calibrations_api
from mir_fleet_client.model.get_cart_calibrations import GetCartCalibrations
from mir_fleet_client.model.post_cart_calibrations import PostCartCalibrations
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
    api_instance = cart_calibrations_api.CartCalibrationsApi(api_client)
    cart_calibrations = PostCartCalibrations(
        created_by_id="created_by_id_example",
        drive_height=1,
        entry_height=1,
        guid="guid_example",
        lock_height=1,
        name="name_example",
        qw=3.14,
        qx=3.14,
        qy=3.14,
        qz=3.14,
        standard=True,
        x=3.14,
        y=3.14,
        z=3.14,
    ) # PostCartCalibrations | The details of the cart_calibrations

    # example passing only required values which don't have defaults set
    try:
        # POST /cart_calibrations
        api_response = api_instance.cart_calibrations_post(cart_calibrations)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling CartCalibrationsApi->cart_calibrations_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **cart_calibrations** | [**PostCartCalibrations**](PostCartCalibrations.md)| The details of the cart_calibrations |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetCartCalibrations**](GetCartCalibrations.md)

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

