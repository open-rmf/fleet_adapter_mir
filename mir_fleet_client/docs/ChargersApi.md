# mir_fleet_client.ChargersApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**chargers_charger_id_charging_groups_get**](ChargersApi.md#chargers_charger_id_charging_groups_get) | **GET** /chargers/{charger_id}/charging_groups | GET /chargers/{charger_id}/charging_groups
[**charging_groups_group_id_chargers_charger_id_delete**](ChargersApi.md#charging_groups_group_id_chargers_charger_id_delete) | **DELETE** /charging_groups/{group_id}/chargers/{charger_id} | DELETE /charging_groups/{group_id}/chargers/{charger_id}
[**charging_groups_group_id_chargers_charger_id_get**](ChargersApi.md#charging_groups_group_id_chargers_charger_id_get) | **GET** /charging_groups/{group_id}/chargers/{charger_id} | GET /charging_groups/{group_id}/chargers/{charger_id}
[**charging_groups_group_id_chargers_get**](ChargersApi.md#charging_groups_group_id_chargers_get) | **GET** /charging_groups/{group_id}/chargers | GET /charging_groups/{group_id}/chargers
[**charging_groups_group_id_chargers_post**](ChargersApi.md#charging_groups_group_id_chargers_post) | **POST** /charging_groups/{group_id}/chargers | POST /charging_groups/{group_id}/chargers


# **chargers_charger_id_charging_groups_get**
> GetChargerChargingGroups chargers_charger_id_charging_groups_get(charger_id)

GET /chargers/{charger_id}/charging_groups

Retrieve list of charging groups associated with the charger with the specified ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import chargers_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_charger_charging_groups import GetChargerChargingGroups
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
    api_instance = chargers_api.ChargersApi(api_client)
    charger_id = "charger_id_example" # str | The charger_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /chargers/{charger_id}/charging_groups
        api_response = api_instance.chargers_charger_id_charging_groups_get(charger_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ChargersApi->chargers_charger_id_charging_groups_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **charger_id** | **str**| The charger_id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetChargerChargingGroups**](GetChargerChargingGroups.md)

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

# **charging_groups_group_id_chargers_charger_id_delete**
> charging_groups_group_id_chargers_charger_id_delete(group_id, charger_id)

DELETE /charging_groups/{group_id}/chargers/{charger_id}

Delete the charger with the specified charger GUID from the charging group with the specified charging group ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import chargers_api
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
    api_instance = chargers_api.ChargersApi(api_client)
    group_id = 1 # int | The group_id to delete
    charger_id = "charger_id_example" # str | The charger_id to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /charging_groups/{group_id}/chargers/{charger_id}
        api_instance.charging_groups_group_id_chargers_charger_id_delete(group_id, charger_id)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ChargersApi->charging_groups_group_id_chargers_charger_id_delete: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **group_id** | **int**| The group_id to delete |
 **charger_id** | **str**| The charger_id to delete |
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

# **charging_groups_group_id_chargers_charger_id_get**
> GetCharger charging_groups_group_id_chargers_charger_id_get(group_id, charger_id)

GET /charging_groups/{group_id}/chargers/{charger_id}

Retrieve the details about the charger with the specified charger GUID associated with the charging group with the specified charging group ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import chargers_api
from mir_fleet_client.model.get_charger import GetCharger
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
    api_instance = chargers_api.ChargersApi(api_client)
    group_id = 1 # int | The group_id to search for
    charger_id = "charger_id_example" # str | The charger_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /charging_groups/{group_id}/chargers/{charger_id}
        api_response = api_instance.charging_groups_group_id_chargers_charger_id_get(group_id, charger_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ChargersApi->charging_groups_group_id_chargers_charger_id_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **group_id** | **int**| The group_id to search for |
 **charger_id** | **str**| The charger_id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetCharger**](GetCharger.md)

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

# **charging_groups_group_id_chargers_get**
> [GetChargers] charging_groups_group_id_chargers_get(group_id)

GET /charging_groups/{group_id}/chargers

Retrieve the list of chargers associated with the charging group with the specified charging group ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import chargers_api
from mir_fleet_client.model.get_chargers import GetChargers
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
    api_instance = chargers_api.ChargersApi(api_client)
    group_id = 1 # int | The group_id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /charging_groups/{group_id}/chargers
        api_response = api_instance.charging_groups_group_id_chargers_get(group_id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ChargersApi->charging_groups_group_id_chargers_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **group_id** | **int**| The group_id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetChargers]**](GetChargers.md)

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

# **charging_groups_group_id_chargers_post**
> GetChargers charging_groups_group_id_chargers_post(group_id, chargers)

POST /charging_groups/{group_id}/chargers

Add new charger to the charging group with the specified ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import chargers_api
from mir_fleet_client.model.get_chargers import GetChargers
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.post_chargers import PostChargers
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
    api_instance = chargers_api.ChargersApi(api_client)
    group_id = 1 # int | The group_id to add the new resource to
    chargers = PostChargers(
        charger_id="charger_id_example",
        group_id=1,
    ) # PostChargers | The details of the chargers

    # example passing only required values which don't have defaults set
    try:
        # POST /charging_groups/{group_id}/chargers
        api_response = api_instance.charging_groups_group_id_chargers_post(group_id, chargers)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ChargersApi->charging_groups_group_id_chargers_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **group_id** | **int**| The group_id to add the new resource to |
 **chargers** | [**PostChargers**](PostChargers.md)| The details of the chargers |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetChargers**](GetChargers.md)

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

