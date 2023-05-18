# mir_fleet_client.EvacuationsApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**evacuations_get**](EvacuationsApi.md#evacuations_get) | **GET** /evacuations | GET /evacuations
[**evacuations_id_get**](EvacuationsApi.md#evacuations_id_get) | **GET** /evacuations/{id} | GET /evacuations/{id}
[**evacuations_post**](EvacuationsApi.md#evacuations_post) | **POST** /evacuations | POST /evacuations


# **evacuations_get**
> [GetEvacuations] evacuations_get()

GET /evacuations

Retrieve evacuation state

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import evacuations_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_evacuations import GetEvacuations
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
    api_instance = evacuations_api.EvacuationsApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /evacuations
        api_response = api_instance.evacuations_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling EvacuationsApi->evacuations_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetEvacuations]**](GetEvacuations.md)

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

# **evacuations_id_get**
> GetEvacuation evacuations_id_get(id)

GET /evacuations/{id}

Retrieve a specific evacuation activation

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import evacuations_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_evacuation import GetEvacuation
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
    api_instance = evacuations_api.EvacuationsApi(api_client)
    id = 1 # int | The id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /evacuations/{id}
        api_response = api_instance.evacuations_id_get(id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling EvacuationsApi->evacuations_id_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **id** | **int**| The id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetEvacuation**](GetEvacuation.md)

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

# **evacuations_post**
> GetEvacuations evacuations_post(evacuations)

POST /evacuations

Set evactuation state

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import evacuations_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_evacuations import GetEvacuations
from mir_fleet_client.model.post_evacuations import PostEvacuations
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
    api_instance = evacuations_api.EvacuationsApi(api_client)
    evacuations = PostEvacuations(
        evacuation_activated=True,
        note="note_example",
    ) # PostEvacuations | The details of the evacuations

    # example passing only required values which don't have defaults set
    try:
        # POST /evacuations
        api_response = api_instance.evacuations_post(evacuations)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling EvacuationsApi->evacuations_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **evacuations** | [**PostEvacuations**](PostEvacuations.md)| The details of the evacuations |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetEvacuations**](GetEvacuations.md)

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

