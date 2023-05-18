# mir_fleet_client.ActionDefinitionsApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**area_events_action_definitions_action_type_get**](ActionDefinitionsApi.md#area_events_action_definitions_action_type_get) | **GET** /area_events/action_definitions/{action_type} | GET /area_events/action_definitions/{action_type}
[**area_events_action_definitions_get**](ActionDefinitionsApi.md#area_events_action_definitions_get) | **GET** /area_events/action_definitions | GET /area_events/action_definitions


# **area_events_action_definitions_action_type_get**
> GetAreaActionDefinition area_events_action_definitions_action_type_get(action_type)

GET /area_events/action_definitions/{action_type}

Retrieve the details about the action. It displays the parameters of the action and the limits for the values among others

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import action_definitions_api
from mir_fleet_client.model.get_area_action_definition import GetAreaActionDefinition
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
    api_instance = action_definitions_api.ActionDefinitionsApi(api_client)
    action_type = "action_type_example" # str | The action_type to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /area_events/action_definitions/{action_type}
        api_response = api_instance.area_events_action_definitions_action_type_get(action_type)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ActionDefinitionsApi->area_events_action_definitions_action_type_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **action_type** | **str**| The action_type to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetAreaActionDefinition**](GetAreaActionDefinition.md)

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

# **area_events_action_definitions_get**
> GetAreaActionDefinitions area_events_action_definitions_get()

GET /area_events/action_definitions

Retrieve definitions of area actions and their parameters

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import action_definitions_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_area_action_definitions import GetAreaActionDefinitions
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
    api_instance = action_definitions_api.ActionDefinitionsApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /area_events/action_definitions
        api_response = api_instance.area_events_action_definitions_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ActionDefinitionsApi->area_events_action_definitions_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetAreaActionDefinitions**](GetAreaActionDefinitions.md)

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

