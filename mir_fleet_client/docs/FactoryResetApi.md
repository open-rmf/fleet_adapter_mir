# mir_fleet_client.FactoryResetApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**factory_reset_post**](FactoryResetApi.md#factory_reset_post) | **POST** /factory_reset | POST /factory_reset


# **factory_reset_post**
> bool, date, datetime, dict, float, int, list, str, none_type factory_reset_post(factory_reset_from_fleet)

POST /factory_reset

Clean and migrate the database. Keep hardware configurations

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import factory_reset_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.post_factory_reset_from_fleet import PostFactoryResetFromFleet
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
    api_instance = factory_reset_api.FactoryResetApi(api_client)
    factory_reset_from_fleet = PostFactoryResetFromFleet(
        robot_id=1,
    ) # PostFactoryResetFromFleet | The details of the factory_reset_from_fleet

    # example passing only required values which don't have defaults set
    try:
        # POST /factory_reset
        api_response = api_instance.factory_reset_post(factory_reset_from_fleet)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling FactoryResetApi->factory_reset_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **factory_reset_from_fleet** | [**PostFactoryResetFromFleet**](PostFactoryResetFromFleet.md)| The details of the factory_reset_from_fleet |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

**bool, date, datetime, dict, float, int, list, str, none_type**

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

