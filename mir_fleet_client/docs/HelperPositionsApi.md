# mir_fleet_client.HelperPositionsApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**positions_parent_guid_helper_positions_get**](HelperPositionsApi.md#positions_parent_guid_helper_positions_get) | **GET** /positions/{parent_guid}/helper_positions | GET /positions/{parent_guid}/helper_positions


# **positions_parent_guid_helper_positions_get**
> [GetHelperPositions] positions_parent_guid_helper_positions_get(parent_guid)

GET /positions/{parent_guid}/helper_positions

Retrieve the list of helper positions for the position with the specified parent GUID. Only Charging Stations, V markers, VL markers, Shelf and Trolley positions have helper positions

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import helper_positions_api
from mir_fleet_client.model.get_helper_positions import GetHelperPositions
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
    api_instance = helper_positions_api.HelperPositionsApi(api_client)
    parent_guid = "parent_guid_example" # str | The parent_guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /positions/{parent_guid}/helper_positions
        api_response = api_instance.positions_parent_guid_helper_positions_get(parent_guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling HelperPositionsApi->positions_parent_guid_helper_positions_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **parent_guid** | **str**| The parent_guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetHelperPositions]**](GetHelperPositions.md)

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

