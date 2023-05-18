# mir_fleet_client.ShelfsApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**docking_offsets_shelfs_get**](ShelfsApi.md#docking_offsets_shelfs_get) | **GET** /docking_offsets/shelfs | GET /docking_offsets/shelfs


# **docking_offsets_shelfs_get**
> [GetDockingOffsetsNoPos] docking_offsets_shelfs_get()

GET /docking_offsets/shelfs

Retrieve the list of docking offsets

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import shelfs_api
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
    api_instance = shelfs_api.ShelfsApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /docking_offsets/shelfs
        api_response = api_instance.docking_offsets_shelfs_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ShelfsApi->docking_offsets_shelfs_get: %s\n" % e)
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

