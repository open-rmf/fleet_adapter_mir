# mir_fleet_client.BlockedApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**area_events_guid_blocked_put**](BlockedApi.md#area_events_guid_blocked_put) | **PUT** /area_events/{guid}/blocked | PUT /area_events/{guid}/blocked


# **area_events_guid_blocked_put**
> GetBlockArea area_events_guid_blocked_put(guid, block_area)

PUT /area_events/{guid}/blocked

Block or unblock entry for a Limit-robots zone. While blocked robots are not allowed to enter the zone no matter the specified limit.

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import blocked_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.put_block_area import PutBlockArea
from mir_fleet_client.model.get_block_area import GetBlockArea
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
    api_instance = blocked_api.BlockedApi(api_client)
    guid = "guid_example" # str | The guid to modify
    block_area = PutBlockArea(
        block=True,
    ) # PutBlockArea | The new values of the block_area

    # example passing only required values which don't have defaults set
    try:
        # PUT /area_events/{guid}/blocked
        api_response = api_instance.area_events_guid_blocked_put(guid, block_area)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling BlockedApi->area_events_guid_blocked_put: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to modify |
 **block_area** | [**PutBlockArea**](PutBlockArea.md)| The new values of the block_area |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetBlockArea**](GetBlockArea.md)

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

