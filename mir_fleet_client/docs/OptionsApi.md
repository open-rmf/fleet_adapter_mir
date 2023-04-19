# mir_fleet_client.OptionsApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**path_guides_path_guide_guid_options_get**](OptionsApi.md#path_guides_path_guide_guid_options_get) | **GET** /path_guides/{path_guide_guid}/options | GET /path_guides/{path_guide_guid}/options


# **path_guides_path_guide_guid_options_get**
> GetPathGuideOptions path_guides_path_guide_guid_options_get(path_guide_guid)

GET /path_guides/{path_guide_guid}/options

Retrieve the list of allowed start/via/goal options for the selected path guide

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import options_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_path_guide_options import GetPathGuideOptions
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
    api_instance = options_api.OptionsApi(api_client)
    path_guide_guid = "path_guide_guid_example" # str | The path_guide_guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /path_guides/{path_guide_guid}/options
        api_response = api_instance.path_guides_path_guide_guid_options_get(path_guide_guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling OptionsApi->path_guides_path_guide_guid_options_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **path_guide_guid** | **str**| The path_guide_guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetPathGuideOptions**](GetPathGuideOptions.md)

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

