# mir_fleet_client.PathGuidesPrecalcApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**path_guides_precalc_get**](PathGuidesPrecalcApi.md#path_guides_precalc_get) | **GET** /path_guides_precalc | GET /path_guides_precalc
[**path_guides_precalc_post**](PathGuidesPrecalcApi.md#path_guides_precalc_post) | **POST** /path_guides_precalc | POST /path_guides_precalc


# **path_guides_precalc_get**
> GetPathGuidesPrecalc path_guides_precalc_get()

GET /path_guides_precalc

Retrieve the status of path guides precalculation

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import path_guides_precalc_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_path_guides_precalc import GetPathGuidesPrecalc
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
    api_instance = path_guides_precalc_api.PathGuidesPrecalcApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /path_guides_precalc
        api_response = api_instance.path_guides_precalc_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PathGuidesPrecalcApi->path_guides_precalc_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetPathGuidesPrecalc**](GetPathGuidesPrecalc.md)

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

# **path_guides_precalc_post**
> GetPathGuidesPrecalc path_guides_precalc_post(path_guides_precalc)

POST /path_guides_precalc

Start/stop precalculation of the specified path guide

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import path_guides_precalc_api
from mir_fleet_client.model.post_path_guides_precalc import PostPathGuidesPrecalc
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_path_guides_precalc import GetPathGuidesPrecalc
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
    api_instance = path_guides_precalc_api.PathGuidesPrecalcApi(api_client)
    path_guides_precalc = PostPathGuidesPrecalc(
        command="command_example",
        guid="guid_example",
    ) # PostPathGuidesPrecalc | The details of the path_guides_precalc

    # example passing only required values which don't have defaults set
    try:
        # POST /path_guides_precalc
        api_response = api_instance.path_guides_precalc_post(path_guides_precalc)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling PathGuidesPrecalcApi->path_guides_precalc_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **path_guides_precalc** | [**PostPathGuidesPrecalc**](PostPathGuidesPrecalc.md)| The details of the path_guides_precalc |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetPathGuidesPrecalc**](GetPathGuidesPrecalc.md)

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

