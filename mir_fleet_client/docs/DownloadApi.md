# mir_fleet_client.DownloadApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**log_error_reports_id_download_get**](DownloadApi.md#log_error_reports_id_download_get) | **GET** /log/error_reports/{id}/download | GET /log/error_reports/{id}/download


# **log_error_reports_id_download_get**
> bool, date, datetime, dict, float, int, list, str, none_type log_error_reports_id_download_get(id)

GET /log/error_reports/{id}/download

Download the file containing the error report with the specified ID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import download_api
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
    api_instance = download_api.DownloadApi(api_client)
    id = 1 # int | The id to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /log/error_reports/{id}/download
        api_response = api_instance.log_error_reports_id_download_get(id)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling DownloadApi->log_error_reports_id_download_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **id** | **int**| The id to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

**bool, date, datetime, dict, float, int, list, str, none_type**

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

