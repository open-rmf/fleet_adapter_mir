# mir_fleet_client.MetricsApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**metrics_get**](MetricsApi.md#metrics_get) | **GET** /metrics | GET /metrics


# **metrics_get**
> bool, date, datetime, dict, float, int, list, str, none_type metrics_get()

GET /metrics

Retrieve the latests MiR metrics related to the given MiR product in the Prometheus or OpenMetrics text format. Default: OpenMetrics.

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import metrics_api
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
    api_instance = metrics_api.MetricsApi(api_client)
    accept = [
        "application/openmetrics-text",
    ] # [str] | The response content types accepted by the client (optional)

    # example passing only required values which don't have defaults set
    try:
        # GET /metrics
        api_response = api_instance.metrics_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MetricsApi->metrics_get: %s\n" % e)

    # example passing only required values which don't have defaults set
    # and optional values
    try:
        # GET /metrics
        api_response = api_instance.metrics_get(accept=accept)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MetricsApi->metrics_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"
 **accept** | **[str]**| The response content types accepted by the client | [optional]

### Return type

**bool, date, datetime, dict, float, int, list, str, none_type**

### Authorization

[Basic](../README.md#Basic)

### HTTP request headers

 - **Content-Type**: Not defined
 - **Accept**: text/plain, application/openmetrics-text


### HTTP response details

| Status code | Description | Response headers |
|-------------|-------------|------------------|
**200** | Successfully retrieve the specified element |  -  |
**400** | Invalid ordering or Invalid filters or Wrong output fields or Invalid limits |  -  |
**404** | Not found |  -  |

[[Back to top]](#) [[Back to API list]](../README.md#documentation-for-api-endpoints) [[Back to Model list]](../README.md#documentation-for-models) [[Back to README]](../README.md)

