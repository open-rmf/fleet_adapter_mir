# mir_fleet_client.LogsApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**software_logs_get**](LogsApi.md#software_logs_get) | **GET** /software/logs | GET /software/logs
[**software_logs_guid_get**](LogsApi.md#software_logs_guid_get) | **GET** /software/logs/{guid} | GET /software/logs/{guid}


# **software_logs_get**
> [GetSoftwareLogs] software_logs_get()

GET /software/logs

Retrieve the list of software upgrade logs

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import logs_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_software_logs import GetSoftwareLogs
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
    api_instance = logs_api.LogsApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /software/logs
        api_response = api_instance.software_logs_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling LogsApi->software_logs_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetSoftwareLogs]**](GetSoftwareLogs.md)

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

# **software_logs_guid_get**
> GetSoftwareLog software_logs_guid_get(guid)

GET /software/logs/{guid}

Retrieve the details about the software upgrade log with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import logs_api
from mir_fleet_client.model.get_software_log import GetSoftwareLog
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
    api_instance = logs_api.LogsApi(api_client)
    guid = "guid_example" # str | The guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /software/logs/{guid}
        api_response = api_instance.software_logs_guid_get(guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling LogsApi->software_logs_guid_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetSoftwareLog**](GetSoftwareLog.md)

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

