# mir_fleet_client.ScanApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**robots_scan_get**](ScanApi.md#robots_scan_get) | **GET** /robots/scan | GET /robots/scan
[**robots_scan_ip_get**](ScanApi.md#robots_scan_ip_get) | **GET** /robots/scan/{ip} | GET /robots/scan/{ip}


# **robots_scan_get**
> [GetRobotsScan] robots_scan_get()

GET /robots/scan

Scan for robots

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import scan_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_robots_scan import GetRobotsScan
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
    api_instance = scan_api.ScanApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /robots/scan
        api_response = api_instance.robots_scan_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ScanApi->robots_scan_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetRobotsScan]**](GetRobotsScan.md)

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

# **robots_scan_ip_get**
> GetRobotScan robots_scan_ip_get(ip)

GET /robots/scan/{ip}

Scan for robots on IP

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import scan_api
from mir_fleet_client.model.get_robot_scan import GetRobotScan
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
    api_instance = scan_api.ScanApi(api_client)
    ip = "ip_example" # str | The ip to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /robots/scan/{ip}
        api_response = api_instance.robots_scan_ip_get(ip)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ScanApi->robots_scan_ip_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **ip** | **str**| The ip to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetRobotScan**](GetRobotScan.md)

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

