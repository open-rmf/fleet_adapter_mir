# mir_fleet_client.ImportApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**sessions_import_delete**](ImportApi.md#sessions_import_delete) | **DELETE** /sessions/import | DELETE /sessions/import
[**sessions_import_get**](ImportApi.md#sessions_import_get) | **GET** /sessions/import | GET /sessions/import
[**sessions_import_post**](ImportApi.md#sessions_import_post) | **POST** /sessions/import | POST /sessions/import


# **sessions_import_delete**
> sessions_import_delete()

DELETE /sessions/import

Cancel currently running import

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import import_api
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
    api_instance = import_api.ImportApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # DELETE /sessions/import
        api_instance.sessions_import_delete()
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ImportApi->sessions_import_delete: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

void (empty response body)

### Authorization

[Basic](../README.md#Basic)

### HTTP request headers

 - **Content-Type**: Not defined
 - **Accept**: application/json


### HTTP response details

| Status code | Description | Response headers |
|-------------|-------------|------------------|
**204** | The element has been deleted successfully |  -  |
**400** | Invalid filters or Invalid JSON or Argument error or Missing content type application/json on the header or Bad request or No fields |  -  |
**404** | Not found |  -  |

[[Back to top]](#) [[Back to API list]](../README.md#documentation-for-api-endpoints) [[Back to Model list]](../README.md#documentation-for-models) [[Back to README]](../README.md)

# **sessions_import_get**
> GetSessionImport sessions_import_get()

GET /sessions/import

Get progress of the running import

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import import_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_session_import import GetSessionImport
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
    api_instance = import_api.ImportApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /sessions/import
        api_response = api_instance.sessions_import_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ImportApi->sessions_import_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetSessionImport**](GetSessionImport.md)

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

# **sessions_import_post**
> GetSessionImport sessions_import_post(session_import)

POST /sessions/import

Import the session contained in the file

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import import_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_session_import import GetSessionImport
from mir_fleet_client.model.post_session_import import PostSessionImport
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
    api_instance = import_api.ImportApi(api_client)
    session_import = PostSessionImport(
        file="file_example",
    ) # PostSessionImport | The details of the session_import

    # example passing only required values which don't have defaults set
    try:
        # POST /sessions/import
        api_response = api_instance.sessions_import_post(session_import)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling ImportApi->sessions_import_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **session_import** | [**PostSessionImport**](PostSessionImport.md)| The details of the session_import |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetSessionImport**](GetSessionImport.md)

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

