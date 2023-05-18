# mir_fleet_client.MeApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**users_me_get**](MeApi.md#users_me_get) | **GET** /users/me | GET /users/me
[**users_me_permissions_get**](MeApi.md#users_me_permissions_get) | **GET** /users/me/permissions | GET /users/me/permissions
[**users_me_put**](MeApi.md#users_me_put) | **PUT** /users/me | PUT /users/me


# **users_me_get**
> [GetMe] users_me_get()

GET /users/me

Retrieve the details about the user currently authorized in the API

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import me_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_me import GetMe
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
    api_instance = me_api.MeApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /users/me
        api_response = api_instance.users_me_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MeApi->users_me_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetMe]**](GetMe.md)

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

# **users_me_permissions_get**
> [GetUserMePermissions] users_me_permissions_get()

GET /users/me/permissions

Retrieve the permission of the user currently authorized in the API

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import me_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_user_me_permissions import GetUserMePermissions
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
    api_instance = me_api.MeApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /users/me/permissions
        api_response = api_instance.users_me_permissions_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MeApi->users_me_permissions_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetUserMePermissions]**](GetUserMePermissions.md)

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

# **users_me_put**
> GetMe users_me_put(me)

PUT /users/me

Modify the values of the user currently authorized in the API

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import me_api
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.get_me import GetMe
from mir_fleet_client.model.put_me import PutMe
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
    api_instance = me_api.MeApi(api_client)
    me = PutMe(
        dashboard_id="dashboard_id_example",
        email="email_example",
        name="name_example",
        password="password_example",
        pincode="pincode_example",
        single_dashboard=True,
        user_group_id="user_group_id_example",
        username="username_example",
    ) # PutMe | The new values of the me

    # example passing only required values which don't have defaults set
    try:
        # PUT /users/me
        api_response = api_instance.users_me_put(me)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling MeApi->users_me_put: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **me** | [**PutMe**](PutMe.md)| The new values of the me |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetMe**](GetMe.md)

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

