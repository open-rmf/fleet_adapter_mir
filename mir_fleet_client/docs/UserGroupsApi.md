# mir_fleet_client.UserGroupsApi

All URIs are relative to *http://localhost*

Method | HTTP request | Description
------------- | ------------- | -------------
[**user_groups_get**](UserGroupsApi.md#user_groups_get) | **GET** /user_groups | GET /user_groups
[**user_groups_guid_delete**](UserGroupsApi.md#user_groups_guid_delete) | **DELETE** /user_groups/{guid} | DELETE /user_groups/{guid}
[**user_groups_guid_get**](UserGroupsApi.md#user_groups_guid_get) | **GET** /user_groups/{guid} | GET /user_groups/{guid}
[**user_groups_guid_put**](UserGroupsApi.md#user_groups_guid_put) | **PUT** /user_groups/{guid} | PUT /user_groups/{guid}
[**user_groups_post**](UserGroupsApi.md#user_groups_post) | **POST** /user_groups | POST /user_groups
[**user_groups_user_group_guid_permissions_get**](UserGroupsApi.md#user_groups_user_group_guid_permissions_get) | **GET** /user_groups/{user_group_guid}/permissions | GET /user_groups/{user_group_guid}/permissions
[**user_groups_user_group_guid_permissions_post**](UserGroupsApi.md#user_groups_user_group_guid_permissions_post) | **POST** /user_groups/{user_group_guid}/permissions | POST /user_groups/{user_group_guid}/permissions


# **user_groups_get**
> [GetUserGroups] user_groups_get()

GET /user_groups

Retrieve the list of user groups

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import user_groups_api
from mir_fleet_client.model.get_user_groups import GetUserGroups
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
    api_instance = user_groups_api.UserGroupsApi(api_client)

    # example passing only required values which don't have defaults set
    try:
        # GET /user_groups
        api_response = api_instance.user_groups_get()
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling UserGroupsApi->user_groups_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetUserGroups]**](GetUserGroups.md)

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

# **user_groups_guid_delete**
> user_groups_guid_delete(guid)

DELETE /user_groups/{guid}

Erase the user group with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import user_groups_api
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
    api_instance = user_groups_api.UserGroupsApi(api_client)
    guid = "guid_example" # str | The guid to delete

    # example passing only required values which don't have defaults set
    try:
        # DELETE /user_groups/{guid}
        api_instance.user_groups_guid_delete(guid)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling UserGroupsApi->user_groups_guid_delete: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to delete |
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

# **user_groups_guid_get**
> GetUserGroup user_groups_guid_get(guid)

GET /user_groups/{guid}

Retrieve the details about the user group with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import user_groups_api
from mir_fleet_client.model.get_user_group import GetUserGroup
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
    api_instance = user_groups_api.UserGroupsApi(api_client)
    guid = "guid_example" # str | The guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /user_groups/{guid}
        api_response = api_instance.user_groups_guid_get(guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling UserGroupsApi->user_groups_guid_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetUserGroup**](GetUserGroup.md)

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

# **user_groups_guid_put**
> GetUserGroup user_groups_guid_put(guid, user_group)

PUT /user_groups/{guid}

Modify the values of the user group with the specified GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import user_groups_api
from mir_fleet_client.model.get_user_group import GetUserGroup
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.put_user_group import PutUserGroup
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
    api_instance = user_groups_api.UserGroupsApi(api_client)
    guid = "guid_example" # str | The guid to modify
    user_group = PutUserGroup(
        name="name_example",
    ) # PutUserGroup | The new values of the user_group

    # example passing only required values which don't have defaults set
    try:
        # PUT /user_groups/{guid}
        api_response = api_instance.user_groups_guid_put(guid, user_group)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling UserGroupsApi->user_groups_guid_put: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **guid** | **str**| The guid to modify |
 **user_group** | [**PutUserGroup**](PutUserGroup.md)| The new values of the user_group |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetUserGroup**](GetUserGroup.md)

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

# **user_groups_post**
> GetUserGroups user_groups_post(user_groups)

POST /user_groups

Add a new user group

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import user_groups_api
from mir_fleet_client.model.get_user_groups import GetUserGroups
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.post_user_groups import PostUserGroups
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
    api_instance = user_groups_api.UserGroupsApi(api_client)
    user_groups = PostUserGroups(
        created_by_id="created_by_id_example",
        guid="guid_example",
        name="name_example",
    ) # PostUserGroups | The details of the user_groups

    # example passing only required values which don't have defaults set
    try:
        # POST /user_groups
        api_response = api_instance.user_groups_post(user_groups)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling UserGroupsApi->user_groups_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **user_groups** | [**PostUserGroups**](PostUserGroups.md)| The details of the user_groups |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetUserGroups**](GetUserGroups.md)

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

# **user_groups_user_group_guid_permissions_get**
> [GetUserGroupPermission] user_groups_user_group_guid_permissions_get(user_group_guid)

GET /user_groups/{user_group_guid}/permissions

Retrieve the list of permissions of the user group with the specified group GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import user_groups_api
from mir_fleet_client.model.get_user_group_permission import GetUserGroupPermission
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
    api_instance = user_groups_api.UserGroupsApi(api_client)
    user_group_guid = "user_group_guid_example" # str | The user_group_guid to search for

    # example passing only required values which don't have defaults set
    try:
        # GET /user_groups/{user_group_guid}/permissions
        api_response = api_instance.user_groups_user_group_guid_permissions_get(user_group_guid)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling UserGroupsApi->user_groups_user_group_guid_permissions_get: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **user_group_guid** | **str**| The user_group_guid to search for |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**[GetUserGroupPermission]**](GetUserGroupPermission.md)

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

# **user_groups_user_group_guid_permissions_post**
> GetUserGroupPermission user_groups_user_group_guid_permissions_post(user_group_guid, user_group_permission)

POST /user_groups/{user_group_guid}/permissions

Add a new permission to the group with the specified group GUID

### Example

* Basic Authentication (Basic):

```python
import time
import mir_fleet_client
from mir_fleet_client.api import user_groups_api
from mir_fleet_client.model.get_user_group_permission import GetUserGroupPermission
from mir_fleet_client.model.error import Error
from mir_fleet_client.model.post_user_group_permission import PostUserGroupPermission
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
    api_instance = user_groups_api.UserGroupsApi(api_client)
    user_group_guid = "user_group_guid_example" # str | The user_group_guid to add the new resource to
    user_group_permission = PostUserGroupPermission(
        endpoint="endpoint_example",
        guid="guid_example",
        permission_type="permission_type_example",
        user_group_guid="user_group_guid_example",
    ) # PostUserGroupPermission | The details of the user_group_permission

    # example passing only required values which don't have defaults set
    try:
        # POST /user_groups/{user_group_guid}/permissions
        api_response = api_instance.user_groups_user_group_guid_permissions_post(user_group_guid, user_group_permission)
        pprint(api_response)
    except mir_fleet_client.ApiException as e:
        print("Exception when calling UserGroupsApi->user_groups_user_group_guid_permissions_post: %s\n" % e)
```


### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **user_group_guid** | **str**| The user_group_guid to add the new resource to |
 **user_group_permission** | [**PostUserGroupPermission**](PostUserGroupPermission.md)| The details of the user_group_permission |
 **accept_language** | **str**| Language header | defaults to "en_US"

### Return type

[**GetUserGroupPermission**](GetUserGroupPermission.md)

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

